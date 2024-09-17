

module Cache
    #(parameter NUM_WORDS=512, // number of words stored in cache
                NUM_BLOCKS_TO_LOAD_ON_HIT=4,
                EN_DISPLAY=1)
    (// INPUTS FROM MEM STAGE
     input logic rst_l, clk,
                 data_write, // asserted if instrution in mem stage is writing to data memory (OP_STORE)
                 data_read, // asserted if instrution in mem stage is reading from cache (OP_LOAD)
     input logic [3:0] store_mask_M,
     input logic [31:0] mem_addr, // address to read or write to (from LW, SW)
     input logic [31:0] store_data_M, // data to store in memory if SW

     // OUTPUTS TO MEM STAGE
     output logic cache_hit,    // asserted if data_addr in cache
                  cache_ready,  // aserted if cache is not currently reading from memory
     output logic [31:0] cached_data,    // Mem[data_addr] if cache_hit

     // INPUTS FROM DATA MEMORY
     input logic [1:0][31:0] data_load,  // value read from memory (used to fill cache)

     // OUTPUTS TO DATA MEMORY
     output logic [29:0] data_addr,
     output logic [31:0] data_store,
     output logic [3:0] store_mask,
     output logic data_load_en
     //
     );
    localparam G = 4; // granularity (word size)
    localparam B = 8; // block size in bytes
    localparam C = NUM_WORDS*G; // capacity in bytes (512 words * 4bytes/word)
    localparam TAG_SIZE = 32 - $clog2(C); // tag size in bits
    localparam IDX_SIZE = $clog2(C/B);  // idx size in bits
    localparam BO_SIZE = $clog2(B/G);   // block offset size in bits
    localparam G_SIZE = $clog2(G);       // # bits that g takes up
    localparam WORDS_PER_BLOCK = B/G;

    // cache line size in bits
    localparam CACHE_LINE_SIZE = TAG_SIZE + 1 + (B*8); // 1 extra bit for valid bit


    // CACHE READ SIGNALS/PARAMS ///////////////////////////////////////////////
    localparam TAG_START = 32;
    localparam TAG_STOP  = TAG_START - TAG_SIZE;

    localparam IDX_START = TAG_STOP;
    localparam IDX_STOP  = IDX_START - IDX_SIZE;

    localparam BO_START = IDX_STOP;
    localparam BO_STOP  = BO_START - BO_SIZE;

    localparam CACHELINE_TAG_START = CACHE_LINE_SIZE;
    localparam CACHELINE_TAG_STOP  = CACHE_LINE_SIZE-TAG_SIZE;

    localparam CACHELINE_VALID_IDX = CACHELINE_TAG_STOP-1;

    localparam CACHELINE_BLOCK_START = CACHELINE_VALID_IDX;
    localparam CACHELINE_BLOCK_STOP  = 0;

    logic [IDX_SIZE-1:0] received_idx;
    logic [BO_SIZE-1:0] received_block_offset;
    logic [WORDS_PER_BLOCK-1:0][(G*8)-1:0] curr_block;
    ////////////////////////////////////////////////////////////////////////////

    logic [TAG_SIZE-1:0] cached_tag, recieved_tag;
    logic                valid;
    logic [IDX_SIZE-1:0] write_idx, read_idx; // SRAM load/store addresses
    logic [CACHE_LINE_SIZE-1:0] read_data, write_data; // signals read/written to cache

    logic we, done_filling_line;

    sram #(.NUM_WORDS(C/B), .WORD_WIDTH(CACHE_LINE_SIZE))
        CACHE (.clk, .rst_l, .we,
            .read_addr(read_idx),
            .read_data,
            .write_addr(write_idx),
            .write_data);

    /***************************************************************************
    READING FROM CACHE
    ***************************************************************************/


    logic [31:0] prev_read_addr, prev_read_addr2, prev_read_addr3, prev_read_addr4;
    logic [29:0] curr_read_addr;
    // write |TAG|VALID|BLOCK OF 2 WORDS(8 bytes)|
    logic [63:0] block_to_write;
    logic [31:0] word_to_write;
    logic inc_curr_data_addr, done_filling_cache;

    assign recieved_tag =          mem_addr[TAG_START-1:TAG_STOP];
    assign received_idx =          mem_addr[IDX_START-1:IDX_STOP];
    assign received_block_offset = mem_addr[BO_START-1:BO_STOP];

    assign read_idx = inc_curr_data_addr && !done_filling_cache ? prev_read_addr[IDX_START-1:IDX_STOP] : received_idx;

    assign cached_tag = read_data[CACHELINE_TAG_START-1:CACHELINE_TAG_STOP];
    assign valid = read_data[CACHELINE_VALID_IDX];

    assign curr_block = read_data[CACHELINE_BLOCK_START-1:CACHELINE_BLOCK_STOP];

    // will not hit until in ready state
    // (prevents missing store instructions while reading from data memory to fill cache)
    assign cache_hit = valid & cached_tag == recieved_tag;
    assign cached_data = curr_block[received_block_offset];

    /***************************************************************************
    FILLING CACHE
    ***************************************************************************/

    logic en_fill_cntr, clr_fill_cntr, start_cache_fill, addr_in_stack;
    logic [$clog2(NUM_BLOCKS_TO_LOAD_ON_HIT)+1:0] fill_cntr_val;

    // Dictates how many cycles to read from cache (and how many blocks are loaded)
    Counter #(.WIDTH($bits(fill_cntr_val)))
                Fill_Cntr (.clk, .en(en_fill_cntr), .clear(clr_fill_cntr),
                           .load(1'b0), .up(1'b1),
                           .D(), .Q(fill_cntr_val));

    assign done_filling_cache = fill_cntr_val >= NUM_BLOCKS_TO_LOAD_ON_HIT-1;

    CacheFSM FSM (.rst_l, .clk,
                  .data_read, .data_write,
                  .cache_hit,
                  .addr_in_stack,
                  .done_filling_cache,
                  .we,
                  .en_fill_cntr,
                  .clr_fill_cntr,
                  .cache_ready,
                //   .start_mem_load,
                  .inc_curr_data_addr,
                  .data_load_en);

    always_comb begin

        if (received_block_offset == 1'b0) begin
            word_to_write[31:24] = store_mask_M[3] ? store_data_M[31:24] : curr_block[0][31:24];
            word_to_write[23:16] = store_mask_M[2] ? store_data_M[23:16] : curr_block[0][23:16];
            word_to_write[15:8]  = store_mask_M[1] ? store_data_M[15:8] : curr_block[0][15:8];
            word_to_write[7:0]   = store_mask_M[0] ? store_data_M[7:0] : curr_block[0][7:0];

            block_to_write = {curr_block[1], word_to_write};
        end
        else begin

            word_to_write[31:24] = store_mask_M[3] ? store_data_M[31:24] : curr_block[1][31:24];
            word_to_write[23:16] = store_mask_M[2] ? store_data_M[23:16] : curr_block[1][23:16];
            word_to_write[15:8]  = store_mask_M[1] ? store_data_M[15:8] : curr_block[1][15:8];
            word_to_write[7:0]   = store_mask_M[0] ? store_data_M[7:0]  : curr_block[1][7:0];

            block_to_write = {word_to_write, curr_block[0]};
        end
    end
    assign write_data = data_write ? {mem_addr[TAG_START-1:TAG_STOP], 1'b1,block_to_write} : {prev_read_addr4[TAG_START-1:TAG_STOP], 1'b1, data_load};
    assign write_idx  = data_write ? mem_addr[IDX_START-1:IDX_STOP] : prev_read_addr4[IDX_START-1:IDX_STOP];

    // register chain to keep track of which address that was requested is
    // curently on the output of data memory
    // TODO: make sure this is not off by one
    always_ff @(posedge clk) begin
        if (~rst_l) begin
            prev_read_addr <= 32'd0;
            prev_read_addr2 <= 32'd0;
            prev_read_addr3 <= 32'd0;
            prev_read_addr4 <= 32'd0;
        end
        else begin
            if (inc_curr_data_addr) begin
                curr_read_addr <= curr_read_addr + 30'd2;
            end
            else begin
                curr_read_addr <= mem_addr[31:2] + 30'd2;
            end
            prev_read_addr  <= inc_curr_data_addr ? {curr_read_addr, 2'b00} : {mem_addr[31:2], 2'b00};
            prev_read_addr2 <= prev_read_addr;
            prev_read_addr3 <= prev_read_addr2;
            prev_read_addr4 <= prev_read_addr3;
        end
    end


    /***************************************************************************
    WRITING TO MEMORY
    ***************************************************************************/


    import MemorySegments::STACK_END, MemorySegments::STACK_START;

    logic [31:0] stack_end, stack_start;
    assign stack_end = MemorySegments::STACK_END;
    assign stack_start = MemorySegments::STACK_START;


    always_comb begin
        if ({data_addr, 2'b00} >= STACK_START && {data_addr, 2'b00} <= STACK_END)
            addr_in_stack = 1'b1;
        else
            addr_in_stack = 1'b0; 
    end

    // only read at 8 byte allignement for blocks
    always_comb begin
        if (data_read)
            data_addr =  inc_curr_data_addr ? {curr_read_addr[29:1], 1'b0} : {mem_addr[31:3], 1'b0};
        else
            data_addr = mem_addr[31:2];
    end
    // assign
    assign store_mask = store_mask_M;
    assign data_store = store_data_M;
    // as long as store mask is set properly, should write to memory if instr_M is STORE

    `ifdef SIMULATION_18447

    generate
        if (EN_DISPLAY) begin : GENERATE_DISPLAY
            Cache_Display #(.C(C), .TAG_SIZE(TAG_SIZE), .IDX_SIZE(IDX_SIZE), .BO_SIZE(BO_SIZE), .G_SIZE(G_SIZE), .CACHE_LINE_SIZE(CACHE_LINE_SIZE))
                DSPLY (.rst_l, .clk,
                       .data_write, // asserted if instrution in mem stage is writing to data memory (OP_STORE)
                       .data_read, // asserted if instrution in mem stage is reading from cache (OP_LOAD)
                       .store_mask_M,
                       .mem_addr, // address to read or write to (from LW, SW)
                       .store_data_M,
                            // OUTPUTS TO MEM STAGE
                       .cache_hit,    // asserted if data_addr in cache
                       .cache_ready,  // aserted if cache is not currently reading from memory
                       .cached_data,    // Mem[data_addr] if cache_hit
                            // INPUTS FROM DATA MEMORY
                       .data_load,  // value read from memory (used to fill cache)
                            // OUTPUTS TO DATA MEMORY
                       .data_addr,
                       .store_mask,
                       .data_load_en,
                            // FSM STATE
                        // .data_read,
                        // .cache_hit,
                        .done_filling_cache,
                        .we,
                        .en_fill_cntr,
                        // .cache_ready,
                        .start_cache_fill,
                        .clr_fill_cntr,
                        .inc_curr_data_addr,
                        .prev_read_addr4,
                        .prev_read_addr3,
                        .prev_read_addr2,
                        .prev_read_addr,
                        .addr_in_stack);
        end
    endgenerate


     //
    `endif /* SIMULATION_18447 */

endmodule: Cache


module CacheFSM
    (input logic rst_l, clk,
                 data_read, data_write,
                 cache_hit,
                 done_filling_cache,
                 addr_in_stack,
     output logic we,
                  en_fill_cntr,
                  clr_fill_cntr,
                  cache_ready,
                  // start_cache_fill,
                  inc_curr_data_addr,
                  data_load_en);

    typedef enum logic [2:0] {READY, MEM_DELAY1, MEM_DELAY2,
                      MEM_DELAY3, FILLING_CACHE} state_t;

    state_t curr_state, next_state;
    // next state logic
    always_comb begin
        case(curr_state)
            READY: begin
                next_state = data_read && (!cache_hit) ? MEM_DELAY1 : curr_state;
            end
            MEM_DELAY1: begin
                next_state = MEM_DELAY2;
            end
            MEM_DELAY2: begin
                next_state = MEM_DELAY3;
            end
            MEM_DELAY3: begin
                next_state = FILLING_CACHE;
            end
            FILLING_CACHE: begin
                next_state = done_filling_cache || cache_hit || addr_in_stack ? READY : curr_state;
            end
            default: begin
                next_state = READY;
            end
        endcase
    end

    always_comb begin
        we = 1'b0;
        en_fill_cntr = 1'b0;
        clr_fill_cntr = 1'b0;
        cache_ready = 1'b0;
        inc_curr_data_addr = 1'b0;
        data_load_en = 1'b0;
        // start_cache_fill = 1'b0;

        case(curr_state)
            READY: begin // in this state, the cache will handle misses
                clr_fill_cntr = 1'b1;
                cache_ready = data_read && (!cache_hit) ? 1'b0 : 1'b1;
                data_load_en = data_read  ? 1'b1 : 1'b0;
                we = data_write;// & cache_hit;
                // start_cache_fill = data_read && (!cache_hit) ? 1'b1 : 1'b0;
                // inc_curr_data_addr = data_read && (!cache_hit) ? 1'b1 : 1'b0;
            end
            MEM_DELAY1: begin
                data_load_en = addr_in_stack ? 1'b0 : 1'b1;
                inc_curr_data_addr = addr_in_stack ? 1'b0 : 1'b1;
            end
            MEM_DELAY2: begin
                data_load_en = addr_in_stack ? 1'b0 : 1'b1;
                inc_curr_data_addr = addr_in_stack ? 1'b0 : 1'b1;
            end
            MEM_DELAY3: begin
                data_load_en = addr_in_stack ? 1'b0 : 1'b1;
                inc_curr_data_addr = addr_in_stack ? 1'b0 : 1'b1;
            end
            FILLING_CACHE: begin
                en_fill_cntr = 1'b1;
                we = 1'b1;
                cache_ready = done_filling_cache || addr_in_stack ? 1'b1 : 1'b0;
                data_load_en = done_filling_cache || addr_in_stack ? 1'b0 : 1'b1;
                inc_curr_data_addr = addr_in_stack ? 1'b0 : 1'b1;
            end
        endcase

    end

    always_ff @(posedge clk, negedge rst_l) begin
        if (~rst_l)
            curr_state <= READY;
        else
            curr_state <= next_state;

    end


endmodule : CacheFSM

`ifdef SIMULATION_18447
module Cache_Display
    #(parameter C, TAG_SIZE, IDX_SIZE, BO_SIZE, G_SIZE, CACHE_LINE_SIZE)
    (// INPUTS FROM MEM STAGE
     input logic rst_l, clk,
                 data_write, // asserted if instrution in mem stage is writing to data memory (OP_STORE)
                 data_read, // asserted if instrution in mem stage is reading from cache (OP_LOAD)
     input logic [3:0] store_mask_M,
     input logic [31:0] mem_addr, // address to read or write to (from LW, SW)
     input logic [31:0] store_data_M,

     // OUTPUTS TO MEM STAGE
     input logic cache_hit,    // asserted if data_addr in cache
                  cache_ready,  // aserted if cache is not currently reading from memory
     input logic [31:0] cached_data,    // Mem[data_addr] if cache_hit

     // INPUTS FROM DATA MEMORY
     input logic [1:0][31:0] data_load,  // value read from memory (used to fill cache)

     // OUTPUTS TO DATA MEMORY
     input logic [29:0] data_addr,
     input logic [3:0] store_mask,
     input logic data_load_en,
                 addr_in_stack,

     // FSM STATE
     input logic done_filling_cache,
                 we,
                 en_fill_cntr,
                 start_cache_fill,
                 inc_curr_data_addr,
                 clr_fill_cntr,

     input logic [31:0] prev_read_addr4,
                        prev_read_addr3,
                        prev_read_addr2,
                        prev_read_addr);

    localparam CACHELINE_TAG_START = CACHE_LINE_SIZE;
    localparam CACHELINE_TAG_STOP  = CACHE_LINE_SIZE-TAG_SIZE;

    localparam CACHELINE_VALID_IDX = CACHELINE_TAG_STOP-1;

    localparam CACHELINE_BLOCK_START = CACHELINE_VALID_IDX;
    localparam CACHELINE_BLOCK_STOP  = 0;

    logic [31:0] stack_end, stack_start;
    assign stack_end = MemorySegments::STACK_END;
    assign stack_start = MemorySegments::STACK_START;

    logic [31:0] cache_misses;
    logic [31:0] cache_hits;
    always_ff @(posedge clk) begin

        if (!rst_l) begin
            cache_misses <= 1'b0;
            cache_hits <= 1'b0;
        end
        else if (clr_fill_cntr && !cache_hit && data_read)
            cache_misses <= cache_misses + 32'd1;
        else if (cache_hit && data_read)
            cache_hits <= cache_hits + 32'd1;
    end

    always_ff @(posedge clk) begin
        if (rst_l) begin
                $display("CACHE INFO:");
                $display("C:%0d, TAG_SIZE: %0d, IDX_SIZE:%0d, BO_SIZE:%0d, G_SIZE, %0d", C, TAG_SIZE, IDX_SIZE, BO_SIZE, G_SIZE);
                $display("STACK_END: %08x, STACK_START: %08x", stack_end, stack_start);
                $display("---------------------------------------------------");
                $display("Inputs from Mem Stage:");
                $display("\tdata_write: %b   |   data_read: %b", data_write, data_read);
                $display("\tstore_mask_M: %b| mem_addr: 0x%08x | store_data_M: 0x%08x,", store_mask_M,mem_addr,store_data_M);
                $display("---------------------------------------------------");
                $display("Outputs to Mem Stage:");
                $display("\tcache_hit: %b | cache_ready: %b | cached_data: 0x%08x,", cache_hit, cache_ready, cached_data);
                $display("---------------------------------------------------");
                $display("Inputs from Data Memory:");
                $display("\tdata_load[0]: 0x%08x | data_load[1]: 0x%08x", data_load[0], data_load[1]);
                $display("---------------------------------------------------");
                $display("Outputs to Data Memory:");
                $display("\tdata_addr: 0x%08x | store_mask: %b | data_load_en: %b | addr_in_stack: %b",{data_addr, 2'b00}, store_mask, data_load_en, addr_in_stack);
                $display("---------------------------------------------------");
                $display("FSM State:");
                $display("\tdone_filling_cache: %b | en_fill_cntr: %b", done_filling_cache, en_fill_cntr);
                $display("\tstart_cache_fill: %b | inc_curr_data_addr: %b", start_cache_fill, inc_curr_data_addr);
                $display("\tFSM_curr_state: %s | FSM_next_state: %s", FSM.curr_state.name, FSM.next_state.name);
                $display("\tprev_read_addr4 = 0x%08x | prev_read_addr3 = 0x%08x\n\tprev_read_addr2 = 0x%08x | prev_read_addr = 0x%08x",
                    prev_read_addr4, prev_read_addr3, prev_read_addr2, prev_read_addr);
                $display("---------------------------------------------------");
                $display("SRAM Inputs:");
                $display("\tsram_read_addr: 0x%08x | sram_read_data: |Tag:0x%x|Valid:%b|Block:0x%x|",
                    CACHE.read_addr,
                    CACHE.read_data[CACHELINE_TAG_START-1:CACHELINE_TAG_STOP],
                    CACHE.read_data[CACHELINE_VALID_IDX],
                    CACHE.read_data[CACHELINE_BLOCK_START-1:CACHELINE_BLOCK_STOP]);
                $display("\tsram_we: %b | sram_write_addr: 0x%x\n\tsram_write_data: |Tag:0x%x|Valid:%b|Block:0x%x|",CACHE.we, CACHE.write_addr,
                        CACHE.write_data[CACHELINE_TAG_START-1:CACHELINE_TAG_STOP], CACHE.write_data[CACHELINE_VALID_IDX], CACHE.write_data[CACHELINE_BLOCK_START-1:CACHELINE_BLOCK_STOP]);
                $display("---------------------------------------------------");
                $display("\tCache Misses: %0d\n", cache_misses);
                $display("\tCache Hits: %0d\n", cache_hits);
        end
    end

endmodule : Cache_Display
`endif /* SIMULATION_18447 */