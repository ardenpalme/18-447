`default_nettype none

`include "memory_segments.vh"           // Memory segment addresses


module Extend
    (input  logic [31:0] data_load_M,
     input opcode_t    opcode_M,
     input logic   [2:0] funct3_M,

     input  logic [1:0] align_offset_M,
     output logic [31:0] data_load_ext_M);

//     opcode_t    opcode_W;
//     logic [2:0] funct3_W;

//     assign opcode_W = opcode_t'(instr_W[6:0]);
//     assign funct3_W = instr_W[14:12]; //EXCEPT for UJ type
    // data-load extended word loaded from memory
    always_comb begin
        unique case(opcode_M)
            OP_LOAD: begin
                unique case(funct3_M)
                    FUNCT3_LW:  data_load_ext_M = data_load_M;
                    FUNCT3_LB:  begin
                        unique case(align_offset_M)
                            2'b00: data_load_ext_M = {{24{data_load_M[7]}},data_load_M[7:0]};
                            2'b01: data_load_ext_M = {{24{data_load_M[15]}},data_load_M[15:8]}; // >> 1 and sign extend
                            2'b10: data_load_ext_M = {{24{data_load_M[24]}},data_load_M[23:16]};
                            2'b11: data_load_ext_M = {{24{data_load_M[31]}},data_load_M[31:24]}; // >> 3 and sign extend
                        endcase

                    end

                    FUNCT3_LBU: begin
                         unique case(align_offset_M)
                             2'b00: data_load_ext_M = {{24{1'b0}}, data_load_M[7:0]};
                             2'b01: data_load_ext_M = {{24{1'b0}}, data_load_M[15:8]};
                             2'b10: data_load_ext_M = {{24{1'b0}}, data_load_M[23:16]};
                             2'b11: data_load_ext_M = {{24{1'b0}}, data_load_M[31:24]};
                        endcase
                    end
                    FUNCT3_LH: begin
                        unique case (align_offset_M)
                            2'b00, 2'b01: data_load_ext_M = {{16{data_load_M[15]}},data_load_M[15:0]};
                            2'b10, 2'b11: data_load_ext_M = {{16{data_load_M[31]}},data_load_M[31:16]};
                        endcase
                    end
                    FUNCT3_LHU: begin
                        unique case (align_offset_M)
                            2'b00, 2'b01: data_load_ext_M = {{16{1'b0}},data_load_M[15:0]};
                            2'b10, 2'b11: data_load_ext_M = {{16{1'b0}},data_load_M[31:16]};
                        endcase
                    end
                endcase
            end

            default: begin
                data_load_ext_M = data_load_M;
            end
        endcase
    end

endmodule: Extend


module Mem_Stage
    (input logic           rst_l, clk,

    // INPUTS FOR STALLING/FORWARDING
                           MEM_WB_en,
     input logic [1:0]     fwd_sel_M,

     // CACHE OUTPUTS FOR STALLING
     output logic mem_stall,

     // INPUT FROM DATA MEMORY
     input logic [1:0][31:0] data_load,

         // OUTPUTS TO DATA MEMORY
     output logic [3:0]  data_store_mask,
     output logic [29:0] data_addr,
     output logic        data_load_en,
     output logic [31:0] data_store,

     // INPUTS FROM EX STAGE
     input logic             syscall_halt_M,
     input ctrl_signals_t    ctrl_signals_M,
     input logic [31:0]      instr_M,
                             alu_result_M,
                             pc_M,
                             pc_plus_4_M,
                             mult_out_M,
                             se_immediate_M,
                             rs2_data_shifted_M,
     input logic [3:0]       data_store_mask_shifted_M,

     // OUTPUTS TO WB STAGE
     output ctrl_signals_t ctrl_signals_W,
     output logic [31:0]   pc_W,
                           pc_plus_4_W,
                           alu_result_W,
                           data_load_ext_W,
                           se_immediate_W,
                           mult_out_W,
                           val_to_fwd_M,
    //  output opcode_t       opcode_W,
    //  output logic   [2:0]  funct3_W,
     output logic [4:0]    rd_W,
     output logic          syscall_halt_W
     );

    import MemorySegments::STACK_END, MemorySegments::USER_DATA_START;

    logic [31:0] stack_end, user_data_start;
    assign stack_end = MemorySegments::STACK_END;
    assign user_data_start = MemorySegments::USER_DATA_START;

    // NOTE: signals sent to/from memory are in the core since
    //       they are outputs/inputs from the core module

    logic [31:0] data_load_M, data_load_ext_M;
    logic [2:0] funct3_M;
    opcode_t opcode_M;

    assign opcode_M = opcode_t'(instr_M[6:0]);
    assign funct3_M = instr_M[14:12]; //EXCEPT for UJ type

    logic data_write, data_read;
    logic cache_hit, cache_ready;

     // Forwarding logic to ID (Forward_Mux4)
     always_comb begin
        unique case(fwd_sel_M)
            2'b00: val_to_fwd_M = alu_result_M;
            2'b01: val_to_fwd_M = se_immediate_M;
            2'b10: val_to_fwd_M = data_load_ext_M;   // if cache hit fwd cache hit
            2'b11: val_to_fwd_M = pc_plus_4_M;
            default:
                val_to_fwd_M = alu_result_M;
        endcase
     end

    always_comb begin
        data_write = 1'b0;
        data_read  = 1'b0;
        unique casez(opcode_M)
            OP_LOAD:  data_read  = 1'b1;
            OP_STORE: data_write = 1'b1;

            default: begin
                data_write = 1'b0;
                data_read  = 1'b0;
            end
        endcase
    end
    logic        data_load_en_out;
    logic [3:0]  data_store_mask_out;
// 1731

     Cache #(.NUM_WORDS(512), .EN_DISPLAY(1), .NUM_BLOCKS_TO_LOAD_ON_HIT(8)) DATA_CACHE
                 (.rst_l, .clk,
     // INPUTS FROM MEM STAGE
                  .data_write, // asserted if instrution in mem stage is writing to data memory (OP_STORE)
                  .data_read,  // asserted if instrution in mem stage is reading from cache (OP_LOAD)
                  .store_mask_M(data_store_mask_shifted_M),
                  .mem_addr(alu_result_M), // address to read or write to (from LW, SW)
                  .store_data_M(rs2_data_shifted_M),
    // OUTPUTS TO MEM STAGE
                  .cache_hit,    // asserted if data_addr in cache
                  .cache_ready,  // asserted if cache is not currently reading from memory
                  .cached_data(data_load_M),    // Mem[data_addr] if cache_hit
    // INPUTS FROM DATA MEMORY
                  .data_load,  // value read from memory (used to fill cache)
    // OUTPUTS TO DATA MEMORY
                  .data_addr,
                  .data_store,
                  .store_mask(data_store_mask_out),
                  .data_load_en(data_load_en_out));

    always_comb begin
        data_load_en    = data_load_en_out;
        data_store_mask = data_store_mask_out;
    end

/*
    module Extend
    (input  logic [31:0] data_load_M,
     input opcode_t    opcode_M,
     input logic   [2:0] funct3_M,

     input  logic [1:0] align_offset_M,
     output logic [31:0] data_load_ext_M);
*/
    Extend EXTENDER (.data_load_M, .opcode_M, .funct3_M, 
                     .align_offset_M(alu_result_M[1:0]), .data_load_ext_M);

    /* stall IF, ID, EX, MEM until cache hits for
     LW in MEM where req'd data not yet in cache */
    assign mem_stall = data_read & (!cache_hit | !cache_ready);
    // assign mem_stall = 1'b0;
    always_ff @(posedge clk, negedge rst_l) begin
        if (~rst_l) begin
            ctrl_signals_W <= 'b0;
            ctrl_signals_W.alu_op <= ALU_DC;
            pc_W           <= 'b0;
            pc_plus_4_W    <= 'b0;
            alu_result_W   <= 'b0;
            data_load_ext_W    <= 'b0;
            se_immediate_W <= 'b0;
            rd_W        <=  5'd0;
            mult_out_W     <= 'b0;
            syscall_halt_W <= 'b0;
        end
        else if(MEM_WB_en) begin
            ctrl_signals_W <= ctrl_signals_M;
            pc_W           <= pc_M;
            pc_plus_4_W    <= pc_plus_4_M;
            alu_result_W   <= alu_result_M;
            data_load_ext_W    <= data_load_ext_M;     // if cache hit, send forth cache hit
            se_immediate_W <= se_immediate_M;
            rd_W           <= instr_M[11:7];

            // opcode_W       <=  opcode_t'(instr_M[6:0]);
            // funct3_W       <= instr_M[14:12]; //EXCEPT for UJ type

            mult_out_W     <= mult_out_M;
            syscall_halt_W <= syscall_halt_M;
        end
    end
endmodule: Mem_Stage
