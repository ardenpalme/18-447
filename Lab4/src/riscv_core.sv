/**
 * riscv_core.sv
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This is the core part of the processor, and is responsible for executing the
 * instructions and updating the CPU state appropriately.
 *
 * This is where you can start to add code and make modifications to fully
 * implement the processor. You can add any additional files or change and
 * delete files as you need to implement the processor, provided that they are
 * under the src directory. You may not change any files outside the src
 * directory. The only requirement is that there is a riscv_core module with the
 * interface defined below, with the same port names as below.
 *
 * The Makefile will automatically find any files you add, provided they are
 * under the src directory and have either a *.v, *.vh, or *.sv extension. The
 * files may be nested in subdirectories under the src directory as well.
 * Additionally, the build system sets up the include paths so that you can
 * place header files (*.vh) in any subdirectory in the src directory, and
 * include them from anywhere else inside the src directory.
 *
 * The compiler and synthesis tools support both Verilog and System Verilog
 * constructs and syntax, so you can write either Verilog or System Verilog
 * code, or mix both as you please.
 **/

/*----------------------------------------------------------------------------*
 *  You may edit this file and add or change any files in the src directory.  *
 *----------------------------------------------------------------------------*/

// RISC-V Includes
`include "riscv_abi.vh"             // ABI registers and definitions
`include "riscv_isa.vh"             // RISC-V ISA definitions
`include "memory_segments.vh"       // Memory segment starting addresses

// Local Includes
`include "internal_defines.vh"      // Control signals struct, ALU ops

/* A quick switch to enable/disable tracing. Comment out to disable. Please
 * comment this out before submitting your code. You'll also want to comment
 * this out for longer tests, as it will make them run much faster. */
`define TRACE

// Force the compiler to throw an error if any variables are undeclared
`default_nettype none

/**
 * The core of the RISC-V processor, everything except main memory.
 *
 * This is the RISC-V processor, which, each cycle, fetches the next
 * instruction, executes it, and then updates the register file, memory,
 * and register file appropriately.
 *
 * The memory that the processor interacts with is dual-ported with a
 * single-cycle synchronous write and combinational read. One port is used to
 * fetch instructions, while the other is for loading and storing data.
 *
 * Inputs:
 *  - clk               The global clock for the processor.
 *  - rst_l             The asynchronous, active low reset for the processor.
 *  - instr_mem_excpt   Indicates that an invalid instruction address was given
 *                      to memory.
 *  - data_mem_excpt    Indicates that an invalid address was given to the data
 *                      memory during a load and/or store operation.
 *  - instr             The instruction loaded loaded from the instr_addr
 *                      address in memory.
 *  - data_load         The data loaded from the data_addr address in memory.
 *
 * Outputs:
 *  - data_load_en      Indicates that data from the data_addr address in
 *                      memory should be loaded.
 *  - halted            Indicates that the processor has stopped because of a
 *                      syscall or exception. Used to indicate to the testbench
 *                      to end simulation. Must be held until next clock cycle.
 *  - data_store_mask   Byte-enable bit mask signal indicating which bytes of data_store
 *                      should be written to the data_addr address in memory.
 *  - instr_addr        The address of the instruction to load from memory.
 *  - instr_stall       stall instruction load from memory if multicycle.
 *  - data_addr         The address of the data to load or store from memory.
 *  - data_stall        stall data load from memory if multicycle.
 *  - data_store        The data to store to the data_addr address in memory.
 **/
module riscv_core
    (input  logic              clk, rst_l, instr_mem_excpt, data_mem_excpt,
     input  logic [1:0] [31:0] instr,
     input  logic [1:0] [31:0] data_load,
     output logic              data_load_en, halted,
     output logic [3:0]        data_store_mask,
     output logic [29:0]       instr_addr, data_addr,
     output logic              instr_stall, data_stall,
     output logic [31:0]       data_store);

    /* Import the ISA field types, and the argument to ecall to halt the
     * simulator, and the start of the user text segment. */
    import RISCV_ISA::*;
    import RISCV_ABI::ECALL_ARG_HALT;
    import MemorySegments::USER_TEXT_START;

    /* Pipeline register enables */
    logic IF_ID_en, ID_EX_en, EX_MEM_en, MEM_WB_en;
    logic MEM0_MEM1_en, MEM1_MEM2_en, MEM2_MEM3_en, MEM3_MEM4_en,MEM4_WB_en ;
    logic IF_IF2_en;

    logic [3:0] pipeline_reg_state;

    assign IF_ID_en  = pipeline_reg_state[3];
    assign IF_IF2_en = pipeline_reg_state[3];

    assign ID_EX_en  = pipeline_reg_state[2];
    assign EX_MEM_en = pipeline_reg_state[1];
    // assign MEM_WB_en = pipeline_reg_state[0];
    assign MEM0_MEM1_en = pipeline_reg_state[0];
    assign MEM1_MEM2_en = pipeline_reg_state[0];
    assign MEM2_MEM3_en = pipeline_reg_state[0];
    assign MEM3_MEM4_en = pipeline_reg_state[0];

    assign MEM_WB_en = pipeline_reg_state[0];


    //assign stall_pc = (syscall_halt_W === 1'bx) ? 1'b0 : syscall_halt_W;

    logic [31:0] instr_D, instr_E, instr_M;
    ctrl_signals_t ctrl_signals_E, ctrl_signals_M, ctrl_signals_W;
    // ctrl_signals_t ctrl_signals_M1, ctrl_signals_M2, ctrl_signals_M3, ctrl_signals_M4;

    logic need_stall, need_insert_bubble_D, mem_stall;
    logic stall_pc, insert_bubble_D, insert_bubble_F;

    // Signals for branch prediction
    logic [31:0] target_addr_E, pc_E, pc_plus_4_E;
    logic        br_cond_E,
                 mispredict,
                 predict_F, predict_D;
    logic [1:0]  br_hist_D, br_hist_E;

    // Signals in MEM/WB Register
    logic [31:0]   pc_W, pc_plus_4_W,
                   alu_result_W,
                //    data_load_ext_W,
                   se_immediate_W,
                   mult_out_W;
    logic syscall_halt_W;

    // Outputs of WB stage that aren't in MEM/WB register
    logic [31:0] target_addr_W,
                 branch_taken_addr_W,
                 rd_data_W;

    // Signals in IF/IF2 register
    // logic predict_F2;
    // logic [1:0] br_hist_F2;
    // logic [29:0] instr_addr_F2;
    // logic [31:0] instr_F2, pc_F2;

    // Signals in IF/ID register
    logic [31:0] pc_D;

    logic [31:0] val_to_fwd_M0;

    // logic [31:0] pc_M1, pc_plus_4_M1, alu_result_M1,
    //              se_immediate_M1, instr_M1,
    //              mult_out_M1, val_to_fwd_M1;
    // logic br_cond_M1, syscall_halt_M1;

    // logic [31:0] pc_M2, pc_plus_4_M2, alu_result_M2,
    //              se_immediate_M2, instr_M2,
    //              mult_out_M2, val_to_fwd_M2;
    // logic br_cond_M2, syscall_halt_M2;

    // logic [31:0] pc_M3, pc_plus_4_M3, alu_result_M3,
    //              se_immediate_M3, instr_M3,
    //              mult_out_M3, val_to_fwd_M3;
    // logic br_cond_M3, syscall_halt_M3;

    // logic [31:0] pc_M4, pc_plus_4_M4, alu_result_M4,
    //              se_immediate_M4, instr_M4,
    //              mult_out_M4, val_to_fwd_M4;
    // logic br_cond_M4, syscall_halt_M4;

    Fetch_Stage FETCH_STG (.clk, .rst_l,
                           .instr,
                           .instr_E,
                           .br_cond_E,
                           .br_hist_E,
                           .pc_E, .pc_plus_4_E,
                           .target_addr_E,
                           .ctrl_signals_W,
                           .IF_IF2_en,
                           .stall_pc,
                           .mispredict,
                           .insert_bubble_F,
                        //    .pc_F2,
                           .pc_D,
                           .instr_D,
                           .instr_addr,
                           .br_hist_D,
                           .predict_D);

    //  Signals in ID/EX Register
    //ctrl_signals_t  ctrl_signals_D; //necessary for hazard detection
    logic [31:0]    rs1_data_E, rs2_data_E, se_immediate_E;

    // signal genereated in decode stage
    logic syscall_halt_D;

    // signals forwarded to decode stage
    logic [31:0] val_to_fwd_E, mult_out_M0;
    logic [3:0] fwd_1, fwd_2;
    logic [1:0] fwd_sel_E, fwd_sel_M;
    logic [1:0] fwd_sel_M1, fwd_sel_M2, fwd_sel_M3, fwd_sel_M4;
    logic syscall_halt_E;

    // pass data onto the MEM 4 if it's a cache hit
    logic cache_hit_M0;

    logic [3:0]  data_store_mask_shifted_M;
    // logic [31:0] data_load_M1, data_load_M2, data_load_M3;
    logic        cache_hit_M1, cache_hit_M2, cache_hit_M3;
    logic [1:0] [31:0] cached_data;

    logic [31:0] mult_out_M;
    logic [31:0] val_to_fwd_M;
    logic [4:0]  rd_W;

    Forwarding_Unit FWD_UNIT (.instr_D, .instr_E, .instr_M,
                              //.instr_M1, .instr_M2, .instr_M3, .instr_M4,
                              //.cache_hit_M0, .cache_hit_M1, .cache_hit_M2, .cache_hit_M3,
                              .ctrl_signals_E, .ctrl_signals_M,
                              //.ctrl_signals_M1, .ctrl_signals_M2, .ctrl_signals_M3, .ctrl_signals_M4,
                              .fwd_sel_E, .fwd_sel_M,
                              //.fwd_sel_M1, .fwd_sel_M2, .fwd_sel_M3, .fwd_sel_M4,
                              .fwd_1, .fwd_2);

    Decode_Stage DEC_STG (.clk, .rst_l,
                          .syscall_halt_W(halted),
                          .pc_D,
                        //   .predict_D,
                          .instr_D, .rd_W,
                          .br_hist_D,
                          .rd_data_W,
                          .ctrl_signals_W,  // signal for writing to register file
                          .ID_EX_en,
                          .insert_bubble_D,
                          .mispredict,
                        //   .predict_E,
                          .syscall_halt_D, .syscall_halt_E,
                          .ctrl_signals_E,
                          .pc_E, .instr_E,
                          .br_hist_E,
                          .rs1_data_E, .rs2_data_E,
                          .se_immediate_E,
                          .fwd_1, .fwd_2,
                          .val_to_fwd_E, .val_to_fwd_M,
                          .mult_out_M
                          );

    // Signals in EX/MEM Register
    logic syscall_halt_M;
    logic [31:0]    pc_M, pc_plus_4_M,
                    alu_result_M,
                    rs2_data_shifted_M,
                    se_immediate_M,
                    data_load_ext_W;



    Execute_Stage EXEC_STG (.clk, .rst_l,
                            .ctrl_signals_E,
                            .pc_E, .pc_plus_4_E,
                            .rs1_data_E, .rs2_data_E,
                            .se_immediate_E,
                            .val_to_fwd_E,
                            .fwd_sel_E,
                            .instr_E,
                            .syscall_halt_E,
                            .EX_MEM_en,
                            .target_addr_E,
                            .br_cond_E,
                            .ctrl_signals_M,
                            .pc_M, .pc_plus_4_M,
                            .alu_result_M,
                            .rs2_data_shifted_M,
                            .se_immediate_M,
                            .data_store_mask_shifted_M,
                            .instr_M,
                            .mult_out_M,
                            .syscall_halt_M); //pass syscall to Mem stage s.t. it gets to the WB stage


    opcode_t opcode_W;
    logic [2:0] funct3_W;

    Mem_Stage MEM_STG (.rst_l, .clk,
                       .data_load,
                        // .cache_hit, .cache_ready,
                       .ctrl_signals_M,
                       .alu_result_M,
                       .instr_M,
                       .mult_out_M,
                       .pc_M,
                       .pc_plus_4_M,
                       .se_immediate_M,
                       .syscall_halt_M,
                       .data_store_mask_shifted_M,
                       .rs2_data_shifted_M,

                       //outputs to MM
                       .data_store_mask,
                       .data_addr,
                       .data_load_en,
                       .data_store,

                       .mem_stall,

                       .MEM_WB_en,
                       .ctrl_signals_W,
                       .pc_W, .pc_plus_4_W,
                       .alu_result_W,
                       .data_load_ext_W,
                       .se_immediate_W,
                       .rd_W,
                    //    .funct3_W,
                    //    .opcode_W,
                       .mult_out_W,
                       .val_to_fwd_M,
                       .syscall_halt_W,
                       .fwd_sel_M);

    Writeback_Stage WB_STG (.ctrl_signals_W,
                            .pc_plus_4_W,
                            .alu_result_W,
                            // .funct3_W,
                            // .opcode_W,
                            .data_load_ext_W,
                            .se_immediate_W,
                            .mult_out_W,
                            .rd_data_W);

    assign data_stall       = 1'b0;

    logic syscall_exit;
    assign syscall_exit = syscall_halt_E  || syscall_halt_M || syscall_halt_W;


        /* Hazard Detection */
     Locate_Hazard LH(.instr_D, .instr_E,
                      .instr_M,
                      .ctrl_signals_E,
                      .ctrl_signals_M,
                      .need_stall, .need_insert_bubble_D);

    // also handles termination and Hazzards
    Stall_Handler STALL_HNDLR (.clk, .rst_l,
                           .syscall_exit, .pipeline_reg_state,
                           .mispredict, .instr_stall,
                           .stall_pc, .insert_bubble_D, .insert_bubble_F,
                           .mem_stall,
                           .need_stall, .need_insert_bubble_D);


    /* Writeback data to the register file, and handle any syscalls/exceptions.
     * Note that you don't need to support exceptions, they are here simply to
     * aid with debugging. */

    //logic [31:0]    a0_value;
    logic           syscall_halt, exception_halt;
    //assign a0_value         = x10_value;
    assign syscall_halt     = ctrl_signals_W.syscall & syscall_halt_W;
    assign exception_halt   = instr_mem_excpt | data_mem_excpt | ctrl_signals_W.illegal_instr;
    //assign exception_halt   = instr_mem_excpt | ctrl_signals_W.illegal_instr;
    assign halted = rst_l & (syscall_halt | exception_halt);
    //assign instr_stall = 1'b0;

`ifdef SIMULATION_18447

    counters_t cnt;
    PipelineCounter COUNTERS (.rst_l, .clk,
                              .mispredict,
                              .BTB_tag_match_F(FETCH_STG.BTB_tag_match_F),
                              .pc_F({instr_addr, 2'b00}),
                              .br_cond_E,
                              .pc_E,
                              .instr_E,
                              .se_immediate_E,
                              .cnt);

    always_ff @(posedge clk) begin
        if (rst_l && instr_mem_excpt) begin
            $display("Instruction memory exception at address 0x%08x.", instr_addr << 2);
        end
        if (rst_l && data_mem_excpt) begin
            $display("Data memory exception at address 0x%10x.", data_addr << 2);
        end
        if (rst_l && syscall_halt) begin
            $display("ECALL invoked with halt argument. Terminating simulation at 0x%08x.", pc_D);
        end
    end
`endif /* SIMULATION_18447 */

    /* When the design is compiled for simulation, the Makefile defines
     * SIMULATION_18447. You can use this to have code that is there for
     * simulation, but is discarded when the design is synthesized. Useful
     * for constructs that can't be synthesized. */
`ifdef SIMULATION_18447
`ifdef TRACE

    opcode_t opcode;
    funct7_t funct7;
    rtype_funct3_t rtype_funct3;
    itype_int_funct3_t itype_int_funct3;
    assign opcode           = opcode_t'(instr_D[6:0]);
    assign funct7           = funct7_t'(instr_D[31:25]);
    assign rtype_funct3     = rtype_funct3_t'(instr_D[14:12]);
    assign itype_int_funct3 = itype_int_funct3_t'(instr_D[14:12]);

    /* Cycle-by-cycle trace messages. You'll want to comment this out for
     * longer tests, or they will take much, much longer to run. Be sure to
     * comment this out before submitting your code, so tests can be run
     * quickly. */

     int DISPLAY=1;

    always_ff @(posedge clk) begin
        if (rst_l && DISPLAY) begin
/*
            $display({"\n", {80{"-"}}});
            $display("- Simulation Cycle %0d", $time);
            $display({{80{"-"}}, "\n"});
        
            $display("\tInstruction Memory Exception: %0b", instr_mem_excpt);
            $display("\tData Memory Exception: %0b", data_mem_excpt);
            $display("\tIllegal Instruction Exception[WB]: %0b", ctrl_signals_W.illegal_instr);
            $display("\tHalted: %0b\n", halted);


            $display("\tOpcode[ID]: 0x%02x (%s)", opcode, opcode.name);
            $display("\tFunct3[ID]: 0x%01x (%s | %s)", rtype_funct3, rtype_funct3.name, itype_int_funct3.name);
            $display("\tFunct7[ID]: 0x%02x (%s)\n", funct7, funct7.name);
            $display("\tstall_pc: %b", stall_pc);
            $display("\tinstr_stall: %b", instr_stall);
            $display("\tmem_stall: %b\n", mem_stall);

            $display("\t===== IF STAGE =====");
            $display("\tprev_pc_F: 0x%08x, next_pc_F: 0x%08x, instr_addr: 0x%08x, BTB Tag Match: %b",FETCH_STG.prev_pc_F, FETCH_STG.next_pc_F,{instr_addr, 2'b0}, FETCH_STG.BTB_tag_match_F);
            $display("\tInstruction[0]: 0x%08x, Instruction[1]: 0x%08x", instr[0], instr[1]);
            // $display("\tbtb_read: addr: 0x%08x, tagPC: 0x%08x, br_hist: %02b, nextPC: 0x%08x", FETCH_STG.BTB_read_addr, {FETCH_STG.BTB_line[61:32],2'b00}, FETCH_STG.BTB_line[31:30], {FETCH_STG.BTB_line[29:0],2'b00});
            // $display("\tbtb_write: addr: 0x%08x, {tagPC: 0x%08x, br_hist: %02b, nextPC: 0x%08x} [BTB_we %b]\n",FETCH_STG.BTB_write_addr,  {FETCH_STG.BTB_write_data[61:32],2'b00}, FETCH_STG.BTB_write_data[31:30], {FETCH_STG.BTB_write_data[29:0],2'b00}, FETCH_STG.BTB_we);

            $display("\t[predict_F: %b, mispredict: %b] final_pc_F: 0x%08x\n", FETCH_STG.predict_F, mispredict, FETCH_STG.final_pc_F);
            $display("\tmispred_br_taken: %b\n", FETCH_STG.mispred_brTaken);
            // $display("\tIF2 STAGE");
            // $display("\tpc_F2: 0x%08x", pc_F2);
            // $display("\tInstruction[IF2]: 0x%08x", instr_F2);
            // $display("\tpredict_F2 %b]\n", predict_F2);


            $display("\t===== DEC STAGE =====");
            $display("\tpc_D: 0x%08x, br_hist_D: %b", pc_D, br_hist_D);
            $display("\tInstruction[ID]: 0x%08x", instr_D);

            $display("\tse_imm: 0x%08x", DEC_STG.se_immediate_D);
            $display("\trd_D: %0d, rs1_D: %0d, rs2_D: %0d", instr_D[11:7], instr_D[19:15], instr_D[24:20]);

            $display("\tsyscall_halt_D: %b", syscall_halt_D);


            $display("\tfwd_sel_E: %b, fwd_sel_M: %b\n\tval_to_fwd_M: 0x%08x, val_to_fwd_E: 0x%08x\n\tfwd_1: %b, fwd_2: %b,\n\trs1_fwd_D: 0x%08x, rs2_fwd_D: 0x%08x\n", fwd_sel_E, fwd_sel_M, fwd_1, fwd_2, DEC_STG.rs1_fwd_D, DEC_STG.rs2_fwd_D, MEM_STG.val_to_fwd_M, EXEC_STG.val_to_fwd_E);


            $display("\t===== EX STAGE =====");
            $display("\tpc_E: 0x%08x, ALU_OP: %s [br_cond_E %b], curr_history: %b, next_history: %b", pc_E, ctrl_signals_E.alu_op.name, br_cond_E, br_hist_E, FETCH_STG.br_hist);
            $display("\tInstruction[EX]: 0x%08x\n", instr_E);

            $display("\tpc_plus_4_E: 0x%08x", EXEC_STG.pc_plus_4_E);
            $display("\t\trs1_data_E: 0x%08x, rs2_data_E: 0x%08x, mult_out_M: 0x%08x", rs1_data_E, rs2_data_E, mult_out_M);
            $display("\tALU: src_1: 0x%08x, src_2: 0x%08x => 0x%08x", EXEC_STG.alu_src1_input_E, EXEC_STG.alu_src2_input_E, EXEC_STG.alu_result_E);
            // $display("\tse_imm: 0x%08x, [predict_E %b] target_addr_E: 0x%08x, ",se_immediate_E, predict_E, target_addr_E);
            $display("\trs2_data_shifted_E: 0x%08x, instr_E: 0x%08x, rs2_data_E; 0x%08x", EXEC_STG.rs2_data_shifted_E, instr_E, rs2_data_E);
            $display("\trd_E: %0d, ctrl_signals_E.rd_we: %b\n", instr_E[11:7], ctrl_signals_E.rd_we);

            // $display("\tMEM STAGES");
            // $display("\trs2_data_shifted_M: 0x%08x [data_load_en %b store_mask %b]", rs2_data_shifted_M, ctrl_signals_M3.data_load_en, data_store_mask);

            $display("\t===== MEM STAGE =====");
            $display("\tpc_M: 0x%08x instr_M: 0x%08x", pc_M, instr_M);
            $display("\tstack_end: 0x%08x user_data_start: 0x%08x \n", MEM_STG.stack_end, MEM_STG.user_data_start);

            $display("\tdata [R/W]: %b, %b", MEM_STG.data_read, MEM_STG.data_write);
            $display("\tcache_hit: %b, cache_ready: %b", MEM_STG.cache_hit, MEM_STG.cache_ready);
            // $display("\tcache_line: {0x%08x, %b, [0x%08x, 0x%08x]} hit: %b", {MEM_STG0.cache_line[94:65], 2'b00}, MEM_STG0.cache_line[64], MEM_STG0.cache_line[63:32], MEM_STG0.cache_line[31:0], cache_hit_M0);
            $display("\tdata_memory_output: 0x%08x, cached_data: 0x%08x", data_load, MEM_STG.data_load_M);
            $display("\tcache_write to sram[%d]", alu_result_M[31:24]);
            $display("\tmult_out_M: 0x%08x", mult_out_M);
            $display("\trd_M0: %0d, rd_we: %b", instr_M[11:7], ctrl_signals_M.rd_we);
            $display("\tfwd_sel_M: %b, val_to_fwd_M: 0x%08x", fwd_sel_M, val_to_fwd_M);
            $display("\tMEM[0x%08x]\n", {MEM_STG.alu_result_M[31:2], 2'b00});

            $display("\tWB STAGE");
            $display("\tpc_W: 0x%08x", pc_W);
            // $display("\trd_W: %0d, rd_data_W: 0x%08x, rd_data1_W: %08x, rd_data2_W: %08x\n", instr_W[11:7], rd_data_W, WB_STG.rd_data1_W, WB_STG.rd_data2_W);
            $display("\tcurr_state: %s\n\n", STALL_HNDLR.curr_state.name);

            $display("\tmult_out_W: 0x%08x", mult_out_W);
            //$display("\tpc_plus_4_W: 0x%08x", pc_plus_4_W);
            // $display("data_load_ext_W = %08x", data_load_ext_W);

            // $display("\tinstr_W 0x%08x\n", WB_STG.instr_W);

            $display("\tSign Extended Immediate [EX]: %0d", se_immediate_E);
            $display("\tpipeline_reg_state: %04b", pipeline_reg_state);

            $display("\tHAZARD [WB]: %b", need_stall);
            $display("\tinsert_bubble_D: %b", insert_bubble_D);
            $display("\tcurr_state: %s\n\n", STALL_HNDLR.curr_state.name);
*/
            if (halted) begin
                $display("Num cycles Elapsed %0d\nNum Instr Fetched: %0d\nNum Instr Exec'd: %0d",
                cnt.cycles, cnt.instr_fetch, cnt.instr_exec);

                $display("                            | rd == X1 | rd != X1  ");
                $display("                            | FWD      | BACK      ");
                $display("---------------------------------------------------");
                $display("Num br executed:            | %0d        %0d      ",
                        cnt.fwd_br_exec, cnt.bkwd_br_exec);
                $display("Num br without BTB pred:    | %0d        %0d      ",
                        cnt.fwd_br_not_predict, cnt.bkwd_br_not_predict);
                $display("Num br taken:               | %0d        %0d      ",
                        cnt.fwd_br_taken, cnt.bkwd_br_taken);
                $display("Num br taken correct pred:  | %0d        %0d      ",
                        cnt.fwd_br_taken_correct, cnt.bkwd_br_taken_correct);
                $display("Num br !taken correct pred: | %0d        %0d      ",
                        cnt.fwd_br_ntaken_correct, cnt.bkwd_br_ntaken_correct);

                $display("Num JAL executed:           | %0d        %0d      ",
                        cnt.jal_ex_funct, cnt.jal_ex_nfunct);
                $display("Num JAL without BTB pred:   | %0d        %0d      ",
                        cnt.jal_no_btb_funct, cnt.jal_no_btb_funct);
                $display("Num JAL correct pred:       | %0d        %0d      ",
                        cnt.jal_pred_correct_funct, cnt.jal_pred_correct_funct);

                $display("Num JALR executed:          | %0d        %0d      ",
                        cnt.jalr_ex_funct, cnt.jalr_ex_nfunct);
                $display("Num JALR without BTB pred:  | %0d        %0d      ",
                        cnt.jalr_no_btb_funct, cnt.jalr_no_btb_funct);
                $display("Num JALR correct pred:      | %0d        %0d      ",
                        cnt.jalr_pred_correct_funct, cnt.jalr_pred_correct_funct);
            end

        end
    end

`endif /* TRACE */
`endif /* SIMULATION_18447 */



endmodule: riscv_core
