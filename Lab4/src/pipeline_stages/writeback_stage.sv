


// NOTE: syscall_halt_W is an output of mem stage thanks!!!!!
module Writeback_Stage
    (/*input logic          rst_l, clk, br_cond_W, */
     input ctrl_signals_t ctrl_signals_W,
     input logic [31:0]   alu_result_W,
                          data_load_ext_W,
                          mult_out_W,
                          se_immediate_W,
                          pc_plus_4_W,
    //  input opcode_t       opcode_W,
    //  input logic   [2:0]  funct3_W,
     output logic [31:0]  rd_data_W);


    logic [31:0] op_result_W;
    logic [31:0] rd_data1_W, rd_data2_W, target_addr, mult_out;


    /*
module Extend
    (input  logic [31:0] instr_W, data_load_W,
     input  logic [1:0] align_offset_W,
     output logic [31:0] data_load_ext_W);
    */

    // Extend Extender (.opcode_W, .funct3_W, .data_load_W, .align_offset_W(alu_result_W[1:0]), .data_load_ext_W);

    mux #(2, $bits(alu_result_W)) Mem_Out_Mux(.in({data_load_ext_W, alu_result_W}),
                                              .sel(ctrl_signals_W.mem_to_reg), .out(op_result_W));

    mux #(2, $bits(rd_data_W)) Rd_MUX2(.in({rd_data2_W, rd_data1_W}),
                                       .sel(ctrl_signals_W.is_J_Rd), .out(rd_data_W)),

                               Rd_MUX1(.in({se_immediate_W, op_result_W}),
                                       .sel(ctrl_signals_W.is_Uinstr), .out(rd_data1_W)),
                               Rd_MUX3(.in({mult_out_W, pc_plus_4_W}),
                                       .sel(ctrl_signals_W.is_mul), .out(rd_data2_W));

    // assign branch_taken_addr_W = pc_W + se_immediate_W;
    //assign next_instr_addr_W = pc_W + 32'd4;

    //assign target_addr_W = alu_result_W & (32'hFFFE);
endmodule: Writeback_Stage
