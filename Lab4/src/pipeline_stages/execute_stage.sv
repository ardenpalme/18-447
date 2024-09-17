

/* shifts data by offset and sets store mask */
module Shift_Data_by_Offset_Set_Mask
(input  logic [31:0] rs2_data_E,
                     instr_E,
 input  logic [1:0]  data_align_offset,
 output logic [31:0] rs2_data_shifted_E,
 output logic [3:0]  data_store_mask_shifted_E
);

    opcode_t opcode;
    logic [2:0] funct3;

    assign funct3 = instr_E[14:12];
    assign opcode = opcode_t'(instr_E[6:0]);

    always_comb begin
        unique case(opcode)
            OP_STORE: begin
                unique case(funct3)
                    FUNCT3_SB: begin
                        data_store_mask_shifted_E= 4'b0001 << data_align_offset;
                        unique case(data_align_offset)
                            2'b00: rs2_data_shifted_E = rs2_data_E;
                            2'b01: rs2_data_shifted_E = rs2_data_E << 8;
                            2'b10: rs2_data_shifted_E = rs2_data_E << 16;
                            2'b11: rs2_data_shifted_E = rs2_data_E << 24;
                        endcase
                    end

                    FUNCT3_SH: begin
                        data_store_mask_shifted_E= 4'b0011 << data_align_offset;
                        unique case(data_align_offset)
                            2'b00: rs2_data_shifted_E = rs2_data_E;
                            2'b10: rs2_data_shifted_E = rs2_data_E << 16;
                            default: rs2_data_shifted_E = rs2_data_E;
                        endcase
                    end

                    FUNCT3_SW: begin
                        data_store_mask_shifted_E = 4'b1111;
                        unique case(data_align_offset)
                            2'b00: rs2_data_shifted_E = rs2_data_E;
                            default: rs2_data_shifted_E = rs2_data_E;
                        endcase
                    end
                    default: begin
                        data_store_mask_shifted_E= 4'b0;
                        rs2_data_shifted_E = rs2_data_E;
                    end
                endcase
            end

            default: begin
                data_store_mask_shifted_E = 4'b0;
                rs2_data_shifted_E = rs2_data_E;
            end

        endcase
    end

endmodule

module Execute_Stage
(input  logic           clk, rst_l,
                        EX_MEM_en, syscall_halt_E,
 input  logic [1:0]     fwd_sel_E,
 input  ctrl_signals_t  ctrl_signals_E,

 input  logic [31:0]    pc_E,
                        rs1_data_E, rs2_data_E,
                        se_immediate_E,
                        instr_E,

 output ctrl_signals_t  ctrl_signals_M,
 output logic [31:0]    pc_M, pc_plus_4_E,
                        pc_plus_4_M,
                        alu_result_M,
                        rs2_data_shifted_M,
                        se_immediate_M,
                        instr_M,
                        mult_out_M,
                        val_to_fwd_E,
                        target_addr_E,
 output logic           syscall_halt_M,
                        br_cond_E,
 output logic [3:0] data_store_mask_shifted_M);      // pass syscall to Mem stage s.t. it gets to the WB stage


    import RISCV_ABI::ECALL_ARG_HALT;

    logic [31:0] alu_result_E, op_result, alu_src1_input_E, alu_src2_input_E;

    mux #(2, $bits(alu_src1_input_E)) ALU_Src1_MUX(.in({pc_E, rs1_data_E}), .sel(ctrl_signals_E.is_pc_alu_in), .out(alu_src1_input_E));

    mux #(2, $bits(alu_src2_input_E)) ALU_Src2_Mux(.in({rs2_data_E, se_immediate_E}), .sel(ctrl_signals_E.rtype), .out(alu_src2_input_E));

    riscv_alu ALU(.alu_src1(alu_src1_input_E), .alu_src2(alu_src2_input_E), .alu_op(ctrl_signals_E.alu_op),
        .alu_out(alu_result_E), .bcond(br_cond_E));

    logic [31:0] rs2_data_shifted_E;
    logic [3:0] data_store_mask_shifted_E;

    Shift_Data_by_Offset_Set_Mask Shift_Mask(.rs2_data_E, .instr_E, .data_align_offset(alu_result_E[1:0]),
         .rs2_data_shifted_E, .data_store_mask_shifted_E);

    mult #(1, 32) multiplier (.A(rs1_data_E), .B(rs2_data_E), .O(mult_out_M), .CLK(clk));

    logic [31:0] JALR_target_addr_E, branch_target_addr_E;


    assign pc_plus_4_E = pc_E + 32'd4;

    // Forward_MUX3
    always_comb begin
        unique case (fwd_sel_E)
            2'b00:
                val_to_fwd_E = alu_result_E;
            2'b01:
                val_to_fwd_E = pc_plus_4_E;
            2'b10:
                val_to_fwd_E = se_immediate_E;
            default:
                val_to_fwd_E = alu_result_E;
        endcase


    end
    //assign val_to_fwd_E = fwd_sel_imm_E ? se_immediate_E : alu_result_E;


    assign JALR_target_addr_E = alu_result_E & 32'hffff_fffe; //set LSB to 0
    assign branch_target_addr_E = pc_E + se_immediate_E;

    //assign target_addr_E = branch_target_addr_E;
    opcode_t opcode_E;
    assign opcode_E = opcode_t'(instr_E[6:0]);

    always_comb begin
        unique case(opcode_E)
            OP_JAL:
                target_addr_E = alu_result_E;
            OP_JALR:
                target_addr_E = JALR_target_addr_E;
            default:
                target_addr_E = branch_target_addr_E;
        endcase
    end
    //mux #(3, $bits(alu_result_E)) Branch_Mux1(.in({branch_target_addr_E, JALR_target_addr_E, alu_result_E}), .sel(2'b00), .out(target_addr_E));

    /* EX/MEM Pipeline Register */
    always_ff @(posedge clk, negedge rst_l) begin
        if(~rst_l) begin
            ctrl_signals_M <= 'b0;
            ctrl_signals_M.alu_op <= ALU_DC;
            pc_M <= 'b0;
            alu_result_M <= 'b0;
            rs2_data_shifted_M <= 'b0;
            se_immediate_M <= 'b0;
            data_store_mask_shifted_M <= 'b0;
            instr_M <= 'b0;
            syscall_halt_M <= 'b0;
            pc_plus_4_M <= 4'b0;
        end

        else if(EX_MEM_en) begin
            ctrl_signals_M <= ctrl_signals_E;
            pc_M <= pc_E;
            pc_plus_4_M <= pc_plus_4_E;
            alu_result_M <= alu_result_E;
            rs2_data_shifted_M <= rs2_data_shifted_E;
            se_immediate_M <= se_immediate_E;
            data_store_mask_shifted_M <= data_store_mask_shifted_E;
            instr_M <= instr_E;
            syscall_halt_M <= syscall_halt_E;
        end
    end

endmodule

