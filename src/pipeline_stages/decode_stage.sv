
//import RISCV_ISA::*;

module Decode_Stage
    (input logic            clk, rst_l, syscall_halt_W,
                            ID_EX_en,
                            insert_bubble_D,
                            mispredict,
                            // predict_D,
     input logic [3:0]      fwd_1, fwd_2,
     input logic [4:0]      rd_W,
     input logic [1:0]      br_hist_D,
     input logic [31:0]     pc_D,
                            instr_D,
                            rd_data_W,

                            val_to_fwd_E,
                            val_to_fwd_M,
                            //val_to_fwd_M1, val_to_fwd_M2, val_to_fwd_M3, val_to_fwd_M4,
                            mult_out_M,
                            //mult_out_M1, mult_out_M2, mult_out_M3, mult_out_M4,

     input ctrl_signals_t   ctrl_signals_W,         // signal for writing to register file
     output logic           syscall_halt_E, syscall_halt_D,
                            // predict_E,
     output ctrl_signals_t  ctrl_signals_E,
     output logic [31:0]    pc_E, instr_E,
                            rs1_data_E, rs2_data_E,
                            se_immediate_E,
     output logic [1:0]     br_hist_E);


    // forwarding mux assignments
    logic [31:0] rs1_fwd_D, rs2_fwd_D;

    opcode_t opcode_D;
    ctrl_signals_t ctrl_signals_D;

    riscv_decode Decoder (.rst_l, .instr(instr_D), .ctrl_signals(ctrl_signals_D), .mispredict);

    // assign is_syscall = (ctrl_signals_D.syscall === 1'bx) ? 1'b0 : ctrl_signals_D.syscall;
    //assign is_syscall = ctrl_signals_D.syscall;

    logic [4:0] rs1_D, rs2_D, rd_D;
    // rs1_D assigned below -- these must be available instantaneously
    // (not in stored in pipeline reg)
    assign rs2_D = instr_D[24:20];
    assign rd_D  = instr_D[11:7];



    logic [31:0] rs1_data_D, rs2_data_D;

    mux#(2, $bits(instr_D[19:15])) Syscall_MUX1(.in({X10, instr_D[19:15]}),
                                                .sel(ctrl_signals_D.syscall),
                                                .out(rs1_D));

    logic [31:0] x10_value;


    mux #(2, $bits(x10_value)) Syscall_MUX2 (.in({rs1_fwd_D, 32'd0}), .sel(ctrl_signals_D.syscall), .out(x10_value));

    assign syscall_halt_D = (x10_value == 32'ha);
    // ensure value init not 'x'
    // logic syscall_halt;
    // assign syscall_halt = (syscall_halt_W === 1'bx) ? 1'b0 : syscall_halt_W;

    register_file #(.FORWARD(1)) Reg_File
                           (.clk, .rst_l, .halted(syscall_halt_W),
                            .rd_we(ctrl_signals_W.rd_we),
                            .rd(rd_W), .rd_data(rd_data_W),
                            .rs1(rs1_D), .rs2(instr_D[24:20]),
                            .rs1_data(rs1_data_D), .rs2_data(rs2_data_D));

    always_comb begin
        // Forward_MUX1
        unique case(fwd_1)
            4'd0: rs1_fwd_D = rs1_data_D;
            4'd1: rs1_fwd_D = val_to_fwd_E;
            4'd2: rs1_fwd_D = val_to_fwd_M;

            /*
            4'd3: rs1_fwd_D = val_to_fwd_M1;
            4'd4: rs1_fwd_D = val_to_fwd_M2;
            4'd5: rs1_fwd_D = val_to_fwd_M3;
            4'd6: rs1_fwd_D = val_to_fwd_M4;
            */

            4'd9: rs1_fwd_D = mult_out_M;
            /*
            4'd10: rs1_fwd_D = mult_out_M1;
            4'd11: rs1_fwd_D = mult_out_M2;
            4'd12: rs1_fwd_D = mult_out_M3;
            4'd13: rs1_fwd_D = mult_out_M4;
            */
            default : rs1_fwd_D = 0;
        endcase
        // Forward_MUX2
        unique case(fwd_2)
            4'd0: rs2_fwd_D = rs2_data_D;
            4'd1: rs2_fwd_D = val_to_fwd_E;
            4'd2: rs2_fwd_D = val_to_fwd_M;
            /*
            4'd3: rs2_fwd_D = val_to_fwd_M1;
            4'd4: rs2_fwd_D = val_to_fwd_M2;
            4'd5: rs2_fwd_D = val_to_fwd_M3;
            4'd6: rs2_fwd_D = val_to_fwd_M4;
            */

            4'd9: rs2_fwd_D = mult_out_M;
            /*
            4'd10: rs2_fwd_D = mult_out_M1;
            4'd11: rs2_fwd_D = mult_out_M2;
            4'd12: rs2_fwd_D = mult_out_M3;
            4'd13: rs2_fwd_D = mult_out_M4;
            */
            default : rs2_fwd_D = 0;
        endcase
    end

    assign opcode_D = opcode_t'(instr_D[6:0]);
    //assign funct3_D = instr_D[14:12]; //EXCEPT for UJ type

    logic [31:0] se_immediate_D;
    //sign-extended immediate set based on opcode and funct3
    always_comb begin
        unique case(opcode_D)
            OP_STORE:  se_immediate_D = {{21{instr_D[31]}}, {instr_D[30:25], instr_D[11:7]}};
            OP_LUI:    se_immediate_D = (instr_D & 32'hfffff000);
            OP_AUIPC:  se_immediate_D = (instr_D & 32'hfffff000);
            OP_JAL:    se_immediate_D = {{12{instr_D[31]}}, instr_D[19:12], instr_D[20], instr_D[30:21], 1'b0};
            OP_JALR:   se_immediate_D = {{21{instr_D[31]}}, instr_D[30:20]};
            OP_BRANCH: se_immediate_D = {{20{instr_D[31]}}, instr_D[7], instr_D[30:25], instr_D[11:8], 1'b0};

            default: begin
                se_immediate_D = {{21{instr_D[31]}}, instr_D[30:20]};
            end
        endcase
    end

    always_ff @(posedge clk, negedge rst_l) begin
        if (~rst_l) begin
            ctrl_signals_E  <= 'b0;
            ctrl_signals_E.alu_op <= ALU_DC;
            pc_E            <= 'b0;
            instr_E         <= 'b0;
            rs1_data_E      <= 'b0;
            rs2_data_E      <= 'b0;
            se_immediate_E  <= 'b0;
            syscall_halt_E  <= 'b0;
            br_hist_E       <= 'b0;
            // predict_E       <= 'b0;
        end

        else if(ID_EX_en) begin

            if(insert_bubble_D || mispredict) begin
                instr_E <= 32'h00000013 ;
                pc_E    <= 'b0;
                ctrl_signals_E <='b0;
                ctrl_signals_E.alu_op <= ALU_DC;
            end
            else begin
                instr_E <= instr_D;
                pc_E    <= pc_D;
                ctrl_signals_E <= ctrl_signals_D;
            end


            rs1_data_E <= rs1_fwd_D;
            rs2_data_E <= rs2_fwd_D;
            se_immediate_E <= se_immediate_D;
            syscall_halt_E <= syscall_halt_D;
            br_hist_E <= br_hist_D;
            // predict_E <= predict_D;
        end
    end

endmodule: Decode_Stage
