/**
 * riscv_decode.sv
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This file contains the implementation of the RISC-V decoder.
 *
 * This takes in information about the current RISC-V instruction and produces
 * the appropriate control signals to get the processor to execute the current
 * instruction.
 **/

/*----------------------------------------------------------------------------*
 *  You may edit this file and add or change any files in the src directory.  *
 *----------------------------------------------------------------------------*/

// RISC-V Includes
`include "riscv_isa.vh"             // RISC-V ISA definitions

// Local Includes
`include "internal_defines.vh"      // Control signals struct, ALU ops

// Force the compiler to throw an error if any variables are undeclared
`default_nettype none

// Define a macro that prints for simulation, does nothing for synthesis
`ifdef SIMULATION_18447
`define display(print, format, arg) \
    do begin \
        if (print) begin \
            $display(format, arg); \
        end \
    end while (0)
`else
`define display(print, format, arg)
`endif /* SIMULATION_18447 */

/**
 * The instruction decoder for the RISC-V processor.
 *
 * This module processes the current instruction, determines what instruction it
 * is, and sets the control signals for the processor appropriately.
 *
 * Inputs:
 *  - rst_l             The asynchronous, active low reset for the processor.
 *  - instr             The current instruction being executed by the processor.
 *
 * Outputs:
 *  - ctrl_signals      The control signals needed to execute the given
 *                      instruction correctly.
 **/
module riscv_decode
    (input  logic           rst_l,
     input  logic [31:0]    instr,
     input  logic           mispredict,
     output ctrl_signals_t  ctrl_signals);

    // Import all of the ISA types and enums (opcodes, functions codes, etc.)
    import RISCV_ISA::*;

    // The various fields of an instruction
    opcode_t            opcode;
    funct7_t            funct7;
    rtype_funct3_t      rtype_funct3;
    itype_int_funct3_t  itype_int_funct3;
    itype_funct12_t     itype_funct12;

    // Decode the opcode and various function codes for the instruction
    assign opcode           = opcode_t'(instr[6:0]);
    assign funct7           = funct7_t'(instr[31:25]);
    assign rtype_funct3     = rtype_funct3_t'(instr[14:12]);
    assign itype_int_funct3 = itype_int_funct3_t'(instr[14:12]);
    assign itype_funct12    = itype_funct12_t'(instr[31:20]);

    always_comb begin
        ctrl_signals = '{
            rtype: 1'b0,
            rd_we: 1'b0,
            syscall: 1'b0,
            illegal_instr: 1'b0,
            alu_op: ALU_DC,
            mem_to_reg: 1'b0,
            data_load_en: 1'b0,
            is_Uinstr: 1'b0,
            is_pc_alu_in: 1'b0,
            is_J_Rd: 1'b0,
            is_J_PC: 1'b0,
            is_mul: 1'b0
        };


        unique case (opcode)
            OP_OP: begin
                unique case (rtype_funct3)
                    // 3-bit function code for add or subtract
                    FUNCT3_ADD_SUB: begin
                        unique case (funct7)
                            // 7-bit function code for typical integer instructions
                            FUNCT7_INT: begin
                                ctrl_signals.alu_op = ALU_ADD;
                                ctrl_signals.rtype = 1'b1;
                                ctrl_signals.rd_we = 1'b1;
                            end

                            FUNCT7_ALT_INT: begin
                                ctrl_signals.alu_op = ALU_SUB;
                                ctrl_signals.rtype = 1'b1;
                                ctrl_signals.rd_we = 1'b1;
                            end

                            FUNCT7_MULDIV: begin
                                ctrl_signals.is_mul = 1'b1;
                                ctrl_signals.is_J_Rd = 1'b1;
                                ctrl_signals.rtype = 1'b1;
                                ctrl_signals.rd_we = 1'b1;
                            end

                            default: begin
                                `display(rst_l, "Encountered unknown/unimplemented 7-bit function code 0x%02x.",
                                        funct7);
                                ctrl_signals.illegal_instr = 1'b1;
                            end
                        endcase
                    end

                    FUNCT3_AND: begin
                            ctrl_signals.alu_op = ALU_AND;
                            ctrl_signals.rtype = 1'b1;
                            ctrl_signals.rd_we = 1'b1;
                    end

                    FUNCT3_OR: begin
                            ctrl_signals.alu_op = ALU_OR;
                            ctrl_signals.rtype = 1'b1;
                            ctrl_signals.rd_we = 1'b1;
                    end

                    FUNCT3_XOR: begin
                            ctrl_signals.alu_op = ALU_XOR;
                            ctrl_signals.rtype = 1'b1;
                            ctrl_signals.rd_we = 1'b1;
                    end

                    FUNCT3_SLT: begin
                            ctrl_signals.alu_op = ALU_SLT;
                            ctrl_signals.rtype = 1'b1;
                            ctrl_signals.rd_we = 1'b1;
                    end

                    FUNCT3_SLTU: begin
                            ctrl_signals.alu_op = ALU_SLTU;
                            ctrl_signals.rtype = 1'b1;
                            ctrl_signals.rd_we = 1'b1;
                    end

                    FUNCT3_SLL: begin
                        ctrl_signals.alu_op = ALU_SLL;
                        ctrl_signals.rtype = 1'b1;
                        ctrl_signals.rd_we = 1'b1;
                    end

                    FUNCT3_SRL_SRA: begin
                        ctrl_signals.rtype = 1'b1;
                        ctrl_signals.rd_we = 1'b1;

                        unique case (funct7)
                            FUNCT7_INT: begin
                                ctrl_signals.alu_op = ALU_SRL;
                            end
                            FUNCT7_ALT_INT: begin
                                ctrl_signals.alu_op = ALU_SRA;
                            end
                        endcase
                    end

                    default: begin
                        `display(rst_l, "Encountered unknown/unimplemented 3-bit rtype integer function code 0x%01x.",
                                rtype_funct3);
                        ctrl_signals.illegal_instr = 1'b1;
                    end
                endcase
            end

            // General I-type arithmetic operation
            OP_IMM: begin
                unique case (itype_int_funct3)
                    FUNCT3_ADDI: begin
                        ctrl_signals.alu_op = ALU_ADD;
                        ctrl_signals.rd_we = 1'b1;
                    end

                    FUNCT3_ANDI: begin
                        ctrl_signals.alu_op = ALU_AND;
                        ctrl_signals.rd_we = 1'b1;
                    end

                    FUNCT3_XORI: begin
                        ctrl_signals.alu_op = ALU_XOR;
                        ctrl_signals.rd_we = 1'b1;
                    end

                    FUNCT3_ORI: begin
                        ctrl_signals.alu_op = ALU_OR;
                        ctrl_signals.rd_we = 1'b1;
                    end

                    FUNCT3_SLLI: begin
                        ctrl_signals.alu_op = ALU_SLL;
                        ctrl_signals.rd_we = 1'b1;
                    end

                    FUNCT3_SRLI_SRAI: begin
                        ctrl_signals.rd_we = 1'b1;
                        unique case (funct7)
                            FUNCT7_INT:
                                ctrl_signals.alu_op = ALU_SRL;
                            FUNCT7_ALT_INT:
                                ctrl_signals.alu_op = ALU_SRA;

                        endcase
                    end

                    FUNCT3_SLTI: begin
                        ctrl_signals.alu_op = ALU_SLT;
                        ctrl_signals.rd_we = 1'b1;
                    end
                    FUNCT3_SLTIU: begin
                        ctrl_signals.alu_op = ALU_SLTU;
                        ctrl_signals.rd_we = 1'b1;
                    end


                    default: begin
                        `display(rst_l, "Encountered unknown/unimplemented 3-bit itype integer function code 0x%01x.",
                                itype_int_funct3);
                        ctrl_signals.illegal_instr = 1'b1;
                    end
                endcase
            end

            // General Load operation
            OP_LOAD: begin
                unique case (itype_int_funct3)
                    //FUNCT3_LW: begin
                    //    ctrl_signals.alu_op = ALU_ADD;
                    //    ctrl_signals.rd_we = 1'b1;
                    //    ctrl_signals.data_load_en = 1'b1;
                    //    ctrl_signals.mem_to_reg = 1'b1;
                    //end

                    FUNCT3_LW, FUNCT3_LB, FUNCT3_LBU, FUNCT3_LH, FUNCT3_LHU: begin
                        ctrl_signals.alu_op = ALU_ADD;
                        ctrl_signals.rd_we = 1'b1;
                        ctrl_signals.data_load_en = 1'b1;
                        ctrl_signals.mem_to_reg = 1'b1;
                    end

                    default: begin
                        `display(rst_l, "Encountered unknown/unimplemented 3-bit itype integer function code 0x%01x.",
                                itype_int_funct3);
                        ctrl_signals.illegal_instr = 1'b1;
                    end
                endcase
            end

            OP_STORE: begin
                unique case (itype_int_funct3)
                    FUNCT3_SW: begin
                        ctrl_signals.alu_op = ALU_ADD;
                    end

                    FUNCT3_SH: begin
                        ctrl_signals.alu_op = ALU_ADD;
                    end

                    FUNCT3_SB: begin
                        ctrl_signals.alu_op = ALU_ADD;
                    end

                endcase
            end

            OP_LUI: begin
                ctrl_signals.rd_we = 1'b1;
                ctrl_signals.is_Uinstr = 1'b1;
            end

            OP_AUIPC: begin
                ctrl_signals.alu_op = ALU_ADD;
                ctrl_signals.rd_we = 1'b1;
                ctrl_signals.is_pc_alu_in = 1'b1;
            end

            OP_JAL: begin
                ctrl_signals.alu_op = ALU_ADD;
                ctrl_signals.rd_we = 1'b1;
                ctrl_signals.is_pc_alu_in = 1'b1;
                ctrl_signals.is_J_Rd = 1'b1;
                ctrl_signals.is_J_PC = 1'b1;
            end

            OP_JALR: begin
                ctrl_signals.alu_op = ALU_ADD;
                ctrl_signals.rd_we = 1'b1;
                ctrl_signals.is_J_Rd = 1'b1;
                ctrl_signals.is_J_PC = 1'b1;
            end

            OP_BRANCH: begin
               ctrl_signals.rtype = 1'b1;

               unique case (rtype_funct3)
                    FUNCT3_BEQ: ctrl_signals.alu_op = ALU_BEQ;
                    FUNCT3_BNE: ctrl_signals.alu_op = ALU_BNE;
                    FUNCT3_BLT: ctrl_signals.alu_op = ALU_BLT;
                    FUNCT3_BGE: ctrl_signals.alu_op = ALU_BGE;
                    FUNCT3_BLTU: ctrl_signals.alu_op = ALU_BLTU;
                    FUNCT3_BGEU: ctrl_signals.alu_op = ALU_BGEU;
                    default: ctrl_signals.alu_op = ALU_DC;      //invalid case
               endcase
            end

            // General system operation
            OP_SYSTEM: begin
                unique case (itype_funct12)
                    FUNCT12_ECALL: begin
                        ctrl_signals.syscall = 1'b1;
                    end

                    default: begin
                        `display(rst_l, "Encountered unknown/unimplemented 12-bit itype function code 0x%03x.",
                                itype_funct12);
                        ctrl_signals.illegal_instr = 1'b1;
                    end
                endcase
            end

            default: begin
                `display(rst_l, "Encountered unknown/unimplemented opcode 0x%02x.", opcode);

                //check if it's a bubble from IF not an illegal inst
                if(opcode == 0 && mispredict)
                    ctrl_signals.illegal_instr = 1'b0;
                else
                ctrl_signals.illegal_instr = 1'b1;
            end
        endcase

        // Only assert the illegal instruction exception after reset
        ctrl_signals.illegal_instr &= rst_l;
    end

endmodule: riscv_decode
