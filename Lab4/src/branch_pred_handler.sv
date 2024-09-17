
import RISCV_ISA::*;
`include "internal_defines.vh";      // Control signals struct, ALU ops

module DetectMispred
    (input logic br_cond_E,
        input logic [31:0] target_addr_E,
                        pc_D,
     input opcode_t     opcode_E,
     output logic       mispredict,
                        mispred_brTaken);

    always_comb begin
        unique case(opcode_E)
            OP_JAL, OP_JALR: begin
                mispredict = (target_addr_E != pc_D);
                mispred_brTaken = 1'b1;
            end

            OP_BRANCH: begin
                if(br_cond_E) begin
                    mispredict = (target_addr_E != pc_D);
                    mispred_brTaken = 1'b1;
                end

                else begin
                    if(pc_D == target_addr_E) begin
                        mispred_brTaken = 1'b0;
                        mispredict = 1'b1;
                    end
                    // if(~predict_E || target_addr_E != pc_D)
                    else begin
                        mispred_brTaken = 1'b0;
                        mispredict = 1'b0;
                    end
                end
            end

            default: begin
                mispredict = 1'b0;      // PC+4
                mispred_brTaken = 1'b0;
            end
        endcase
    end

endmodule

module HysteresisCounter
    (input  logic       jump_taken_F,
     input  opcode_t    opcode_E,
     input  logic [1:0] curr,
     output logic [1:0] next);


    typedef enum logic[1:0] {strong_take  = 2'b11,
                             weak_take    = 2'b10,
                             weak_ntake   = 2'b01,
                             strong_ntake = 2'b00} pred_t;
    pred_t curr_pred, next_pred;
    assign curr_pred= pred_t'(curr);

    always_comb begin
        if(opcode_E == OP_BRANCH) begin
                // Hysteresis
    
            unique case(curr_pred)
                strong_take:  next_pred = jump_taken_F ? strong_take : weak_take;
                weak_take:    next_pred = jump_taken_F ? strong_take : strong_ntake;
                weak_ntake:   next_pred = jump_taken_F ? strong_take : strong_ntake;
                strong_ntake: next_pred = jump_taken_F ? weak_ntake  : strong_ntake;
            endcase
            
            // Saturation
            /*
            unique case(curr_pred)
            
                strong_take:  next_pred = jump_taken_F ?  strong_take : weak_take;
                weak_take:    next_pred = jump_taken_F ?  strong_take : weak_ntake;
                weak_ntake:   next_pred = jump_taken_F ?  weak_take: strong_ntake;
                strong_ntake: next_pred = jump_taken_F ?  weak_ntake: strong_ntake;
            endcase
            */
            next = next_pred;
        end

        /* Always Take */
        else if(opcode_E == OP_JAL || opcode_E == OP_JALR)
            next = 2'b11;
        else
            next = 2'b00;  // never should reach
    end
endmodule : HysteresisCounter
