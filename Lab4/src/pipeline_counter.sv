`default_nettype none
module PipelineCounter
    (input  logic             clk, rst_l,
                              br_cond_E,
                              BTB_tag_match_F,
                              mispredict,
     input  logic [31:0]      pc_F, pc_E,
                              instr_E,
                              se_immediate_E,
     output                   counters_t cnt);

    logic [31:0] last_pc_F, last_pc_E,
                 branch_target_addr_E;

    logic [31:0] BTB_tag_match_F_prev, BTB_tag_match_F_prev2;
    logic [4:0] rs1_E;
    opcode_t opcode_E;

    assign opcode_E = opcode_t'(instr_E[6:0]);
    assign rs1_E = instr_E[19:15];
    assign branch_target_addr_E = se_immediate_E + pc_E;

    always_ff @ (posedge clk, negedge rst_l) begin
        if(~rst_l) begin
            cnt <= '0;
            cnt.cycles <= 1;
            last_pc_F <= 32'd0;
            last_pc_E <= 32'd0;
        end
        else begin
            cnt.cycles <= cnt.cycles + 1;

            if (last_pc_F != 32'd0 && last_pc_F != pc_F)
                cnt.instr_fetch <= cnt.instr_fetch + 1;
            if (pc_E != 32'd0 && pc_E != last_pc_E)
                cnt.instr_exec <= cnt.instr_exec + 1;

            if (opcode_E == OP_BRANCH && pc_E != last_pc_E) begin
                if (branch_target_addr_E < pc_E)
                    cnt.bkwd_br_exec <= cnt.bkwd_br_exec + 1; // backwards branch
                else
                    cnt.fwd_br_exec <= cnt.fwd_br_exec + 1; // forward branch

                if (br_cond_E) begin

                    if (branch_target_addr_E < pc_E)
                        cnt.bkwd_br_taken <= cnt.bkwd_br_taken + 1; // backwards branch
                    else
                        cnt.fwd_br_taken <= cnt.fwd_br_taken + 1; // forward branch

                    if (!mispredict) begin
                        if (branch_target_addr_E < pc_E)
                            cnt.bkwd_br_taken_correct <= cnt.bkwd_br_taken_correct + 1; // backwards branch
                        else
                            cnt.fwd_br_taken_correct <= cnt.fwd_br_taken_correct+ 1; // forward branch
                    end
                end
                else begin
                    if (!mispredict) begin
                        if (branch_target_addr_E < pc_E)
                            cnt.bkwd_br_ntaken_correct <= cnt.bkwd_br_ntaken_correct + 1; // backwards branch
                        else
                            cnt.fwd_br_ntaken_correct <= cnt.fwd_br_ntaken_correct+ 1; // forward branch
                    end

                end

                if (!BTB_tag_match_F_prev2) begin
                    if (branch_target_addr_E < pc_E)
                        cnt.bkwd_br_not_predict <= cnt.bkwd_br_not_predict + 1; // backwards branch
                    else
                        cnt.fwd_br_not_predict <= cnt.fwd_br_not_predict + 1; // forward branch
                end

                if (mispredict) begin


                end
                else begin

                end
            end

            else if (opcode_E == OP_JAL && pc_E != last_pc_E) begin
                if (rs1_E == X1)
                    cnt.jal_ex_funct <= cnt.jal_ex_funct + 1;
                else
                    cnt.jal_ex_nfunct <= cnt.jal_ex_nfunct + 1;

                if (!BTB_tag_match_F_prev2) begin
                    if (rs1_E == X1)
                        cnt.jal_no_btb_funct <= cnt.jal_no_btb_funct + 1;
                    else
                        cnt.jal_no_btb_nfunct <= cnt.jal_no_btb_nfunct + 1;
                end

                if (!mispredict) begin
                    if (rs1_E == X1)
                        cnt.jal_pred_correct_funct <= cnt.jal_pred_correct_funct + 1;
                    else
                        cnt.jal_pred_correct_nfunct <= cnt.jal_pred_correct_nfunct + 1;
                end
            end
            else if (opcode_E == OP_JALR && pc_E != last_pc_E) begin
                if (rs1_E == X1)
                    cnt.jalr_ex_funct <= cnt.jalr_ex_funct + 1;
                else
                    cnt.jalr_ex_nfunct <= cnt.jalr_ex_nfunct + 1;

                if (!BTB_tag_match_F_prev2) begin
                    if (rs1_E == X1)
                        cnt.jalr_no_btb_funct <= cnt.jalr_no_btb_funct + 1;
                    else
                        cnt.jalr_no_btb_nfunct <= cnt.jalr_no_btb_nfunct + 1;
                end

                if (!mispredict) begin
                    if (rs1_E == X1)
                        cnt.jalr_pred_correct_funct <= cnt.jalr_pred_correct_funct + 1;
                    else
                        cnt.jalr_pred_correct_nfunct <= cnt.jalr_pred_correct_nfunct + 1;
                end
            end


            last_pc_F <= pc_F;
            last_pc_E <= pc_E;
            BTB_tag_match_F_prev <= BTB_tag_match_F;
            BTB_tag_match_F_prev2 <= BTB_tag_match_F_prev;

        end
    end

endmodule : PipelineCounter