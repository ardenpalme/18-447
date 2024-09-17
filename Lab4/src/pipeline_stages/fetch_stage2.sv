module Fetch_Stage2
(input logic            clk, rst_l,
                        IF_ID_en,
                        mispredict,     // inserts bubble when IF detects misprediction from EX
                        stall_pc,
                        predict_F2,
 input logic [31:0]     pc_F2,
                        instr_F2,
 input logic [1:0]      br_hist_F2,

 output logic [31:0]    pc_D,
                        instr_D,
 output logic           predict_D,
 output logic [1:0]     br_hist_D
);

    logic [31:0] output_pc_F2;
    logic [1:0] output_br_hist_F2;
    logic output_predict_F2;
    register #($bits(pc_F2)) PC_Register(.clk, .rst_l, .en(~stall_pc), .clear(mispredict), .D(pc_F2), .Q(output_pc_F2));
    register #($bits(br_hist_F2)) BR_Hist_Register(.clk, .rst_l, .en(~stall_pc), .clear(mispredict), .D(br_hist_F2), .Q(output_br_hist_F2));
    register #($bits(predict_F2)) BR_Pred_Register(.clk, .rst_l, .en(~stall_pc), .clear(mispredict), .D(predict_F2), .Q(output_predict_F2));

    /* IF/ID Pipeline Register */
    always_ff @(posedge clk, negedge rst_l) begin
        if(~rst_l) begin
            pc_D <= 32'd0;
            instr_D <= 32'h00000013; // NOP
            br_hist_D  <= 'b0;
            predict_D  <= 'b0;
        end

        else if (IF_ID_en) begin
            instr_D   <= mispredict ? 32'h00000013 : instr_F2; //bubble is NOP
            pc_D      <= mispredict ? 32'b0 : output_pc_F2;
            br_hist_D <= output_br_hist_F2;
            predict_D <= output_predict_F2;
        end
    end

endmodule : Fetch_Stage2
