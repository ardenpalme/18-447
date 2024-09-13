`default_nettype none

module Fetch_Stage
(input  logic            clk, rst_l,
 input  logic            IF_IF2_en,                 // enableREG
                         stall_pc,
                         br_cond_E,
                         insert_bubble_F,
 input logic [1:0]       br_hist_E,
 input logic [1:0][31:0] instr,
 input logic [31:0]      instr_E,
                         target_addr_E,
                         pc_E,
                        //  pc_D,
                         pc_plus_4_E,
 input  ctrl_signals_t   ctrl_signals_W,

 output logic [31:0]         pc_D,
                             instr_D,
 output logic [29:0]         instr_addr,            // outputs of riscv-core
 output logic                mispredict,
                             predict_D,
 output logic [1:0]          br_hist_D
);
    import MemorySegments::USER_TEXT_START;

    // Manage the value of the PC, don't increment if the processor is halted
    logic [31:0]    prev_pc_F, pc_F, next_pc_F, pc_plus4_F, final_pc_F, next_instr_addr;
    // logic [1:0] prev_br_hist_F;
    // logic prev_predict_F;

    logic [38:0] BTB_line, BTB_write_data;
    logic [1:0]  br_hist, prev_br_hist_F;
    logic        use_BTB_pred,
                 jump_taken_F, predict_F,
                 BTB_we, BTB_tag_match_F, BTB_pred_taken,
                 mispred_brTaken;

    logic [29:0] pred_target_F;

    register #($bits(pc_F), USER_TEXT_START) PC_Register(.clk, .rst_l, .en((~stall_pc) | mispredict), .clear(1'b0), .D(final_pc_F), .Q(pc_F));

    assign instr_addr= pc_F[31:2];
    adder #($bits(pc_F)) Next_PC_Adder(.A(pc_F), .B('d4), .cin(1'b0), .sum(pc_plus4_F), .cout());


    opcode_t opcode_E;
    assign opcode_E = opcode_t'(instr_E[6:0]);

    DetectMispred is_mispred (.br_cond_E, .target_addr_E, .pc_D, .opcode_E, .mispredict, .mispred_brTaken);

    logic [31:0] instr_F;
    assign next_pc_F = predict_F ? {BTB_line[29:0],2'b00} : pc_plus4_F;
    assign final_pc_F = mispredict ?
                          (mispred_brTaken ?  target_addr_E : pc_plus_4_E)
                          : next_pc_F;


    // does instruction in EX jump?
    always_comb begin
        unique case(opcode_E)
            OP_JAL, OP_JALR: begin
                BTB_we= 1'b1;
                jump_taken_F = 1'b1;
            end

            OP_BRANCH: begin
                BTB_we= 1'b1;
                if(br_cond_E)
                    jump_taken_F = 1'b1;
                else
                    jump_taken_F = 1'b0;
            end

            default: begin
                jump_taken_F = 1'b0;
                BTB_we= 1'b0;
            end
        endcase
    end


    assign pred_target_F = target_addr_E[31:2];

    logic [6:0] BTB_read_addr, BTB_write_addr;;
    assign BTB_read_addr  = pc_F[8:2];
    assign BTB_write_addr = pc_E[8:2];
    assign BTB_write_data = {pc_E[15:9], br_hist, pred_target_F};

    sram #(.NUM_WORDS(128), .WORD_WIDTH(39))
         BTB(.clk, .rst_l, .we(BTB_we),
             .read_addr(BTB_read_addr), .read_data(BTB_line),
             .write_addr(BTB_write_addr),
             .write_data(BTB_write_data));


    assign BTB_tag_match_F = pc_F[15:9] == BTB_line[38:32];
    assign BTB_pred_taken  = BTB_line[31];
    assign predict_F = BTB_pred_taken & BTB_tag_match_F;

    HysteresisCounter HYS_CT (.curr(br_hist_E), .opcode_E, .next(br_hist), .jump_taken_F);

    /* IF/IF2 Pipeline Register */
    always_ff @(posedge clk, negedge rst_l) begin
        if(~rst_l) begin
            pc_D       <= 32'd0;
            instr_D    <= 32'h00000013; // NOP
            br_hist_D  <= 'd0;
            predict_D  <= 'd0;

            // prev_predict_F <= 'd0;
            prev_pc_F  <= 'd0;
            prev_br_hist_F <= 2'b00;
        end

        else if (IF_IF2_en) begin
            instr_D    <= (mispredict | insert_bubble_F) ? 32'h00000013 : instr[0]; //bubble is NOP
            pc_D       <= (mispredict | insert_bubble_F) ? 32'h00000000 : prev_pc_F;
            br_hist_D  <= prev_br_hist_F;
            predict_D  <= predict_F;

            // prev_predict_F <= predict_F;
            prev_pc_F  <= mispredict ? final_pc_F : pc_F;
            prev_br_hist_F <= BTB_line[31:30];
        end
    end
endmodule

