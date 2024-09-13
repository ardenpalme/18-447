

// given the instructions in every stage except fetch, asserts output signals
// that correspond to which stage causes write in RAW hazard
module Locate_Hazard
    (input logic [31:0]   instr_D, instr_E,
                         instr_M,
     input ctrl_signals_t ctrl_signals_E,
                          ctrl_signals_M,
     output logic         need_stall, need_insert_bubble_D);


    logic [4:0] rd_D, rd_E, rs1_D, rs1_E, rs2_D, rs2_E;

    opcode_t opcode_E, opcode_D;
    funct7_t funct7_E;

    assign rd_D = instr_D[11:7];
    assign rs1_D = instr_D[19:15];
    assign rs2_D = instr_D[24:20];

    assign rd_E = instr_E[11:7];
    assign rs1_E = instr_E[19:15];
    assign rs2_E = instr_E[24:20];

    assign funct7_E = funct7_t'(instr_E[31:25]);

    assign opcode_E = opcode_t'(instr_E[6:0]);
    assign opcode_D = opcode_t'(instr_D[6:0]);


    // MEM Stage
    logic [4:0] rd_M0, rs1_M0, rs2_M0;

    opcode_t opcode_M0;
    funct7_t funct7_M0;

    assign rd_M0 = instr_M[11:7];
    assign rs1_M0 = instr_M[19:15];
    assign rs2_M0 = instr_M[24:20];

    assign funct7_M0 = funct7_t'(instr_M[31:25]);
    assign opcode_M0 = opcode_t'(instr_M[6:0]);

    /*
    * Possible Hazards:
    * 1. MUL in EX stage with rd == rs1 || rd == rs2
    * 2. LOAD in EX-M3 stage with rd == rd1 || rd == rs2
    * 3. instr_D is syscall and one of prev two hazards present
    */

    always_comb begin
        need_stall = 1'b0;
        need_insert_bubble_D = 1'b0;

        // LUI, AUIPC, JAL never have dependencies since they don't use rs1 or rs2
        if (opcode_D != OP_LUI && opcode_D != OP_AUIPC && opcode_D != OP_JAL) begin

            // stall with RAW dependency of dist 1 (with LOAD or MUL in EX)
            if (opcode_E == OP_LOAD || (opcode_E == OP_OP && funct7_E == FUNCT7_MULDIV)) begin
                // check rs1 dependency (remaining instructions all use rs1)
                if (rd_E != X0 && rs1_D == rd_E && opcode_D != OP_SYSTEM) begin
                    need_stall = 1'b1;
                end
                else if (rd_E == X10 && opcode_D == OP_SYSTEM) begin
                    need_stall = 1'b1;
                end

                // check rs2 dependency (I-type don't use rs2)
                if (opcode_D != OP_IMM && opcode_D != OP_LOAD) begin
                    if (rd_E != X0 && rd_E == rs2_D) begin
                        need_stall = 1'b1;
                    end
                end
            end

            // stall with RAW dependency of dist 2 (with LOAD in M0)
            if (opcode_M0 == OP_LOAD ) begin
                // check rs1 dependency (remaining instructions all use rs1)
                if (rd_M0 != X0 && rs1_D == rd_M0 && opcode_D != OP_SYSTEM) begin
                    //if(~cache_hit_M0) need_stall = 1'b1;
                    //else need_stall = 1'b0;
                    need_stall = 1'b1;
                end
                else if (rd_M0 == X10 && opcode_D == OP_SYSTEM) begin
                    need_stall = 1'b1;
                end

                // check rs2 dependency (I-type don't use rs2)
                if (opcode_D != OP_IMM && opcode_D != OP_LOAD) begin
                    if (rd_M0 != X0 && rd_M0 == rs2_D) begin
                        //if(~cache_hit_M0) need_stall = 1'b1;
                        //else need_stall = 1'b0;
                        need_stall = 1'b1;
                    end
                end
            end

            // stall with RAW dependency of dist 2 (with MUL in M0)
            if (opcode_M0 == OP_OP && funct7_M0 == FUNCT7_MULDIV) begin
                // check rs1 dependency (remaining instructions all use rs1)
                if (rd_M0 != X0 && rs1_D == rd_M0 && opcode_D != OP_SYSTEM) begin
                    need_stall = 1'b1;
                end
                else if (rd_M0 == X10 && opcode_D == OP_SYSTEM) begin
                    need_stall = 1'b1;
                end

                // check rs2 dependency (I-type don't use rs2)
                if (opcode_D != OP_IMM && opcode_D != OP_LOAD) begin
                    if (rd_M0 != X0 && rd_M0 == rs2_D) begin
                        need_stall = 1'b1;
                    end
                end
            end
        end

        // need to insert a bubble in IF/ID since there is a one cycle delay for IMEM
        // if ((opcode_E == OP_BRANCH && br_cond_E) || opcode_E == OP_JAL || opcode_E == OP_JALR) begin
        //     need_insert_bubble_D = 1'b1;
        // end

    end

endmodule: Locate_Hazard


module Stall_Handler
    (input  logic clk, rst_l, syscall_exit,
     input  logic need_stall, need_insert_bubble_D, mispredict,
                  mem_stall,
     output logic stall_pc, insert_bubble_D, insert_bubble_F,
                  instr_stall,
     output logic [3:0] pipeline_reg_state);

     enum logic [3:0] {BEGIN, START, S1, S2, RUNNING, MISPRED, M0, M1, M2, M3} curr_state, next_state;

    // next state logic
    always_comb begin
        unique case(curr_state)
            BEGIN: begin
                next_state = START;
            end
            START: begin
                next_state = S1;
            end
            S1: begin
                next_state = S2;
            end
            S2: begin
                next_state = RUNNING;
            end
            RUNNING: begin
                next_state = mispredict && !syscall_exit ? MISPRED : curr_state;
            end
            MISPRED: begin
                next_state = RUNNING;
            end

            //not visited
            M0: begin
                next_state = M1;
            end
            M1: begin
                next_state = M2;
            end
            M2: begin
                next_state = M3;
            end
            M3: begin
                next_state = RUNNING;
            end

        endcase
    end

    // output logic
    always_comb begin

        stall_pc = 1'b0;
        insert_bubble_D = 1'b0;
        insert_bubble_F = 1'b0;
        instr_stall = 1'b0;
        // pipeline_reg_state = 4'b1111;
        unique case(curr_state)
            BEGIN: begin
                pipeline_reg_state = 4'b1000;
                //insert_bubble_F = 1'b1;
            end
            START: begin
                pipeline_reg_state = 4'b1000;
                // stall_pc = 1'b1;
                insert_bubble_F = 1'b1;
            end
            S1: begin
                pipeline_reg_state = 4'b1100;
            end
            S2: begin
                if (need_stall && !mispredict) begin
                    pipeline_reg_state = 4'b0110; // ID/EX must store 1 bubble
                    insert_bubble_D = 1'b1;
                    stall_pc = 1'b1;
                    instr_stall = 1'b1;
                end
                else begin
                    insert_bubble_D = need_insert_bubble_D ? 1'b1 : 1'b0;
                    pipeline_reg_state = 4'b1110;
                end
            end

            RUNNING: begin
                if (mem_stall) begin
                    pipeline_reg_state = 4'b0000; // only advance WB stage
                    stall_pc = 1'b1;
                    instr_stall = 1'b1;
                end
                else if (need_stall && !mispredict) begin
                    pipeline_reg_state = 4'b0111; // ID/EX must store 1 bubble
                    insert_bubble_D = 1'b1;
                    stall_pc = 1'b1;
                    instr_stall = 1'b1;
                end
                else begin
                    insert_bubble_D = need_insert_bubble_D ? 1'b1 : 1'b0;
                    pipeline_reg_state = (syscall_exit) ? 4'b0111 : 4'b1111;
                    stall_pc = (syscall_exit) ? 1'b1 : 1'b0;
                    instr_stall = (syscall_exit) ? 1'b1 : 1'b0;
                end
            end
            MISPRED: begin
                insert_bubble_F = 1'b1;
                insert_bubble_D = 1'b1;
                pipeline_reg_state = (syscall_exit) ? 4'b0111 : 4'b1111;
                stall_pc = (syscall_exit) ? 1'b1 : 1'b0;
                instr_stall = (syscall_exit) ? 1'b1 : 1'b0;
            end
            M0: begin
                    pipeline_reg_state = 4'b0000; // ID/EX must store 1 bubble
                    stall_pc = 1'b1;
                    instr_stall = 1'b1;
            end
            M1: begin
                pipeline_reg_state = 4'b0000; // ID/EX must store 1 bubble
                    stall_pc = 1'b1;
                    instr_stall = 1'b1;
            end
            M2: begin
                    pipeline_reg_state = 4'b0000; // ID/EX must store 1 bubble
                    stall_pc = 1'b1;
                    instr_stall = 1'b1;
            end
            M3: begin
                    pipeline_reg_state = 4'b1111;
            end
        endcase
    end


     always_ff @(posedge clk, negedge rst_l) begin
         if (~rst_l) begin
             curr_state <= START;
         end
         else begin
            curr_state <= next_state;
         end
     end

endmodule: Stall_Handler



module Forwarding_Unit
    (input logic [31:0] instr_D, instr_E, instr_M,
     //instr_M1, instr_M2, instr_M3, instr_M4,
     input ctrl_signals_t ctrl_signals_E, ctrl_signals_M,
     // ctrl_signals_M1, ctrl_signals_M2, ctrl_signals_M3, ctrl_signals_M4,
     //input logic        cache_hit_M0, cache_hit_M1, cache_hit_M2, cache_hit_M3,
     output logic [3:0] fwd_1, fwd_2,
     output logic [1:0] fwd_sel_M, fwd_sel_E /*, fwd_sel_M1, fwd_sel_M2, fwd_sel_M3, fwd_sel_M4, */ );

    logic [4:0] rd_E, rd_M0, rs1_D, rs2_D;

    opcode_t opcode_E, opcode_D, opcode_M0; //opcode_M1, opcode_M2, opcode_M3, opcode_M4;
    funct7_t funct7_E, funct7_M0; //funct7_M1, funct7_M2, funct7_M3, funct7_M4;


    assign opcode_D = opcode_t'(instr_D[6:0]);
    assign opcode_E = opcode_t'(instr_E[6:0]);
    assign opcode_M0 = opcode_t'(instr_M[6:0]);

    assign funct7_E = funct7_t'(instr_E[31:25]);
    assign funct7_M0 = funct7_t'(instr_M[31:25]);

    assign rd_E = instr_E[11:7];
    assign rd_M0 = instr_M[11:7];

    assign rs1_D = instr_D[19:15];
    assign rs2_D = instr_D[24:20];


    always_comb begin

        fwd_1 = 4'd0;
        fwd_2 = 4'd0;
        fwd_sel_M = 2'b00;
        /*
        fwd_sel_M1 = 2'b00;
        fwd_sel_M2 = 2'b00;
        fwd_sel_M3 = 2'b00;
        fwd_sel_M4 = 2'b00;
        */
        fwd_sel_E = 2'b00;


            // LUI, AUIPC, JAL never forwarded to since they don't use rs1 or rs2
        if (opcode_D != OP_LUI && opcode_D != OP_AUIPC && opcode_D != OP_JAL) begin

        // dist 1 forwarding ///////////////////////////////////////////////////

            if (opcode_E == OP_LUI) begin
                // result of EX is immediate
                fwd_sel_E = 2'b10;
            end
            else if (opcode_E == OP_JALR || opcode_E == OP_JAL) begin
                fwd_sel_E = 2'b01; // forward pc+4
            end

            // never forward when MUL/LOAD/STORE/BRANCH in EX stage
            // MUL/LOAD not resolved, BRANCH/STORE have no rd
            if (opcode_E != OP_LOAD && !(opcode_E == OP_OP && funct7_E == FUNCT7_MULDIV) && opcode_E != OP_STORE && opcode_E != OP_BRANCH) begin

                // check rs1 dependency
                if (rd_E != X0 && rd_E == rs1_D && opcode_D != OP_SYSTEM) begin
                    fwd_1 = 4'd1; // forward val from ex stage to rs1
                end
                else if (opcode_D == OP_SYSTEM) begin
                    if (rd_E == X10)
                        fwd_1 = 4'd1;
                end

                // check rs2 dependency
                if (opcode_D != OP_IMM && opcode_D != OP_JALR && opcode_D != OP_SYSTEM) begin
                    if (rd_E != X0 && rd_E == rs2_D) begin
                        fwd_2 = 4'd1; // forward val from ex stage to rs2
                    end
                end

            end


        // dist 2 forwarding ///////////////////////////////////////////////////

        if (opcode_M0 == OP_LOAD) begin
            fwd_sel_M = 2'b10;
        end
        else if (opcode_M0 == OP_LUI) begin
            fwd_sel_M = 2'b01; // result of MEM is immediate
        end
        else if (opcode_M0 == OP_JAL || opcode_M0 == OP_JALR)
            fwd_sel_M = 2'b11; // forward pc+4

        // never forward when STORE/BRANCH in MEM (no rd)
        if (opcode_M0 != OP_STORE && opcode_M0 != OP_BRANCH) begin
            if (fwd_1 == 4'd0) begin

                if (rd_M0 != X0 && rd_M0 == rs1_D && opcode_D != OP_SYSTEM) begin
                    if (opcode_M0 == OP_OP && funct7_M0 == FUNCT7_MULDIV)
                        fwd_1 = 4'd9;  // forward multiplier output
                    else
                        fwd_1 = 4'd2;  // forward Mem stage result
                end
                else if (opcode_D == OP_SYSTEM) begin
                    if (rd_M0 == X10)
                        fwd_1 = 4'd1;
                end
            end

            // only forward to rs2 if not forwarding from EX stage
            if (fwd_2 == 4'd0 && (opcode_D != OP_IMM && opcode_D != OP_JALR && opcode_D != OP_SYSTEM)) begin
                if (rd_M0 != X0 && rd_M0 == rs2_D) begin
                    if (opcode_M0 == OP_OP && funct7_M0 == FUNCT7_MULDIV)
                        fwd_2 = 4'd9;  // forward multiplier output
                    else
                        fwd_2 = 4'd2;  // forward Mem stage result
                end
            end
        end

      end
  end

endmodule: Forwarding_Unit
