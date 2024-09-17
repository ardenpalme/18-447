/**
 * lib.sv
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This is the library of standard components used by the RISC-V processor,
 * which includes both synchronous and combinational components.
 **/

/*----------------------------------------------------------------------------*
 *  You may edit this file and add or change any files in the src directory.  *
 *----------------------------------------------------------------------------*/

// Force the compiler to throw an error if any variables are undeclared
`default_nettype none


`include "internal_defines.vh"      // Control signals struct, ALU ops
/*--------------------------------------------------------------------------------------------------------------------
 * Combinational Components
 *--------------------------------------------------------------------------------------------------------------------*/

/**
 * Selects on input from INPUTS inputs to output, each of WIDTH bits.
 *
 * Parameters:
 *  - INPUTS    The number of values from which the mux can select.
 *  - WIDTH     The number of bits each value contains.
 *
 * Inputs:
 *  - in        The values from which to select, packed together as a single
 *              bit-vector.
 *  - sel       The value from the inputs to output.
 *
 * Outputs:
 *  - out       The selected output from the inputs.
 **/
module mux
    #(parameter INPUTS=0, WIDTH=0)
    (input  logic [INPUTS-1:0][WIDTH-1:0]   in,
     input  logic [$clog2(INPUTS)-1:0]      sel,
     output logic [WIDTH-1:0]               out);

    assign out = in[sel];

endmodule: mux

/**
 * Adds two numbers of WIDTH bits, with a carry in bit, producing a sum and a
 * carry out bit.
 *
 * Parameters:
 *  - WIDTH     The number of bits of the numbers being summed together.
 *
 * Inputs:
 *  - cin       The carry in to the addition.
 *  - A         The first number to add.
 *  - B         The second number to add.
 *
 * Outputs:
 *  - cout      The carry out from the addition.
 *  - sum       The result of the addition.
 **/
module adder
    #(parameter WIDTH=0)
    (input  logic               cin,
     input  logic [WIDTH-1:0]   A, B,
     output logic               cout,
     output logic [WIDTH-1:0]   sum);

     assign {cout, sum} = A + B + cin;

endmodule: adder


/**
 * The arithmetic-logic unit (ALU) for the RISC-V processor.
 *
 * The ALU handles executing the current instruction, producing the
 * appropriate output based on the ALU operation specified to it by the
 * decoder.
 *
 * Inputs:
 *  - alu_src1      The first operand to the ALU.
 *  - alu_src2      The second operand to the ALU.
 *  - alu_op        The ALU operation to perform.
 * Outputs:
 *  - alu_out       The result of the ALU operation on the two sources.
 **/
module riscv_alu
    (input  logic [31:0]    alu_src1,
     input  logic [31:0]    alu_src2,
     input  alu_op_t        alu_op,
     output logic           bcond,
     output logic [31:0]    alu_out);

    logic [31:0]    sum;
    adder #($bits(alu_src1)) ALU_Adder(.A(alu_src1), .B(alu_src2), .cin(1'b0),
            .sum, .cout());

    always_comb begin
        unique case (alu_op)
            ALU_ADD:  alu_out = sum;
            ALU_SUB:  alu_out = alu_src1 - alu_src2;
            ALU_AND:  alu_out = alu_src1 & alu_src2;
            ALU_OR:   alu_out = alu_src1 | alu_src2;
            ALU_XOR:  alu_out = alu_src1 ^ alu_src2;
            ALU_SLT:  alu_out = $signed(alu_src1) < $signed(alu_src2);
            ALU_SLTU: alu_out = alu_src1 < alu_src2;
            ALU_SLL:  alu_out = alu_src1 << alu_src2[4:0];
            ALU_SRL:  alu_out = alu_src1 >> alu_src2[4:0];
            ALU_SRA:  begin
                alu_out = alu_src1 >> (alu_src2 & 32'h0000_001F);
                if (alu_src1[31] == 1'b1)
                    alu_out = alu_out | (32'hFFFF_FFFF << (5'd31 - alu_src2[4:0]));
            end
            default:  alu_out = 'd0;
        endcase
    end

    always_comb begin
        unique case (alu_op)
            ALU_BEQ:  bcond = alu_src1 == alu_src2;
            ALU_BNE:  bcond = alu_src1 != alu_src2;
            ALU_BLT:  bcond = $signed(alu_src1) < $signed(alu_src2);
            ALU_BGE:  bcond = $signed(alu_src1) >= $signed(alu_src2);
            ALU_BLTU: bcond = alu_src1 < alu_src2;
            ALU_BGEU: bcond = alu_src1 >= alu_src2;
            default:  bcond = 1'b0;
        endcase
    end

endmodule: riscv_alu

/*--------------------------------------------------------------------------------------------------------------------
 * Synchronous Components
 *--------------------------------------------------------------------------------------------------------------------*/

/**
 * Latches and stores values of WIDTH bits and initializes to RESET_VAL.
 *
 * This register uses an asynchronous active-low reset and a synchronous
 * active-high clear. Upon clear or reset, the value of the register becomes
 * RESET_VAL.
 *
 * Parameters:
 *  - WIDTH         The number of bits that the register holds.
 *  - RESET_VAL     The value that the register holds after a reset.
 *
 * Inputs:
 *  - clk           The clock to use for the register.
 *  - rst_l         An active-low asynchronous reset.
 *  - clear         An active-high synchronous reset.
 *  - en            Indicates whether or not to load the register.
 *  - D             The input to the register.
 *
 * Outputs:
 *  - Q             The latched output from the register.
 **/
module register
   #(parameter                      WIDTH=0,
     parameter logic [WIDTH-1:0]    RESET_VAL='b0)
    (input  logic               clk, en, rst_l, clear,
     input  logic [WIDTH-1:0]   D,
     output logic [WIDTH-1:0]   Q);

     always_ff @(posedge clk, negedge rst_l) begin
         if (!rst_l)
             Q <= RESET_VAL;
         else if (clear)
             Q <= RESET_VAL;
         else if (en)
             Q <= D;
     end

endmodule:register

module Counter
  #(parameter WIDTH=4)
  (input  logic             clk, en, clear, load, up,
   input  logic [WIDTH-1:0] D,
   output logic [WIDTH-1:0] Q);

  always_ff @(posedge clk)
    if (clear)
      Q <= 'd0;
    else if (load)
      Q <= D;
    else if (en & up)
      Q <= Q + 'd1;
    else if (en & ~up)
      Q <= Q - 'd1;

endmodule : Counter
