/**
 * internal_defines.vh
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This contains the definitions of constants and types that are used by the
 * core of the RISC-V processor, such as control signals and ALU operations.
 **/

/*----------------------------------------------------------------------------*
 *  You may edit this file and add or change any files in the src directory.  *
 *----------------------------------------------------------------------------*/

`ifndef INTERNAL_DEFINES_VH_
`define INTERNAL_DEFINES_VH_

// Constants that specify which operation the ALU should perform
typedef enum logic [3:0] {
    ALU_ADD,                // Addition operation
    ALU_SUB,                // Subtraction Operation
    ALU_AND,                // AND Operation
    ALU_OR,                 // OR Operation
    ALU_XOR,                // XOR Operation
    ALU_SLT,                // SLT Operation
    ALU_SLTU,               // SLTU Operation
    ALU_SLL,                // Left Shift
    ALU_SRL,                // Right Shift
    ALU_SRA,                // Arithemtic Right Shift

    ALU_BEQ,                // Branch conditions
    ALU_BNE,
    ALU_BLT,
    ALU_BGE,
    ALU_BLTU,
    ALU_BGEU,

    ALU_DC = 'bx            // Don't care value
} alu_op_t;

/* The definition of the control signal structure, which contains all
 * microarchitectural control signals for controlling the MIPS datapath. */
typedef struct packed {
    logic rtype;                 // Indicates if the current instruction is R-type
    logic rd_we;                 // Indicates to write data back to the register file
    logic mem_to_reg;            // Indicates if we write the dataMemory result or ALU result to RF
    logic data_load_en;          // Indicates whether we are reading from memory
    logic is_Uinstr;             // Indicates if the current instruction is an LUI instr
    logic is_J_Rd;               // Indicates if current instr. is a jump (to load PC+4 into rd)
    logic is_J_PC;               // Indicates if current instr. is a jump (to target_addr into PC)
    logic is_pc_alu_in;          // Indicates if the we should load PC into rs1 of ALU
    logic syscall;               // Indicates if the current instruction is a syscall
    logic illegal_instr;         // Indicates if the current instruction is illegal
    logic is_mul;                // Indicates if the current instr is a mul
    alu_op_t alu_op;             // The ALU operation to perform
} ctrl_signals_t;

// Struct to hold all counter values
typedef struct packed{

    int cycles;                 // # of cycles elapsed
    int instr_fetch;            // # of cycles fetched
    int instr_exec;             // # of cycles actually executed

    int fwd_br_exec;            // # of forward branches executed
    int bkwd_br_exec;           // # of backward branches executed

    int fwd_br_not_predict;     // # of forward branches seen w/o BTB prediction
    int bkwd_br_not_predict;    // # of backward branches seen w/o BTB prediction

    int fwd_br_taken;           // # of forward branches taken
    int bkwd_br_taken;          // # of backward branches taken

    int fwd_br_taken_correct;   // # of forward branches predicted taken correctly
    int bkwd_br_taken_correct;  // # of backward branches predicted taken correctly

    int fwd_br_ntaken_correct; // # of forward branhes predicted not taken correctly
    int bkwd_br_ntaken_correct; // # of backward branches predicted not taken correctly

    int jal_ex_funct;           // # of JAL instructions executed w/ rd = x1
    int jal_ex_nfunct;          // # of JAL instructions executed w/ rd != x1

    int jal_no_btb_funct;       // # of JAL instructions w/out BTB prediction w/ rd = x1
    int jal_no_btb_nfunct;      // # of JAL instructions w/out BTB prediction w/ rd != x1

    int jal_pred_correct_funct; // # of JAL instructions correctly predicted w/ rd = x1
    int jal_pred_correct_nfunct;// # of JAL instructions correctly predicted w/ rd != x1

    int jalr_ex_funct;          // # of JALR instructions executed w/ rd = x1
    int jalr_ex_nfunct;         // # of JALR instructions executed w/ rd != x1

    int jalr_no_btb_funct;      // # of JALR instructions w/out BTB prediction w/ rd = x1
    int jalr_no_btb_nfunct;     // # of JALR instructions w/out BTB prediction w/ rd != x1

    int jalr_pred_correct_funct; // # of JALR instructions correctly predicted w/ rd = x1
    int jalr_pred_correct_nfunct;// # of JALR instructions correctly predicted w/ rd != x1

} counters_t;

`endif /* INTERNAL_DEFINES_VH_ */
