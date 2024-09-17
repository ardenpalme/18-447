#define DATA_SEG_SIZE           20

    .data                       // Declare items to be in the .data segment
data:                           // Symbol representing the start of .data
    .space  DATA_SEG_SIZE       // Allocate 20 bytes for the .data segment
data_end:                       // Symbol representing the end of .data

    .text                       // Declare the code to be in the .text segment
    .global main                // Make main visible to the linker
main:
    add     x1,  zero,  gp      // At program start gp has the start of the
                                // .data segment. x1 = gp (x3)

    addi    x5,  zero,  255     // x5 = 255

    sw      x5,  0(x1)          // *(x1 + 0) = x5
    sw      x5,  4(x1)          // *(x1 + 4) = x6
    sw      x5,  8(x1)          // *(x1 + 8) = x7
    sw      x5,  12(x1)         // *(x1 + 12) = x8

    lw      x9,  0(x1)          // x9 = *(x1 + 0)
    lw      x10, 4(x1)          // x10 = *(x1 + 4)
    lw      x11, 8(x1)          // x11 = *(x1 + 8)
    lw      x12, 12(x1)         // x12 = *(x1 + 12)

    addi    x1,  x1,    4       // x1 = x1 + 4
    sw      x5,  0(x1)          // *(x1 + 0) = x5
    sw      x6,  4(x1)          // *(x1 + 4) = x6
    sw      x7,  8(x1)          // *(x1 + 8) = x7
    sw      x8,  12(x1)         // *(x1 + 12) = x8

    lw      x13, -4(x1)         // x13 = *(x1 - 4)
    lw      x14, 0(x1)          // x14 = *(x1 + 0)
    lw      x15, 4(x1)          // x15 = *(x1 + 4)
    lw      x16, 8(x1)          // x16 = *(x1 + 8)

    addi    a0,  zero,  0xa     // a0 (x10) = 0xa
    ecall                       // Terminate the simulation by passing 0xa to
                                // ecall in register a0 (x10).
