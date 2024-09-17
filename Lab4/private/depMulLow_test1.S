    .text                       # Declare the code to be in the .text segment
    .global main                # Make main visible to the linker
main:
    # Initialize the registers with values.
    li      x0,  0x2b8b4568
    li      x1,  0xf27b23c7
    li      x2,  0x243c986a
    li      x3,  0x26334874
    li      x4,  0x34b0dc52
    li      x5,  0xd9495d00
    li      x6,  0xeae8944b
    li      x7,  0x225558ed
    li      x8,  0xe38e1f2a
    li      x9,  0x6e87cce
    li      x10, 0xfd1b58bb
    li      x11, 0x107ed7ac
    li      x12, 0xeeb141f3
    li      x13, 0x1b71efc
    li      x14, 0x39e2a9e4
    li      x15, 0x3545e147
    li      x16, 0x115f007d
    li      x17, 0x1bd062c3
    li      x18, 0xd2200855
    li      x19, 0xdb127f9
    li      x20, 0xc216231c
    li      x21, 0xdf16e9e9
    li      x22, 0xd190cde8
    li      x23, 0x26ef438e
    li      x24, 0xd40e0f77
    li      x25, 0xf352255b
    li      x26, 0xd09cf92f
    li      x27, 0xcded7264
    li      x28, 0x3fdcc234
    li      x29, 0xdbefd7a0
    li      x30, 0x1a7c4ca
    li      x31, 0x2b68079b

    # Tests dependencies at various hazard distances.
    mul     x5,  x13, x27       # 0  0  0       [-nan]
    mul     x1,  x17, x24       # 1  1  1       [1.000000]
    addi    x3,  x26, 212       # 2  2  2       [1.500000]
    addi    x7,  x2,  1716      # 3  3  3       [2.000000]
    mul     x20, x28, x27       # 4  4  4       [2.500000]
    addi    x18, x24, -1471     # 5  5  5       [3.000000]
    mul     x12, x22, x15       # 6  6  6       [3.500000]
    mul     x15, x9,  x25       # 7  7  7       [4.000000]
    mul     x19, x19, x20       # 8  8  4       [4.250000]
    addi    x18, x18, 254       # 4  4  9       [4.500000]
    mul     x4,  x19, x11       # 10 2  10      [4.650000]
    addi    x26, x28, -132      # 11 11 11      [5.227273]
    addi    x26, x11, 1161      # 1  12 12      [5.791667]
    addi    x29, x25, -79       # 13 13 13      [6.346154]
    mul     x15, x22, x2        # 7  14 14      [6.892857]
    addi    x7,  x17, 1994      # 12 15 15      [7.433333]
    addi    x18, x16, -156      # 7  16 10      [7.781250]
    addi    x19, x25, -228      # 9  17 16      [8.294118]
    mul     x22, x11, x19       # 18 18 1       [8.361111]
    mul     x3,  x24, x5        # 17 19 19      [8.921053]
    mul     x10, x21, x29       # 20 20 7       [9.150000]
    addi    x18, x27, 1563      # 5  21 2       [9.261905]
    mul     x3,  x18, x29       # 3  1  9       [9.068182]
    addi    x18, x15, -1767     # 2  9  23      [9.369565]
    addi    x5,  x24, 1761      # 24 24 24      [9.979167]
    addi    x15, x5,  -2007     # 11 1  25      [10.100000]
    addi    x5,  x21, 714       # 2  26 3       [10.269231]
    mul     x26, x25, x24       # 15 27 27      [10.888889]
    mul     x14, x8,  x3        # 28 28 6       [11.107142]
    mul     x21, x13, x13       # 29 29 29      [11.724138]
    mul     x4,  x10, x19       # 20 10 13      [11.716666]
    mul     x17, x16, x29       # 31 31 18      [12.129032]
    addi    x21, x26, -1900     # 3  5  7       [11.937500]
    addi    x1,  x20, 670       # 32 29 5       [12.090909]
    mul     x24, x0,  x2        # 34 34 34      [12.735294]
    addi    x5,  x9,  -630      # 9  35 35      [13.371428]
    mul     x3,  x4,  x13       # 14 6  36      [13.583333]
    addi    x25, x30, 1708      # 37 37 37      [14.216216]
    addi    x5,  x28, -467      # 3  38 38      [14.842105]
    mul     x30, x6,  x24       # 39 39 5       [15.025641]
    mul     x25, x21, x26       # 3  8  13      [14.912500]
    mul     x28, x5,  x26       # 41 3  14      [14.756098]
    mul     x14, x19, x18       # 14 25 19      [14.928572]
    mul     x27, x16, x4        # 43 43 13      [15.232558]
    mul     x9,  x13, x20       # 44 44 40      [15.840909]
    mul     x26, x24, x15       # 18 11 20      [15.833333]
    addi    x11, x17, 1044      # 46 15 29      [15.967391]
    mul     x1,  x23, x30       # 14 47 8       [16.212767]
    mul     x30, x15, x20       # 9  23 44      [16.572916]
    addi    x29, x1,  1897      # 36 2  3       [16.285715]
    addi    x29, x12, 1641      # 1  44 32      [16.719999]
    mul     x4,  x22, x3        # 21 33 15      [16.862745]
    addi    x25, x18, -876      # 12 29 1       [16.826923]
    mul     x8,  x13, x14       # 53 53 11      [17.113207]
    addi    x26, x17, -1051     # 9  23 34      [17.324074]

    li      x10, 0xa            # a0 (x10) = 0xa
    ecall                       # Terminate the simulation by passing 0xa to
