
447inputs/arithtest.elf:     file format elf32-littleriscv


Disassembly of section .text:

00400000 <main>:
  400000:	40000113          	addi	x2,x0,1024
  400004:	002101b3          	add	x3,x2,x2
  400008:	0021e233          	or	x4,x3,x2
  40000c:	4d200293          	addi	x5,x0,1234
  400010:	01029313          	slli	x6,x5,0x10
  400014:	3e730393          	addi	x7,x6,999
  400018:	40238433          	sub	x8,x7,x2
  40001c:	003244b3          	xor	x9,x4,x3
  400020:	0ff14513          	xori	x10,x2,255
  400024:	00535593          	srli	x11,x6,0x5
  400028:	40435613          	srai	x12,x6,0x4
  40002c:	0075f6b3          	and	x13,x11,x7
  400030:	0642f713          	andi	x14,x5,100
  400034:	40a007b3          	sub	x15,x0,x10
  400038:	00064837          	lui	x16,0x64
  40003c:	00a00513          	addi	x10,x0,10
  400040:	00000073          	ecall
