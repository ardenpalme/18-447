
447inputs/addtest.elf:     file format elf32-littleriscv


Disassembly of section .text:

00400000 <main>:
  400000:	20000293          	addi	x5,x0,512
  400004:	00500333          	add	x6,x0,x5
  400008:	006303b3          	add	x7,x6,x6
  40000c:	00738e33          	add	x28,x7,x7
  400010:	00a00513          	addi	x10,x0,10
  400014:	00000073          	ecall
