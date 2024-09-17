
447inputs/lwtest.elf:     file format elf32-littleriscv


Disassembly of section .text:

00400000 <main>:
  400000:	003000b3          	add	x1,x0,x3
  400004:	0ff00293          	addi	x5,x0,255
  400008:	00528333          	add	x6,x5,x5
  40000c:	006303b3          	add	x7,x6,x6
  400010:	7d038413          	addi	x8,x7,2000
  400014:	01008093          	addi	x1,x1,16
  400018:	0000a803          	lw	x16,0(x1)
  40001c:	0040a883          	lw	x17,4(x1)
  400020:	0080a903          	lw	x18,8(x1)
  400024:	00c0a983          	lw	x19,12(x1)
  400028:	ffc0aa03          	lw	x20,-4(x1)
  40002c:	ff80aa83          	lw	x21,-8(x1)
  400030:	ff40ab03          	lw	x22,-12(x1)
  400034:	ff00ab83          	lw	x23,-16(x1)
  400038:	00a00513          	addi	x10,x0,10
  40003c:	00000073          	ecall

Disassembly of section .data:

10000000 <data>:
10000000:	ffff                	0xffff
10000002:	ffff                	0xffff
10000004:	fffe                	c.fswsp	f31,252(x2)
10000006:	ffff                	0xffff
10000008:	fffd                	c.bnez	x15,10000006 <data+0x6>
1000000a:	ffff                	0xffff
1000000c:	fffc                	c.fsw	f15,124(x15)
1000000e:	ffff                	0xffff
10000010:	fff5                	c.bnez	x15,1000000c <data+0xc>
10000012:	ffff                	0xffff
10000014:	fff4                	c.fsw	f13,124(x15)
10000016:	ffff                	0xffff
10000018:	fffffff3          	csrrci	x31,0xfff,31
1000001c:	fff2                	c.fswsp	f28,252(x2)
1000001e:	ffff                	0xffff
	...
