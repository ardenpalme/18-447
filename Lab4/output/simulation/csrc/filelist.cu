PIC_LD=ld

ARCHIVE_OBJS=
ARCHIVE_OBJS += _20712_archive_1.so
_20712_archive_1.so : archive.7/_20712_archive_1.a
	@$(AR) -s $<
	@$(PIC_LD) -shared  -o .//../riscv_core.daidir//_20712_archive_1.so --whole-archive $< --no-whole-archive
	@rm -f $@
	@ln -sf .//../riscv_core.daidir//_20712_archive_1.so $@


ARCHIVE_OBJS += _20729_archive_1.so
_20729_archive_1.so : archive.7/_20729_archive_1.a
	@$(AR) -s $<
	@$(PIC_LD) -shared  -o .//../riscv_core.daidir//_20729_archive_1.so --whole-archive $< --no-whole-archive
	@rm -f $@
	@ln -sf .//../riscv_core.daidir//_20729_archive_1.so $@


ARCHIVE_OBJS += _20730_archive_1.so
_20730_archive_1.so : archive.7/_20730_archive_1.a
	@$(AR) -s $<
	@$(PIC_LD) -shared  -o .//../riscv_core.daidir//_20730_archive_1.so --whole-archive $< --no-whole-archive
	@rm -f $@
	@ln -sf .//../riscv_core.daidir//_20730_archive_1.so $@


ARCHIVE_OBJS += _prev_archive_1.so
_prev_archive_1.so : archive.7/_prev_archive_1.a
	@$(AR) -s $<
	@$(PIC_LD) -shared  -o .//../riscv_core.daidir//_prev_archive_1.so --whole-archive $< --no-whole-archive
	@rm -f $@
	@ln -sf .//../riscv_core.daidir//_prev_archive_1.so $@






%.o: %.c
	$(CC_CG) $(CFLAGS_CG) -c -o $@ $<
CU_UDP_OBJS = \


CU_LVL_OBJS = \
SIM_l.o 

MAIN_OBJS = \
amcQwB.o objs/amcQw_d.o 

CU_OBJS = $(MAIN_OBJS) $(ARCHIVE_OBJS) $(CU_UDP_OBJS) $(CU_LVL_OBJS)

