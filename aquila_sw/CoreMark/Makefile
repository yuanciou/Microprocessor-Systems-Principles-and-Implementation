# ======================================================================
#  This Makefile builds an application *.elf or a boot code memory
#  file *.mem for Aquila.
#
#  Chun-Jen Tsai, Jan/12/2020
# ======================================================================
RISCV = /mnt/c/Course/mpd/riscv-gnu-toolchain

CROSS = riscv32-unknown-elf
CCPATH = $(RISCV)/bin

CC = $(CCPATH)/$(CROSS)-gcc
LD = $(CCPATH)/$(CROSS)-ld
AR = $(CCPATH)/$(CROSS)-ar
OC = $(CCPATH)/$(CROSS)-objcopy
OD = $(CCPATH)/$(CROSS)-objdump
SP = $(CCPATH)/$(CROSS)-strip

GCCVERSION = $(shell $(CC) --version | grep gcc | sed 's/^.* //g')

LIBC = ../elibc
LIBC_OBJS = $(LIBC)/crt0.o $(LIBC)/stdio.o $(LIBC)/stdlib.o $(LIBC)/string.o $(LIBC)/time.o $(LIBC)/uart.o

# -fno-optimize-sibling-calls
CCFLAGS = -Wall -O2 -I$(LIBC) -fno-builtin -march=rv32ima_zicsr_zifencei -mstrict-align -mabi=ilp32 -fno-inline-small-functions
XCFLAGS = -DITERATIONS=0 -DUSE_CLOCK=1
LDFLAGS = -L. -Map=$*.map
LD_SOFT_FP = -L$(RISCV)/lib/gcc/riscv32-unknown-elf/$(GCCVERSION) -L$(RISCV)/riscv32-unknown-elf/lib -lgcc -lm

OCFLAGS = -O binary
ODFLAGS = -d

PROJ = coremark
OBJS = core_main.o core_portme.o core_util.o core_list_join.o core_matrix.o core_state.o

FLAGS_STR = "$(CCFLAGS) $(XCFLAGS)"

all:
	make MEM_TYPE=tcm $(PROJ).elf

tcm:
	make MEM_TYPE=tcm $(PROJ).elf

ddr:
	make MEM_TYPE=ddr $(PROJ).elf

clean:
	rm -f *.elf *.objdump *.map *.o *.a

%.o: %.c
	$(CC) $(CCFLAGS) $(XCFLAGS) $(LCFLAGS) -DFLAGS_STR=\"$(FLAGS_STR)\" -c $< -o $@

libelibc.a : $(LIBC_OBJS)
	$(AR) r libelibc.a $(LIBC_OBJS)
	rm $^

%.elf: XCFLAGS = -DITERATIONS=5 -DUSE_CLOCK=1
%.elf: $(OBJS) libelibc.a
	$(LD) $(LDFLAGS) -T$*_$(MEM_TYPE).ld $(OBJS) -lelibc $(LD_SOFT_FP) -o $@
	$(OD) $(ODFLAGS) $@ > $*.objdump
	$(SP) -g $@
	rm $^

