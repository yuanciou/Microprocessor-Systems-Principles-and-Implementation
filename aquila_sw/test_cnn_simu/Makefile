# ======================================================================
#  This Makefile builds an application *.elf for Aquila.
#
#  Chun-Jen Tsai, Nov/29/2021
# ======================================================================
RISCV = /mnt/c/Course/mpd/riscv-gnu-toolchain
CROSS = riscv32-unknown-elf
CCPATH = $(RISCV)/bin

CC = $(CCPATH)/$(CROSS)-gcc
LD = $(CCPATH)/$(CROSS)-ld
AR = $(CCPATH)/$(CROSS)-ar
OD = $(CCPATH)/$(CROSS)-objdump
SP = $(CCPATH)/$(CROSS)-strip

GCCVERSION = $(shell $(CC) --version | grep gcc | sed 's/^.* //g')

LIBC = ../elibc
LIBC_OBJS = $(LIBC)/crt0.o $(LIBC)/stdio.o $(LIBC)/stdlib.o $(LIBC)/string.o $(LIBC)/time.o $(LIBC)/uart.o


CCFLAGS = -Wall -O2 -I$(LIBC) -fno-builtin -march=rv32ima_zicsr_zifencei -mstrict-align -mabi=ilp32
LDFLAGS = -L. -Map=$*.map
LD_SOFT_FP = -L$(RISCV)/lib/gcc/riscv32-unknown-elf/$(GCCVERSION) -lgcc

OCFLAGS = -O binary
ODFLAGS = -d

PROJ = test
OBJS = $(PROJ).o 

all:
	make $(PROJ).elf

clean:
	rm -f *.elf *.objdump *.map *.o *.a

%.o: %.c $(HDRS)
	$(CC) $(CCFLAGS) -c $< -o $@

libelibc.a : $(LIBC_OBJS) 
	$(AR) r libelibc.a $(LIBC_OBJS) 
	rm $^

%.elf: $(OBJS) libelibc.a
	$(LD) $(LDFLAGS) -T$(PROJ).ld $(OBJS) -lelibc $(LD_SOFT_FP) -o $@
	$(OD) $(ODFLAGS) $@ > $*.objdump
	$(SP) $@
	rm $^