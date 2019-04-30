
TARGET = blink.bin
ELFBIN = $(TARGET:.bin=.elf)

CROSS_COMPILE = arm-none-linux-gnueabi-

CC = $(CROSS_COMPILE)gcc -marm -ffreestanding -O0 -g3 -fPIC
LD = $(CROSS_COMPILE)gcc -marm -fPIC
AS = $(CROSS_COMPILE)gcc -marm -fPIC

OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump

LDS = blink.lds

HDRS = $(wildcard *.h)
SRCS = $(wildcard *.c)
OBJS = $(SRCS:.c=.o)

CFLAGS = -I. -fdata-sections -ffunction-sections
LDFLAGS = -nostdlib --gc-sections

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

%.o: %.c
	$(CC) $(CFLAGS) -c $<

all: $(TARGET)

$(ELFBIN): $(OBJS) $(LDS)
	$(LD) $(LDFLAGS) -T $(LDS) -o $@ $(OBJS)

dis: $(ELFBIN)
	$(OBJDUMP) -DS $<

clean:
	$(RM) $(OBJS) *.bin *.elf *.bak *.old *~
