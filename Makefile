CC=arm-none-eabi-gcc
CSIZE=arm-none-eabi-size
CCOPY=arm-none-eabi-objcopy
CDUMP=arm-none-eabi-objdump

DC=ldc2
DC_MODULE=$(wildcard src/stm32/*.d) $(wildcard src/cm3/*.d) $(wildcard src/cm3/core/stdc/*.d) 
DC_CORE=src/cm3/core/stdc/

VERSION=stm32f1x
CPU=cortex-m3
CPU_SHORT=cm3
ARCH=v7m
PIO_DIR=.pio/build/$(VERSION)

NAME=firmware

CFLAGS= -mcpu=$(CPU) -mthumb -ffunction-sections -fdata-sections -Os -g -nostartfiles -specs=nano.specs -specs=nosys.specs -z noexecstack
DFLAGS= -d-version=$(CPU_SHORT),$(VERSION) -mtriple=thumb$(ARCH)-none-eabi -mcpu=$(CPU) -static -disable-linker-strip-dead -O0 -g -betterC -gcc=$(CC)

C_SOURCES := $(wildcard src/*.c)
C_OBJS    := $(patsubst src/%.c, $(PIO_DIR)/%.o, $(C_SOURCES))
D_SOURCES := $(wildcard src/*.d)
D_OBJS    := $(patsubst src/%.d, $(PIO_DIR)/%.o, $(D_SOURCES))

OBJS = $(C_OBJS) $(D_OBJS)

name_elf := $(PIO_DIR)/$(NAME).elf
name_bin := $(PIO_DIR)/$(NAME).bin
name_disasm := $(PIO_DIR)/$(NAME).disasm

COPY = $(CCOPY) -O binary $(name_elf) $(name_bin)
DUMP = $(CDUMP) -d $(name_elf) > $(name_disasm)

all: $(PIO_DIR) $(name_elf) copy dump size

$(name_elf): $(OBJS) | $(PIO_DIR)
	$(CC) $(CFLAGS) -Wl,-T,linker/$(VERSION).ld,-Map=$(PIO_DIR)/$(NAME).map,--gc-sections -o $@ $^

$(PIO_DIR):
	mkdir -p $(PIO_DIR)

$(PIO_DIR)/%.o: src/%.c | $(PIO_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(PIO_DIR)/%.o: src/%.d | $(PIO_DIR)
	$(DC) $(DFLAGS) -c -of=$@ $(DC_MODULE) $^

copy: $(name_elf)
	$(COPY)

dump: $(name_elf)
	$(DUMP)

size: 
	$(CSIZE) $(name_elf)

flash: $(name_elf)
	pio run

clean:
	$(RM) -r $(PIO_DIR)
