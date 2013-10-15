NAME := serdac
MCU := atmega328
DEVICE := m328
#MCU := atmega328p
#DEVICE := m328p
MCU_FREQ := 11059200UL

# program device using avrispmkII
AVRDUDE := avrdude -c avrispmkII -P usb -p $(DEVICE)


EFUSE:= 0x07
HFUSE:= 0xd9
LFUSE:= 0xff
#---------------------------------------------------------------------
# ATMega328P
#---------------------------------------------------------------------
# Fuse extended byte (default 0xff):
# 0x07 = - - - -   - 1 1 1
#                    \-+-/
#                      +------ BODLEVEL 0..2 (111 = disabled)
#
# Fuse high byte (default 0xd9):
# 0xd9 = 1 1 0 1   1 0 0 1 <-- BOOTRST (0 = jump to bootloader at start)
#        ^ ^ ^ ^   ^  \/
#        | | | |   |   +------ BOOTSZ 0..1 (01 = 2KB starting at 0x7800)
#        | | | |   + --------- EESAVE (0 = preserve EEPROM over chip erase)
#        | | | +-------------- WDTON (1 = watchdog disabled at start)
#        | | +---------------- SPIEN (0 = allow serial programming)
#        | +------------------ DWEN (1 = debug wire disable)
#        +-------------------- RSTDISBL (1 = reset pin is enabled)
#
# Fuse low byte (default 0x62):
# 0xf7 = 1 1 1 1   0 1 1 1
#        ^ ^ \ /   \--+--/
#        | |  |       +------- CKSEL 3..0
#        | |  +--------------- SUT 1..0 (10 = startup time 14CK + 65ms)
#        | +------------------ CKOUT (1 = clock output disabled)
#        +-------------------- CKDIV8 (1 = do not divide clock by 8)


HEX := $(NAME).hex
OUT := $(NAME).out
MAP := $(NAME).map
SOURCES := $(wildcard *.c)
HEADERS := $(wildcard *.h)
OBJECTS := $(patsubst %.c,%.o,$(SOURCES))

CC := avr-gcc
OBJCOPY := avr-objcopy
SIZE := avr-size -A

CFLAGS := -Wall -mmcu=$(MCU) -std=c99 -g -Os -DF_CPU=$(MCU_FREQ) -lm -mcall-prologues
DEBUG_CFLAGS := -Wl,-u,vfprintf -lprintf_flt -DDEBUG=1



all: clean $(HEX)

debug: CFLAGS := $(CFLAGS) $(DEBUG_CFLAGS)
debug: all

clean:
	rm -f $(HEX) $(OUT) $(MAP) $(OBJECTS)

flash: $(HEX)
	$(AVRDUDE) -U flash:w:$(HEX)

fuse:
	$(AVRDUDE) -U lfuse:w:$(LFUSE):m -U hfuse:w:$(HFUSE):m -U efuse:w:$(EFUSE):m

$(HEX): $(OUT)
	$(OBJCOPY) -R .eeprom -O ihex $< $@

$(OUT): $(OBJECTS)
	$(CC) $(CFLAGS) -o $@ -Wl,-Map,$(MAP) $^
	@echo
	@$(SIZE) $@
	@echo

%.o: %.c $(HEADERS) Makefile
	$(CC) $(CFLAGS) -c -o $@ $<


.PHONY: all clean flash debug

