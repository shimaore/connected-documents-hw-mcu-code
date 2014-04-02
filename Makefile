CC=avr-gcc
LD=avr-ld
MCU_TARGET=attiny2313a
DUDE_TARGET=attiny2313
DUDE_PROG=stk500v2
AVRDUDE=avrdude
PORT=/dev/ttyUSB0
override CFLAGS=-Wall -g -O4 -Os -fshort-enums -mmcu=${MCU_TARGET} -std=gnu99
override LDFLAGS=-Os

%.o: %.c
	${CC} ${CFLAGS} -c $<

%.elf: %.o
	${LD} ${LDFLAGS} -Map $@.map -o $@ $< /usr/lib/avr/lib/avr25/libc.a /usr/lib/avr/lib/avr25/crttn2313a.o

%.elf.lst: %.elf
	avr-objdump -h -S $< > $@

%.o.lst: %.o
	avr-objdump -h -S $< > $@

%.hex: %.elf
	avr-objcopy -j .text -j .data -O ihex $< $@

program-%: %.hex
	# Write code out
	${AVRDUDE} -c ${DUDE_PROG} -p ${DUDE_TARGET} -P ${PORT} -U flash:w:$<

current: main.elf.lst

clean:
	rm -f *.o *.elf *.elf.map *.lst *.hex

# Project-specific
install: program-main

erase:
	${AVRDUDE} -c ${DUDE_PROG} -p ${DUDE_TARGET} -P ${PORT} -e

show-fuses:
	${AVRDUDE} -c ${DUDE_PROG} -p ${DUDE_TARGET} -P /dev/ttyUSB0  -U lfuse:r:/dev/stdout:h
	${AVRDUDE} -c ${DUDE_PROG} -p ${DUDE_TARGET} -P /dev/ttyUSB0  -U hfuse:r:/dev/stdout:h
	${AVRDUDE} -c ${DUDE_PROG} -p ${DUDE_TARGET} -P /dev/ttyUSB0  -U efuse:r:/dev/stdout:h

set-fuse:
	# Nothing to do here.

clear-fuse:
	# Note: bit=1 means "unprogrammed", bit=0 means "programmed"

	# This is the default: internal 8MHz clock divided by 8 = 1MHz clock
	#                                                                         Divide clock by 8
	#                                                                         |Output clock on CKOUT
	#                                                                         ||SUT1
	#                                                                         |||SUT0
	#                                                                         ||||CKSEL3
	#                                                                         |||||CKSEL2
	#                                                                         ||||||CKSEL1
	#                                                                         |||||||CKSEL0
	#                                                                         ||||||||
	${AVRDUDE} -c ${DUDE_PROG} -p ${DUDE_TARGET} -P /dev/ttyUSB0 -U lfuse:w:0b01100100:m

	#                                                                         DebugWire
	#                                                                         |EESAVE
	#                                                                         ||SPIEN
	#                                                                         |||WDTON
	#                                                                         ||||BODLEVEL2
	#                                                                         |||||BODLEVEL1
	#                                                                         ||||||BODLEVEL0
	#                                                                         |||||||RSTDISBL
	#                                                                         ||||||||
	${AVRDUDE} -c ${DUDE_PROG} -p ${DUDE_TARGET} -P /dev/ttyUSB0 -U hfuse:w:0b11011111:m
	# Default: BOD disabled

	#                                                                                SELFPRGEN
	#                                                                                |
	${AVRDUDE} -c ${DUDE_PROG} -p ${DUDE_TARGET} -P /dev/ttyUSB0 -U efuse:w:0b11111111:m
	# Self Programming: disabled

high-speed:
	echo sck 8.68 | ${AVRDUDE} -c ${DUDE_PROG} -p ${DUDE_TARGET} -P /dev/ttyUSB0 -t

slow-speed:
	echo sck 276.7 | ${AVRDUDE} -c ${DUDE_PROG} -p ${DUDE_TARGET} -P /dev/ttyUSB0 -t -F

voltage:
	echo vtarg 3.0 | ${AVRDUDE} -c ${DUDE_PROG} -p ${DUDE_TARGET} -P /dev/ttyUSB0 -t -F

all: slow-speed clear-fuse high-speed install set-fuse
