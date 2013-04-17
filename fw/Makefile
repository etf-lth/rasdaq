TARGET  =radaq_avr
MCU     =atmega88p
OBJS    =main.o

CC      =avr-gcc
OBJCOPY =avr-objcopy
SIZE    =avr-size

CFLAGS  =-g -Os -mmcu=$(MCU) -I. -Wall -DF_CPU=13000000UL

all: $(TARGET).hex

flash: $(TARGET).hex
	avrdude -c avrftdi -p$(MCU) -U flash:w:$^

#fuse:
	#avrdude -c avrftdi -p$(MCU) -U lfuse:w:0x62:m -U hfuse:w:0xdd:m -U efuse:w:0x01:m

	# med 13 MHz-kristall
	#avrdude -c avrftdi -p$(MCU) -U lfuse:w:0xcf:m -U hfuse:w:0xdd:m -U efuse:w:0x01:m

	# med intern oscillator
	#avrdude -c avrftdi -p$(MCU) -U lfuse:w:0xe2:m -U hfuse:w:0xdd:m -U efuse:w:0x01:m

size: $(OBJS)
	$(SIZE) $(OBJS)

clean:
	rm -f *.o

veryclean: clean
	rm -f $(TARGET).hex

$(TARGET).hex: $(TARGET).o
	$(OBJCOPY) -j .text -j .data -O ihex $(TARGET).o $@

$(TARGET).o: $(OBJS)
	$(CC) -g -mmcu=$(MCU) -Wall -o $@ $^
	$(SIZE) $@
