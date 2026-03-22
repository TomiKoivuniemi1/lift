MCU=atmega328p
F_CPU=16000000UL

AVR_BIN  = C:/Users/tomik/AppData/Local/Arduino15/packages/arduino/tools/avr-gcc/7.3.0-atmel3.6.1-arduino7/bin
AVRD_ROOT= C:/Users/tomik/AppData/Local/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino17
AVRD_BIN = $(AVRD_ROOT)/bin
AVRDUDECONF = $(AVRD_ROOT)/etc/avrdude.conf

CC      = $(AVR_BIN)/avr-gcc
OBJCOPY = $(AVR_BIN)/avr-objcopy
AVRDUDE = $(AVRD_BIN)/avrdude
PORT=COM3
BAUD=115200
PROGRAMMER=arduino
TARGET=main
SRC=main.c
CFLAGS=-mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -Wall
LDFLAGS=-mmcu=$(MCU)

all: $(TARGET).hex

$(TARGET).elf: $(SRC)
	$(CC) $(CFLAGS) $(SRC) -o $@ $(LDFLAGS)

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

flash: $(TARGET).hex
	$(AVRDUDE) -C "$(AVRDUDECONF)" -c $(PROGRAMMER) -P $(PORT) -b $(BAUD) -p m328p -D -U flash:w:$(TARGET).hex:i

clean:
	del /Q $(TARGET).elf $(TARGET).hex 2>nul || ver>nul
