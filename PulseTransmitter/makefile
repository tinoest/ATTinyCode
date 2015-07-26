DEVICE			= attiny84
CLOCK				= 8000000UL
PROGRAMMER	= usbtiny
FILENAME		= main
COMPILE			= avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)
SOURCES			= $(wildcard *.cpp)
OBJECTS			= $(SOURCES:.cpp=.o)

all: clean build upload

build:
	$(COMPILE) -c $(SOURCES)
	$(COMPILE) -o $(FILENAME).elf $(OBJECTS)
	avr-objcopy -j .text -j .data -O ihex $(FILENAME).elf $(FILENAME).hex
	avr-size --format=avr --mcu=$(DEVICE) $(FILENAME).elf

%.o: %.cpp
	$(COMPILE) -c $< -o $@ 

upload:
	avrdude -v -p $(DEVICE) -c $(PROGRAMMER) -U flash:w:$(FILENAME).hex:i 

clean:
	rm $(OBJECTS)
	rm $(FILENAME).hex
	rm $(FILENAME).elf
