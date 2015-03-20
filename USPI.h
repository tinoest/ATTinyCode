#ifndef USPI_h
#define USPI_h

#include <stdlib.h>
#include "Arduino.h"

//ATTiny84
#define SS_DDR      DDRB
#define SS_PORT     PORTB

#define RFM_IRQ     2

#define SPI_SS      1     // PB1, pin 3
#define SPI_MISO    4     // PA6, pin 7
#define SPI_MOSI    5     // PA5, pin 8
#define SPI_SCK     6     // PA4, pin 9

class USPI
{
public:
  USPI(void);
  void init(uint8_t cs_pin);
  uint8_t transfer (uint8_t outputData);
private:

};

#endif

