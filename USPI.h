#ifndef USPI_h
#define USPI_h

#include <stdlib.h>
#include "Arduino.h"

//ATTiny84
#define SS_DDR      DDRB
#define SS_PORT     PORTB

#define RFM_IRQ     2

//USI ports and pins
#if defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
#define USPI_DDR_PORT DDRA
#define USPI_USCK_PIN DDA4
#define USPI_MISO_PIN DDA5
#define USPI_MOSI_PIN DDA6
#endif

class USPI
{
public:
  USPI(void);
  void init(void);
  uint8_t transfer (uint8_t outputData);
  void end(void);
private:

};

#endif

