// USPI for Attiny84
// http://opensource.org/licenses/BSD-3-Clause
// 2015 Martyn Brown : http://tinoest.no-ip.org
#ifndef USPI_h
#define USPI_h

#include <stdlib.h>
#include <stdint.h>
#include<avr/io.h>

//USI ports and pins
#if defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
#define USPI_DDR_PORT DDRA
#define USPI_USCK_PIN DDA4
#define USPI_MISO_PIN DDA5
#define USPI_MOSI_PIN DDA6
#endif

#define USPI_MODE0 0
#define USPI_MODE1 1

class USPI
{
public:
  USPI(void);
  void init(void);
  uint8_t transfer (uint8_t outputData);
  void setDataMode(uint8_t mode);
  void end(void);
private:

};

#endif

