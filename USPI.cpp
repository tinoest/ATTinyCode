#include "USPI.h"

USPI::USPI(void)
{

}

void USPI::init (void) 
{

  USPI_DDR_PORT |= _BV(USPI_USCK_PIN);     // set the USCK pin as output
  USPI_DDR_PORT |= _BV(USPI_MISO_PIN);     // set the MISO pin as output
  USPI_DDR_PORT &= ~_BV(USPI_MOSI_PIN);    // set the MOSI pin as input

  // ATtiny
  USICR = (1<<USIWM0);  

}

uint8_t USPI::transfer (uint8_t outputData) 
{

  USIDR = outputData;
  USISR = (1<<USIOIF); // clear OVF flag
  do {
    USICR = (1<<USIWM0)|(1<<USICS1)|(1<<USICLK)|(1<<USITC);
  } 
  while ((USISR & (1<<USIOIF)) == 0);

  return USIDR;        // return the reply

}

void USPI::end(void)
{
  USICR &= ~(_BV(USIWM1) | _BV(USIWM0));
}

