#include "USPI.h"

USPI::USPI(void)
{

}

void USPI::init (uint8_t cs_pin) {

  bitSet(SS_PORT, cs_pin);
  bitSet(SS_DDR, cs_pin);
  digitalWrite(SPI_SS, HIGH);
  pinMode(SPI_SS, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);
  pinMode(SPI_SCK, OUTPUT);

  // ATtiny
  USICR = bit(USIWM0);  

  pinMode(RFM_IRQ, INPUT);
  digitalWrite(RFM_IRQ, HIGH); // pull-up
}

uint8_t USPI::transfer (uint8_t outputData) {
  
  USIDR = outputData;
  USISR = (1<<USIOIF); // clear OVF flag
  do {
    USICR = (1<<USIWM0)|(1<<USICS1)|(1<<USICLK)|(1<<USITC);
  } while ((USISR & (1<<USIOIF)) == 0);
  
  return USIDR;        // return the reply
  
}
