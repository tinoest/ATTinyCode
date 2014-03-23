/*
                 +-\/-+
           VCC  1|    |14  GND
      (D0) PB0  2|    |13  AREF (D10)
      (D1) PB1  3|    |12  PA1 (D9)
         RESET  4|    |11  PA2 (D8)
 INT0 (D2) PB2  5|    |10  PA3 (D7)
  PWM (D3) PA7  6|    |9   PA4 (D6)
  PWM (D4) PA6  7|    |8   PA5 (D5) PWM
                 +----+
*/

#include <stdlib.h>
#include <util/delay.h>
#include <JeeLib.h> // https://github.com/jcw/jeelib

ISR(WDT_vect) { 
  Sleepy::watchdogEvent(); 
} // interrupt handler for JeeLabs Sleepy power saving

#define myNodeID 8     // RF12 node ID in the range 1-30
#define network 210      // RF12 Network group
#define freq RF12_433MHZ // Frequency of RFM12B module

#define USE_ACK           // Enable ACKs, comment out to disable
#define RETRY_PERIOD 5    // How soon to retry (in seconds) if ACK didn't come in
#define RETRY_LIMIT 5     // Maximum number of times to retry
#define ACK_TIME 10       // Number of milliseconds to wait for an ack

#define BAUD            9600
#define STX_PORT        PORTA
#define STX_DDR         DDRA
#define STX_BIT         7 // Which port on PORTB to use 0 = D8 , 1 = D9 , 2 = D10

#define TMP_OFFSET      7

//#define DEBUG

int counter;

typedef struct {
  int counter;
  int tmpC;
  int supplyV;	// Supply voltage
} 
Payload;

Payload temptx;

void setup() {

#if defined(DEBUG)
  sinit();
#endif
  rf12_initialize(myNodeID,freq,network); // Initialize RFM12 with settings defined above 
  rf12_sleep(0);                          // Put the RFM12 to sleep

  PRR = bit(PRTIM1); // only keep timer 0 going

  ADCSRA &= ~ bit(ADEN); 
  bitSet(PRR, PRADC); // Disable the ADC to save power

}

void loop() {

  counter++;

  temptx.counter = (counter); // Get temperature reading and convert to integer, reversed at receiving end
  temptx.supplyV = readVcc(); // Get supply voltage
  temptx.tmpC    = readTmpC();

#if defined(DEBUG)
  char tmp[50];
  sprintf(tmp,"Count %i Supply %i TmpC %i\n", counter , temptx.supplyV , temptx.tmpC);
  sputs(tmp);
#endif

  rfwrite(); // Send data via RF 

  Sleepy::loseSomeTime(10000); //JeeLabs power save function: enter low power mode for 10 seconds (valid range 16-65000 ms)
}

// Wait a few milliseconds for proper ACK
#ifdef USE_ACK
static byte waitForAck() {
  MilliTimer ackTimer;
  while (!ackTimer.poll(ACK_TIME)) {
    if (rf12_recvDone() && rf12_crc == 0 &&
      rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | myNodeID))
      return 1;
  }
  return 0;
}
#endif


//--------------------------------------------------------------------------------------------------
// Send payload data via RF
//-------------------------------------------------------------------------------------------------
static void rfwrite(){
#ifdef USE_ACK
  for (byte i = 0; i <= RETRY_LIMIT; ++i) {  // tx and wait for ack up to RETRY_LIMIT times
    rf12_sleep(-1);              // Wake up RF module
    while (!rf12_canSend())
      rf12_recvDone();
    rf12_sendStart(RF12_HDR_ACK, &temptx, sizeof temptx); 
    rf12_sendWait(2);           // Wait for RF to finish sending while in standby mode
    byte acked = waitForAck();  // Wait for ACK
    rf12_sleep(0);              // Put RF module to sleep
    if (acked) { 
      return; 
    }      // Return if ACK received

    Sleepy::loseSomeTime(RETRY_PERIOD * 1000);     // If no ack received wait and try again
  }
#else
  rf12_sleep(-1);              // Wake up RF module
  while (!rf12_canSend())
    rf12_recvDone();
  rf12_sendStart(0, &temptx, sizeof temptx); 
  rf12_sendWait(2);           // Wait for RF to finish sending while in standby mode
  rf12_sleep(0);              // Put RF module to sleep
  return;
#endif
}

//--------------------------------------------------------------------------------------------------
// Read current supply voltage
//--------------------------------------------------------------------------------------------------
long readVcc() {
  bitClear(PRR, PRADC); 
  ADCSRA |= bit(ADEN); // Enable the ADC
  long result;
  // Read 1.1V reference against Vcc
  ADMUX = _BV(MUX5) | _BV(MUX0); // For ATtiny84
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate Vcc in mV
  ADCSRA &= ~ bit(ADEN); 
  bitSet(PRR, PRADC); // Disable the ADC to save power
  return result;
} 


//--------------------------------------------------------------------------------------------------
// Read Internal Temperature , Return in Degree C
//--------------------------------------------------------------------------------------------------
long readTmpC() {
  bitClear(PRR, PRADC); 
  ADCSRA |= bit(ADEN); // Enable the ADC
  long result;
  ADMUX = B00100010;                        // Select temperature sensor
  ADMUX &= ~_BV( ADLAR );                   // Right-adjust result
  ADMUX |= _BV( REFS1 );                    // Set Ref voltage
  ADMUX &= ~( _BV( REFS0 ) );               // to 1.1V
  // Configure ADCSRA
  ADCSRA &= ~( _BV( ADATE ) |_BV( ADIE ) ); // Disable autotrigger, Disable Interrupt
  ADCSRA |= _BV(ADEN);                      // Enable ADC
  delay(2);
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  ADCSRA &= ~ bit(ADEN); 
  bitSet(PRR, PRADC); // Disable the ADC to save power
  return result - 273 + TMP_OFFSET;
} 

// Serial Functions Start
#if defined(DEBUG)

void sputchar( uint8_t c )
{
  c = ~c;
  STX_PORT &= ~(1<<STX_BIT);            // start bit
  for( uint8_t i = 10; i; i-- ){        // 10 bits
    _delay_us( 1e6 / BAUD );            // bit duration
    if( c & 1 )
      STX_PORT &= ~(1<<STX_BIT);        // data bit 0
    else
      STX_PORT |= 1<<STX_BIT;           // data bit 1 or stop bit
    c >>= 1;
  }
} 

void sputs(const void *s ) {
  uint8_t *s1 = (uint8_t*)s;
  while( *s1 )
    sputchar( *s1++ );
}

void sinit() {

  STX_PORT |= 1<<STX_BIT;
  STX_DDR |= 1<<STX_BIT; 

}

#endif
// Serial Functions End
