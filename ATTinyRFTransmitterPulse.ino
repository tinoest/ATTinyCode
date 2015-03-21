/*
       +-\/-+
 VCC  1|    |14  GND
 PB0  2|    |13  AREF 
 PB1  3|    |12  PA1 
 RST  4|    |11  PA2 
 PB2  5|    |10  PA3 
 PA7  6|    |9   PA4 
 PA6  7|    |8   PA5
 +----+
 
 */

#include <stdlib.h>
#include <util/delay.h>
#include "RFM12.h"
#include "WatchdogSleep.h"

#define BAUD            9600
#define STX_PORT        PORTA
#define STX_DDR         DDRA
#define STX_BIT         7 // Which port on PORTB to use 0 = D8 , 1 = D9 , 2 = D10

#define TMP_OFFSET      7

//#define USE_ACK           // Enable ACKs, comment out to disable
#define RETRY_PERIOD 5    // How soon to retry (in seconds) if ACK didn't come in
#define RETRY_LIMIT 5     // Maximum number of times to retry
#define ACK_TIME 10       // Number of milliseconds to wait for an ack

volatile uint16_t pulse;

//ATTiny84
#define SS_BIT      1

// Enable the Watchdog Sleep
WatchdogSleep sleep;

// Pass the Slave Select Port Information
RFM12 radio(SS_BIT);

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

  // PB0 
  GIMSK  |= (1<<PCIE1);                     // enable Pin Change Interrupt 1 
  PCMSK1 = (1<<PCINT8);                     // enable PCINT8

  radio.init(10,RF12_433MHZ,210);           // Initialize RFM12 with settings defined above 
  radio.sleep(0);                           // Put the RFM12 to sleep
  PRR = bit(PRTIM1);                        // only keep timer 0 going
  ADCSRA &= ~ bit(ADEN); 
  bitSet(PRR, PRADC);                        // Disable the ADC to save power
  sleep.init(6);
  pulse = 0;

}

void loop() {

  sleep.sleep(30);

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

}

ISR (PCINT1_vect) 
{ 
  if (PINB & (1<<PB0)) { // detect rising edge 
    pulse++;
    _delay_ms(20); //simple debounce 
  } 
  else { // detect falling edge 
    _delay_ms(20); // simple debounce 
  } 
} 

// Wait a few milliseconds for proper ACK
#ifdef USE_ACK
static byte waitForAck() {
  MilliTimer ackTimer;
  while (!ackTimer.poll(ACK_TIME)) {
    if (radio.recvDone() && crc == 0 &&
      hdr == (RF12_HDR_DST | RF12_HDR_CTL | myNodeID))
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
    radio.sleep(-1);              // Wake up RF module
    while (!radio.canSend())
      radio.recvDone();
    radio.sendStart(RF12_HDR_ACK, &temptx, sizeof temptx); 
    radio.sendWait(2);           // Wait for RF to finish sending while in standby mode
    byte acked = waitForAck();  // Wait for ACK
    radio.sleep(0);              // Put RF module to sleep
    if (acked) { 
      return; 
    }      // Return if ACK received
    sleep(30);
  }
#else
  radio.sleep(-1);              // Wake up RF module
  while (!radio.canSend())
    radio.recvDone();
  radio.sendStart(0, &temptx, sizeof temptx); 
  radio.sendWait(2);           // Wait for RF to finish sending while in standby mode
  radio.sleep(0);              // Put RF module to sleep
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





