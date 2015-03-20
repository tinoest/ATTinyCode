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
 
 Watchdog Timer Prescale Select
 
 WDP3   WDP2 WDP1 WDP0   Number of WDT     Typical Time-out at Oscillator Cycles     VCC = 5.0V
 
 0    0    0      0             2K (2048) cycles       16 ms
 0    0    0      1             4K (4096) cycles       32 ms
 0    0    1      0             8K (8192) cycles       64 ms
 0    0    1      1            16K (16384) cycles      0.125 s
 0    1    0      0            32K (32768) cycles      0.25 s
 0    1    0      1            64K (65536) cycles      0.5 s
 0    1    1      0            128K (131072) cycles    1.0 s
 0    1    1      1            256K (262144) cycles    2.0 s
 1    0    0      0            512K (524288) cycles    4.0 s
 1    0    0      1            1024K (1048576) cycles  8.0 s
 
 */
// digitalWrite Low 
#define CLR(x,y) (x&=(~(1<<y)))
// digitalWrite High
#define SET(x,y) (x|=(1<<y))

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <stdlib.h>
#include <util/delay.h>
#include "RFM12.h"


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

RFM12 radio;

//#define DEBUG

int counter;
volatile uint8_t watchdogCounter;

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
  GIMSK  = (1<<PCIE1);            // enable Pin Change Interrupt 1 
  PCMSK1 = (1<<PCINT8);           // enable PCINT8

  radio.init(10,RF12_433MHZ,210);  // Initialize RFM12 with settings defined above 
  radio.sleep(0);                           // Put the RFM12 to sleep
  PRR = bit(PRTIM1);                       // only keep timer 0 going
  ADCSRA &= ~ bit(ADEN); 
  bitSet(PRR, PRADC);                      // Disable the ADC to save power
  setup_watchdog(6);
  pulse = 0;

}

void loop() {

  sleep(30);

  counter++;
  temptx.counter = (counter); // Get temperature reading and convert to integer, reversed at receiving end
  temptx.supplyV = readVcc(); // Get supply voltage
  temptx.tmpC    = pulse;

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


//--------------------------------------------------------------------------------------------------
// Sleep Configuration 
//-------------------------------------------------------------------------------------------------
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms,6=1 sec,7=2 sec, 8=4 sec, 9=8 sec
void setup_watchdog(uint8_t sleepMode) {

  byte sleepByte;
  if (sleepMode > 9 ) sleepMode = 9;
  sleepByte = sleepMode & 0x07;
  if (sleepMode > 7) sleepByte |= (1<<5);
  sleepByte |= (1<<WDCE);

  MCUSR &= ~(1 << WDRF);                           // reset status flag
  WDTCSR |= (1 << WDCE) | (1 << WDE);              // enable configuration changes
  WDTCSR = sleepByte;
  WDTCSR |= (1 << WDIE);                           // enable interrupt mode

}

void sleep(uint8_t sleepTime) {

  do {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);             // select the watchdog timer mode
    sleep_enable();                                  // enable the sleep mode ready for use
    sleep_mode();                                    // trigger the sleep
    sleep_disable();                                 // prevent further sleeps 
  } 
  while(watchdogCounter < sleepTime);

  watchdogCounter = 0;

}

ISR(WDT_vect) {
  watchdogCounter++;
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





