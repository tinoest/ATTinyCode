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
#include <util/crc16.h>

//ATTiny84
#define RFM_IRQ     2
#define SS_DDR      DDRB
#define SS_PORT     PORTB
#define SS_BIT      1

#define SPI_SS      1     // PB1, pin 3
#define SPI_MISO    4     // PA6, pin 7
#define SPI_MOSI    5     // PA5, pin 8
#define SPI_SCK     6     // PA4, pin 9

#define RF12_433MHZ     1   ///< RFM12B 433 MHz frequency band.
#define RF12_868MHZ     2   ///< RFM12B 868 MHz frequency band.
#define RF12_915MHZ     3   ///< RFM12B 915 MHz frequency band.

// RF12 command codes
#define RF_RECEIVER_ON  0x82DD
#define RF_XMITTER_ON   0x823D
#define RF_IDLE_MODE    0x820D
#define RF_SLEEP_MODE   0x8205
#define RF_WAKEUP_MODE  0x8207
#define RF_TXREG_WRITE  0xB800
#define RF_RX_FIFO_READ 0xB000
#define RF_WAKEUP_TIMER 0xE000

// RF12 status bits
#define RF_LBD_BIT      0x0400
#define RF_RSSI_BIT     0x0100

// bits in the node id configuration byte
#define NODE_BAND       0xC0        // frequency band
#define NODE_ACKANY     0x20        // ack on broadcast packets if set
#define NODE_ID         0x1F        // id of this node, as A..Z or 1..31

#define myNodeID 10     // RF12 node ID in the range 1-30
#define network 210      // RF12 Network group
#define freq RF12_433MHZ // Frequency of RFM12B module

//#define USE_ACK           // Enable ACKs, comment out to disable
#define RETRY_PERIOD 5    // How soon to retry (in seconds) if ACK didn't come in
#define RETRY_LIMIT 5     // Maximum number of times to retry
#define ACK_TIME 10       // Number of milliseconds to wait for an ack

#define BAUD            9600
#define STX_PORT        PORTA
#define STX_DDR         DDRA
#define STX_BIT         7 // Which port on PORTB to use 0 = D8 , 1 = D9 , 2 = D10

#define TMP_OFFSET      7



static uint8_t nodeid;              // address of this node
static uint8_t group;               // network group
static volatile uint8_t rxfill;     // number of data bytes in rf12_buf
static volatile int8_t rxstate;     // current transceiver state
volatile uint16_t rf12_crc;         // running crc value
volatile uint16_t pulse;

enum {
  TXCRC1, TXCRC2, TXTAIL, TXDONE, TXIDLE,
  TXRECV,
  TXPRE1, TXPRE2, TXPRE3, TXSYN1, TXSYN2,
};

/// Shorthand for RFM12B group byte in rf12_buf.
#define rf12_grp        rf12_buf[0]
/// Shorthand for RFM12B header byte in rf12_buf.
#define rf12_hdr        rf12_buf[1]
/// Shorthand for RFM12B length byte in rf12_buf.
#define rf12_len        rf12_buf[2]
/// Shorthand for first RFM12B data byte in rf12_buf.
#define rf12_data       (rf12_buf + 3)

/// RFM12B CTL bit mask.
#define RF12_HDR_CTL    0x80
/// RFM12B DST bit mask.
#define RF12_HDR_DST    0x40
/// RFM12B ACK bit mask.
#define RF12_HDR_ACK    0x20
/// RFM12B HDR bit mask.
#define RF12_HDR_MASK   0x1F

/// RFM12B Maximum message size in bytes.
#define RF12_MAXDATA    66
// maximum transmit / receive buffer: 3 header + data + 2 crc bytes
#define RF_MAX   (RF12_MAXDATA + 5)
volatile uint8_t rf12_buf[RF_MAX];  // recv/xmit buf, including hdr & crc bytes
long rf12_seq;                      // seq number of encrypted packet (or -1)

static uint8_t cs_pin = SS_BIT;     // chip select pin

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

  rf12_initialize(myNodeID,freq,network);  // Initialize RFM12 with settings defined above 
  rf12_sleep(0);                           // Put the RFM12 to sleep
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
    sleep(30);
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

uint8_t rf12_initialize (uint8_t id, uint8_t band, uint8_t g) {
  nodeid = id;
  group = g;

  rf12_spiInit();

  rf12_xfer(0x0000); // intitial SPI transfer added to avoid power-up problem

  rf12_xfer(RF_SLEEP_MODE); // DC (disable clk pin), enable lbd

  // wait until RFM12B is out of power-up reset, this takes several *seconds*
  rf12_xfer(RF_TXREG_WRITE); // in case we're still in OOK mode
  while (digitalRead(RFM_IRQ) == 0)
    rf12_xfer(0x0000);

  rf12_xfer(0x80C7 | (band << 4)); // EL (ena TX), EF (ena RX FIFO), 12.0pF 
  rf12_xfer(0xA640); // 868MHz 
  rf12_xfer(0xC606); // approx 49.2 Kbps, i.e. 10000/29/(1+6) Kbps
  rf12_xfer(0x94A2); // VDI,FAST,134kHz,0dBm,-91dBm 
  rf12_xfer(0xC2AC); // AL,!ml,DIG,DQD4 
  if (group != 0) {
    rf12_xfer(0xCA83); // FIFO8,2-SYNC,!ff,DR 
    rf12_xfer(0xCE00 | group); // SYNC=2DXX； 
  } 
  else {
    rf12_xfer(0xCA8B); // FIFO8,1-SYNC,!ff,DR 
    rf12_xfer(0xCE2D); // SYNC=2D； 
  }
  rf12_xfer(0xC483); // @PWR,NO RSTRIC,!st,!fi,OE,EN 
  rf12_xfer(0x9850); // !mp,90kHz,MAX OUT 
  rf12_xfer(0xCC77); // OB1，OB0, LPX,！ddy，DDIT，BW0 
  rf12_xfer(0xE000); // NOT USE 
  rf12_xfer(0xC800); // NOT USE 
  rf12_xfer(0xC049); // 1.66MHz,3.1V 

  rxstate = TXIDLE;
#if PINCHG_IRQ
#if RFM_IRQ < 8
  if ((nodeid & NODE_ID) != 0) {
    bitClear(DDRD, RFM_IRQ);      // input
    bitSet(PORTD, RFM_IRQ);       // pull-up
    bitSet(PCMSK2, RFM_IRQ);      // pin-change
    bitSet(PCICR, PCIE2);         // enable
  } 
  else
    bitClear(PCMSK2, RFM_IRQ);
#elif RFM_IRQ < 14
  if ((nodeid & NODE_ID) != 0) {
    bitClear(DDRB, RFM_IRQ - 8);  // input
    bitSet(PORTB, RFM_IRQ - 8);   // pull-up
    bitSet(PCMSK0, RFM_IRQ - 8);  // pin-change
    bitSet(PCICR, PCIE0);         // enable
  } 
  else
    bitClear(PCMSK0, RFM_IRQ - 8);
#else
  if ((nodeid & NODE_ID) != 0) {
    bitClear(DDRC, RFM_IRQ - 14); // input
    bitSet(PORTC, RFM_IRQ - 14);  // pull-up
    bitSet(PCMSK1, RFM_IRQ - 14); // pin-change
    bitSet(PCICR, PCIE1);         // enable
  } 
  else
    bitClear(PCMSK1, RFM_IRQ - 14);
#endif
#else
  if ((nodeid & NODE_ID) != 0)
    attachInterrupt(0, rf12_interrupt, LOW);
  else
    detachInterrupt(0);
#endif

  return nodeid;
}

void rf12_sleep (char n) {
  if (n < 0) {
    rf12_control(RF_IDLE_MODE);
  } 
  else {
    rf12_control(RF_WAKEUP_TIMER | 0x0500 | n);
    rf12_control(RF_SLEEP_MODE);
    if (n > 0)
      rf12_control(RF_WAKEUP_MODE);
  }
  rxstate = TXIDLE;
}

uint8_t rf12_recvDone () {
  if (rxstate == TXRECV && (rxfill >= rf12_len + 5 || rxfill >= RF_MAX)) {
    rxstate = TXIDLE;
    if (rf12_len > RF12_MAXDATA)
      rf12_crc = 1; // force bad crc if packet length is invalid
    if (!(rf12_hdr & RF12_HDR_DST) || (nodeid & NODE_ID) == 31 || (rf12_hdr & RF12_HDR_MASK) == (nodeid & NODE_ID)) {
      rf12_seq = -1;
      return 1; // it's a broadcast packet or it's addressed to this node
    }
  }
  if (rxstate == TXIDLE)
    rf12_recvStart();
  return 0;
}

static void rf12_recvStart () {
  rxfill = rf12_len = 0;
  rf12_crc = ~0;

  if (group != 0) {
    rf12_crc = _crc16_update(~0, group);
  }

  rxstate = TXRECV;    
  rf12_xfer(RF_RECEIVER_ON);
}

void rf12_spiInit () {

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

static uint8_t rf12_byte (uint8_t out) {

  // ATtiny
  USIDR = out;
  byte v1 = bit(USIWM0) | bit(USITC);
  byte v2 = bit(USIWM0) | bit(USITC) | bit(USICLK);
#if F_CPU <= 5000000
  // only unroll if resulting clock stays under 2.5 MHz
  USICR = v1; 
  USICR = v2;
  USICR = v1; 
  USICR = v2;
  USICR = v1; 
  USICR = v2;
  USICR = v1; 
  USICR = v2;
  USICR = v1; 
  USICR = v2;
  USICR = v1; 
  USICR = v2;
  USICR = v1; 
  USICR = v2;
  USICR = v1; 
  USICR = v2;
#else
  for (uint8_t i = 0; i < 8; ++i) {
    USICR = v1;
    USICR = v2;
  }
#endif
  return USIDR;

}

static void rf12_xfer (uint16_t cmd) {
  // writing can take place at full speed, even 8 MHz works
  bitClear(SS_PORT, cs_pin);
  rf12_byte(cmd >> 8) << 8;
  rf12_byte(cmd);
  bitSet(SS_PORT, cs_pin);
}

uint8_t rf12_canSend () {
  if (rxstate == TXRECV && rxfill == 0 && (rf12_control(0x0000) & RF_RSSI_BIT) == 0) {
    rf12_control(RF_IDLE_MODE); // stop receiver
    rxstate = TXIDLE;
    return 1;
  }
  return 0;
}

void rf12_sendStart (uint8_t hdr) {
  rf12_hdr = hdr & RF12_HDR_DST ? hdr : (hdr & ~RF12_HDR_MASK) + (nodeid & NODE_ID);

  rf12_crc = ~0;

  rf12_crc = _crc16_update(rf12_crc, group);

  rxstate = TXPRE1;
  rf12_xfer(RF_XMITTER_ON); // bytes will be fed via interrupts
}

void rf12_sendStart (uint8_t hdr, const void* ptr, uint8_t len) {
  rf12_len = len;
  memcpy((void*) rf12_data, ptr, len);
  rf12_sendStart(hdr);
}

/// @deprecated Use the 3-arg version, followed by a call to rf12_sendWait.
void rf12_sendStart (uint8_t hdr, const void* ptr, uint8_t len, uint8_t sync) {
  rf12_sendStart(hdr, ptr, len);
  rf12_sendWait(sync);
}

void rf12_sendWait (uint8_t mode) {
  // wait for packet to actually finish sending
  // go into low power mode, as interrupts are going to come in very soon
  while (rxstate != TXIDLE)
    if (mode) {
      // power down mode is only possible if the fuses are set to start
      // up in 258 clock cycles, i.e. approx 4 us - else must use standby!
      // modes 2 and higher may lose a few clock timer ticks
      set_sleep_mode(mode == 3 ? SLEEP_MODE_PWR_DOWN :
#ifdef SLEEP_MODE_STANDBY
      mode == 2 ? SLEEP_MODE_STANDBY :
#endif
      SLEEP_MODE_IDLE);
      sleep_mode();
    }
}

uint16_t rf12_control(uint16_t cmd) {

  // ATtiny
  bitClear(GIMSK, INT0);
  uint16_t r = rf12_xferSlow(cmd);
  bitSet(GIMSK, INT0);

  return r;

}

static void rf12_interrupt() {
  // a transfer of 2x 16 bits @ 2 MHz over SPI takes 2x 8 us inside this ISR
  // correction: now takes 2 + 8 µs, since sending can be done at 8 MHz
  rf12_xfer(0x0000);

  if (rxstate == TXRECV) {
    uint8_t in = rf12_xferSlow(RF_RX_FIFO_READ);

    if (rxfill == 0 && group != 0)
      rf12_buf[rxfill++] = group;

    rf12_buf[rxfill++] = in;
    rf12_crc = _crc16_update(rf12_crc, in);

    if (rxfill >= rf12_len + 5 || rxfill >= RF_MAX)
      rf12_xfer(RF_IDLE_MODE);
  } 
  else {
    uint8_t out;

    if (rxstate < 0) {
      uint8_t pos = 3 + rf12_len + rxstate++;
      out = rf12_buf[pos];
      rf12_crc = _crc16_update(rf12_crc, out);
    } 
    else
      switch (rxstate++) {
      case TXSYN1: 
        out = 0x2D; 
        break;
      case TXSYN2: 
        out = group; 
        rxstate = - (2 + rf12_len); 
        break;
      case TXCRC1: 
        out = rf12_crc; 
        break;
      case TXCRC2: 
        out = rf12_crc >> 8; 
        break;
      case TXDONE: 
        rf12_xfer(RF_IDLE_MODE); // fall through
      default:     
        out = 0xAA;
      }

    rf12_xfer(RF_TXREG_WRITE + out);
  }
}

static uint16_t rf12_xferSlow (uint16_t cmd) {
  // slow down to under 2.5 MHz
#if F_CPU > 10000000
  bitSet(SPCR, SPR0);
#endif
  bitClear(SS_PORT, cs_pin);
  uint16_t reply = rf12_byte(cmd >> 8) << 8;
  reply |= rf12_byte(cmd);
  bitSet(SS_PORT, cs_pin);
#if F_CPU > 10000000
  bitClear(SPCR, SPR0);
#endif
  return reply;
}





