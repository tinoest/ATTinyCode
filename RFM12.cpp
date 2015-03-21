// RFM12
// http://opensource.org/licenses/BSD-3-Clause
// 2014 Martyn Brown : http://tinoest.no-ip.org
// Based on the RFM12 driver from jeelabs.com (2009-02-09 <jc@wippler.nl>)

#include "RFM12.h"
#include "USPI.h"

USPI uspi;

volatile uint8_t RFM12::rxfill;       // number of data bytes in rf12_buf
volatile int8_t RFM12::rxstate;       // current transceiver state
uint8_t RFM12::nodeid;                // address of this node
uint8_t RFM12::group;                 // network group

uint8_t RFM12::_ssPin;

volatile uint8_t buf[RF_MAX];  // recv/xmit buf, including hdr & crc bytes
volatile uint16_t crc;         // running crc value

RFM12::RFM12(uint8_t ssPin ) {
  _ssPin   = ssPin;
}

uint8_t RFM12::init (uint8_t id, uint8_t band, uint8_t g) {
  nodeid = id;
  group = g;

  // Set the Slave Select Pin
  bitSet(SS_PORT, _ssPin);
  bitSet(SS_DDR, _ssPin);
  SS_PORT |= (1 << _ssPin); //digitalWrite(_ssPin, HIGH);
  SS_DDR  |= (1 << _ssPin); //pinMode(_ssPin, OUTPUT);

  // Initialise the uspi
  uspi.init();

  // Bring up the interrupt pin
  DDRB  &= ~(1<<RFM_IRQ); // pinMode(RFM_IRQ, INPUT);
  PORTB |= (1<<RFM_IRQ);  // digitalWrite(RFM_IRQ, HIGH); // pull-up

  xfer(0x0000); // intitial SPI transfer added to avoid power-up problem

  xfer(RF_SLEEP_MODE); // DC (disable clk pin), enable lbd

  // wait until RFM12B is out of power-up reset, this takes several *seconds*
  xfer(RF_TXREG_WRITE); // in case we're still in OOK mode
  //while (digitalRead(RFM_IRQ) == 0)
  while( PINB & ( 1<<RFM_IRQ) == 0)
    xfer(0x0000);

  xfer(0x80C7 | (band << 4)); // EL (ena TX), EF (ena RX FIFO), 12.0pF 
  xfer(0xA640); // 868MHz 
  xfer(0xC606); // approx 49.2 Kbps, i.e. 10000/29/(1+6) Kbps
  xfer(0x94A2); // VDI,FAST,134kHz,0dBm,-91dBm 
  xfer(0xC2AC); // AL,!ml,DIG,DQD4 
  if (group != 0) {
    xfer(0xCA83); // FIFO8,2-SYNC,!ff,DR 
    xfer(0xCE00 | group); // SYNC=2DXX； 
  } 
  else {
    xfer(0xCA8B); // FIFO8,1-SYNC,!ff,DR 
    xfer(0xCE2D); // SYNC=2D； 
  }
  xfer(0xC483); // @PWR,NO RSTRIC,!st,!fi,OE,EN 
  xfer(0x9850); // !mp,90kHz,MAX OUT 
  xfer(0xCC77); // OB1，OB0, LPX,！ddy，DDIT，BW0 
  xfer(0xE000); // NOT USE 
  xfer(0xC800); // NOT USE 
  xfer(0xC049); // 1.66MHz,3.1V 

  rxstate = TXIDLE;

  if ((nodeid & NODE_ID) != 0) {
    MCUCR = (MCUCR & ~((1 << ISC00) | (1 << ISC01))) | (0 << ISC00);
    GIMSK |= (1 << INT0); //attachInterrupt(0, RFM12::interruptHandler, LOW);
  } else {
    GIMSK &= ~(1 << INT0); //detachInterrupt(0);
  }

  return nodeid;
}

void RFM12::sleep (char n) {
  if (n < 0) {
    control(RF_IDLE_MODE);
  } 
  else {
    control(RF_WAKEUP_TIMER | 0x0500 | n);
    control(RF_SLEEP_MODE);
    if (n > 0)
      control(RF_WAKEUP_MODE);
  }
  rxstate = TXIDLE;
}

uint8_t RFM12::recvDone (void) {
  if (rxstate == TXRECV && (rxfill >= rf12_len + 5 || rxfill >= RF_MAX)) {
    rxstate = TXIDLE;
    if (rf12_len > RF12_MAXDATA)
      rf12_len = 1; // force bad crc if packet length is invalid
    if (!(rf12_hdr & RF12_HDR_DST) || (nodeid & NODE_ID) == 31 || (rf12_hdr & RF12_HDR_MASK) == (nodeid & NODE_ID)) {
      seq = -1;
      return 1; // it's a broadcast packet or it's addressed to this node
    }
  }
  if (rxstate == TXIDLE)
    recvStart();
  return 0;
}



void RFM12::recvStart (void) {
  rxfill = rf12_len = 0;
  crc = ~0;

  if (group != 0) {
    crc = _crc16_update(~0, group);
  }

  rxstate = TXRECV;    
  xfer(RF_RECEIVER_ON);
}

uint8_t RFM12::transferByte (uint8_t outputData) {

  return uspi.transfer(outputData);

}

void RFM12::xfer (uint16_t cmd) {
  // writing can take place at full speed, even 8 MHz works
  bitClear(SS_PORT, _ssPin);
  transferByte(cmd >> 8) << 8;
  transferByte(cmd);
  bitSet(SS_PORT, _ssPin);
}

uint8_t RFM12::canSend () {
  if (rxstate == TXRECV && rxfill == 0 && (control(0x0000) & RF_RSSI_BIT) == 0) {
    control(RF_IDLE_MODE); // stop receiver
    rxstate = TXIDLE;
    return 1;
  }
  return 0;
}

void RFM12::sendStart (uint8_t hdr) {
  rf12_hdr = hdr & RF12_HDR_DST ? hdr : (hdr & ~RF12_HDR_MASK) + (nodeid & NODE_ID);

  crc = ~0;

  crc = _crc16_update(crc, group);

  rxstate = TXPRE1;
  xfer(RF_XMITTER_ON); // bytes will be fed via interrupts
}

void RFM12::sendStart (uint8_t hdr, const void* ptr, uint8_t len) 
{
  rf12_len = len;
  memcpy((void*) rf12_data, ptr, len);
  sendStart(hdr);
}

void RFM12::sendWait (uint8_t mode) {
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

uint16_t RFM12::control(uint16_t cmd) {

  // ATtiny
  bitClear(GIMSK, INT0);
  uint16_t r = xferSlow(cmd);
  bitSet(GIMSK, INT0);

  return r;

}

void RFM12::interruptHandler() {
  // a transfer of 2x 16 bits @ 2 MHz over SPI takes 2x 8 us inside this ISR
  // correction: now takes 2 + 8 µs, since sending can be done at 8 MHz
  xfer(0x0000);

  if (rxstate == TXRECV) {
    uint8_t in = xferSlow(RF_RX_FIFO_READ);

    if (rxfill == 0 && group != 0)
      buf[rxfill++] = group;

    buf[rxfill++] = in;
    crc = _crc16_update(crc, in);

    if (rxfill >= rf12_len + 5 || rxfill >= RF_MAX)
      xfer(RF_IDLE_MODE);
  } 
  else {
    uint8_t out;

    if (rxstate < 0) {
      uint8_t pos = 3 + rf12_len + rxstate++;
      out = buf[pos];
      crc = _crc16_update(crc, out);
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
        out = crc; 
        break;
      case TXCRC2: 
        out = crc >> 8; 
        break;
      case TXDONE: 
        xfer(RF_IDLE_MODE); // fall through
      default:     
        out = 0xAA;
      }

    xfer(RF_TXREG_WRITE + out);
  }
}

uint16_t RFM12::xferSlow (uint16_t cmd) {

  bitClear(SS_PORT, _ssPin);
  uint16_t reply = transferByte(cmd >> 8) << 8;
  reply |= transferByte(cmd);
  bitSet(SS_PORT, _ssPin);

  return reply;
}


ISR (EXT_INT0_vect) 
{ 
  RFM12::interruptHandler();
} 





