// RFM12 for Attiny84
// http://opensource.org/licenses/BSD-3-Clause
// 2014 Martyn Brown : http://tinoest.no-ip.org
// Based on the RFM12 driver from jeelabs.com (2009-02-09 <jc@wippler.nl>)

#ifndef RFM12_h
#define RFM12_h
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <util/crc16.h>
#include <string.h> 
#include "USPI.h"

// RFM Interrupt port
#define RFM_IRQ     2

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


/// Shorthand for RFM12B group byte in rf12_buf.
#define rf12_grp        buf[0]
/// Shorthand for RFM12B header byte in rf12_buf.
#define rf12_hdr        buf[1]
/// Shorthand for RFM12B length byte in rf12_buf.
#define rf12_len        buf[2]
/// Shorthand for first RFM12B data byte in rf12_buf.
#define rf12_data       (buf + 3)

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

enum {
  TXCRC1, TXCRC2, TXTAIL, TXDONE, TXIDLE,
  TXRECV,
  TXPRE1, TXPRE2, TXPRE3, TXSYN1, TXSYN2,
};

// Slave Select Ports
#define SS_DDR      DDRB
#define SS_PORT     PORTB

class RFM12
{
public:
  RFM12(uint8_t csPin);
  uint8_t init(uint8_t id, uint8_t band, uint8_t group);
  void sleep (char n);
  void transmit(uint8_t hdr, const void* packetBuffer, uint8_t packetLen);
  static void interruptHandler(void);
private:
  void sendStart (uint8_t hdr);
  void sendStart (uint8_t hdr, const void* packetBuffer, uint8_t packetLen);
  void sendWait (uint8_t mode);
  uint8_t canSend ();
  uint8_t recvDone (void);
  void recvStart (void);
  static uint8_t transferByte(uint8_t outputData);
  static void xfer (uint16_t cmd);
  static uint16_t xferSlow (uint16_t cmd);
  uint16_t control(uint16_t cmd);
  long seq;                           // seq number of encrypted packet (or -1)
  static uint8_t _node;               // address of this node
  static uint8_t _group;              // network group
  static volatile uint8_t rxfill;     // number of data bytes in rf12_buf
  static volatile int8_t rxstate;     // current transceiver state
  static uint8_t _ssPin;
};

#endif





