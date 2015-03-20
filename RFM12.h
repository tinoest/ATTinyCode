#ifndef RFM12_h
#define RFM12_h
#include <avr/sleep.h>
#include <stdlib.h>
#include <util/crc16.h>

#include "Arduino.h"

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

static uint8_t cs_pin = SS_BIT;     // chip select pin

enum {
  TXCRC1, TXCRC2, TXTAIL, TXDONE, TXIDLE,
  TXRECV,
  TXPRE1, TXPRE2, TXPRE3, TXSYN1, TXSYN2,
};

class RFM12
{
public:
  RFM12(void);
  uint8_t init(uint8_t id, uint8_t band, uint8_t g);
  void sleep (char n);
  uint8_t recvDone (void);
  void recvStart (void);
  void spiInit (void);
  static uint8_t transferByte(uint8_t out);
  static void xfer (uint16_t cmd);
  static uint16_t xferSlow (uint16_t cmd);
  uint8_t canSend ();
  void sendStart (uint8_t hdr);
  void sendStart (uint8_t hdr, const void* ptr, uint8_t len);
  void sendWait (uint8_t mode);
  uint16_t control(uint16_t cmd);
  static void interruptHandler(void);
private:
  //uint8_t _interruptPin;
  //volatile uint8_t buf[RF_MAX];     // recv/xmit buf, including hdr & crc bytes
  long seq;                           // seq number of encrypted packet (or -1)
  static uint8_t nodeid;              // address of this node
  static uint8_t group;               // network group
  static volatile uint8_t rxfill;     // number of data bytes in rf12_buf
  static volatile int8_t rxstate;     // current transceiver state
  //volatile uint16_t crc;            // running crc value
};

#endif


