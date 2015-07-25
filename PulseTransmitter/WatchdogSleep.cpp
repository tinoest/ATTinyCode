// WatchdogSleep for Attiny84
// http://opensource.org/licenses/BSD-3-Clause
// 2015 Martyn Brown : http://tinoest.no-ip.org

#include "WatchdogSleep.h"

volatile uint8_t _watchdogCounter;

WatchdogSleep::WatchdogSleep(void) 
{

	_watchdogCounter = 0;
}


//--------------------------------------------------------------------------------------------------
// Sleep Configuration 
//-------------------------------------------------------------------------------------------------
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms,6=1 sec,7=2 sec, 8=4 sec, 9=8 sec
void WatchdogSleep::init(uint8_t sleepMode)
{

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

void WatchdogSleep::sleep(uint8_t sleepTime)
{

  do {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);             // select the watchdog timer mode
    sleep_enable();                                  // enable the sleep mode ready for use
    sleep_mode();                                    // trigger the sleep
    sleep_disable();                                 // prevent further sleeps 
  } 
  while(_watchdogCounter < sleepTime);

  _watchdogCounter = 0;

}

ISR(WDT_vect) {
  _watchdogCounter++;
}


