// WatchdogSleep for Attiny84
// http://opensource.org/licenses/BSD-3-Clause
// 2015 Martyn Brown : http://tinoest.no-ip.org

#include "WatchdogSleep.h"

volatile bool _watchdogOverrunFlag;
volatile uint8_t _watchdogCounter;

WatchdogSleep::WatchdogSleep(void) 
{

	// Initialise the variables
	_watchdogOverrunFlag	= 0;
	_watchdogCounter			= 0;

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
	uint8_t oldSREG = SREG; // Store SREG
	sei(); // turn interrupts on

  do {
		_watchdogOverrunFlag	= 0;
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);             // select the watchdog timer mode
    sleep_enable();                                  // enable the sleep mode ready for use
    sleep_mode();                                    // trigger the sleep
    sleep_disable();                                 // prevent further sleeps 
  } 
  while(_watchdogCounter < sleepTime);

	SREG = oldSREG; // restore SREG
  _watchdogCounter			= 0;
	_watchdogOverrunFlag	= 0;

}

ISR(WDT_vect) {
	
	if(_watchdogOverrunFlag  == 0) { 
		_watchdogCounter++;
		_watchdogOverrunFlag = 1;
	} 
	else {
		// Watchdog overrun reset MCU
	}
}


