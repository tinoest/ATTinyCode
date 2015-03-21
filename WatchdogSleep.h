#ifndef WATCHDOGSLEEP_h
#define WATCHDOGSLEEP_h

#include <stdlib.h>
#include "Arduino.h"
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

/*
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

class WatchdogSleep
{
public:
  WatchdogSleep(void);
  void init(uint8_t sleepMode);
  void sleep(uint8_t sleepTime);
private:
};

#endif
