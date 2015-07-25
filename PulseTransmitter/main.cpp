/*
// PulseTransmitter 
// http://opensource.org/licenses/BSD-3-Clause
// 2015 Martyn Brown : http://tinoest.no-ip.org

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

#ifndef F_CPU 
#define F_CPU 8000000L
#endif

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <util/delay.h>

#include "RFM12.h"
#include "WatchdogSleep.h"

//#define DEBUG
#define TMP_OFFSET      7

// Function defines 
long readVcc();
long int readTmpC();

#if defined(DEBUG)

#define BAUD            9600
#define STX_PORT        PORTA
#define STX_DDR         DDRA
#define STX_BIT         7 // Which port on PORTB to use 0 = D8 , 1 = D9 , 2 = D10

void sinit();
void sputs(const void *s);
void sputchar(uint8_t c);

#endif

volatile uint16_t pulse;

//ATTiny84
#define SS_BIT      1

// Enable the Watchdog Sleep
WatchdogSleep sleep;

// Pass the Slave Select Port Information
RFM12 radio(SS_BIT);

typedef struct {
	uint16_t pulse;
	int16_t tmpC;
	int16_t supplyV;	// Supply voltage
} 
Payload;

Payload temptx;

int main() 
{

#if defined(DEBUG)
	sinit();
	char tmp[10] = {
		'\0'
	};
	sprintf(tmp,"Starting\n");
	sputs(tmp);
#endif

	// PB0 
	GIMSK  |= (1<<PCIE1); // enable Pin Change Interrupt 1 
	PCMSK1  = (1<<PCINT8);// enable PCINT8

	radio.init(10,RF12_433MHZ,210); // Initialize RFM12 with settings defined above 
	radio.sleep(0);                 // Put the RFM12 to sleep
	PRR    = (1<<PRTIM1);           // bit(PRTIM1); //only keep timer 0 going
	ADCSRA &= ~(1<<ADEN);           // bit(ADEN); 
	PRR    |= (1<<PRADC);           // bitSet(PRR, PRADC); // Disable the ADC to save power
	sleep.init(6);
	pulse  = 0;
	sei(); // Enable interrupts

	for(;;) 
	{

		sleep.sleep(30);

		temptx.pulse		= pulse;       
		temptx.supplyV	= readVcc();  // Get supply voltage
		temptx.tmpC			= readTmpC(); // Get temperature reading and convert to integer, reversed at receiving end

#if defined(DEBUG)
		tmp[0] = '\0';
		sprintf(tmp,"Count %i Supply %i TmpC %i\n", pulse , temptx.supplyV , temptx.tmpC);
		sputs(tmp);
#endif

		radio.transmit(0, &temptx, sizeof(temptx)); // Send data via RF 

	}


	return 1;

}


ISR (PCINT1_vect) 
{ 
	if (PINB & (1<<PB0)) { // detect rising edge 
		pulse++;
		_delay_ms(20); //simple debounce, should really be on a timer and not a delay 
	} 
	else { // detect falling edge 
		_delay_ms(20); // simple debounce, should really be on a timer and not a delay 
	} 
} 

//--------------------------------------------------------------------------------------------------
// Read current supply voltage
//--------------------------------------------------------------------------------------------------
long readVcc() 
{
	PRR    &= ~(1<<PRADC); // bitClear(PRR, PRADC); 
	ADCSRA |= (1<<ADEN);   // Enable the ADC
	long result;
	// Read 1.1V reference against Vcc
	ADMUX = (1<<MUX5) | (1<<MUX0); // For ATtiny84
	_delay_ms(2); // Wait for Vref to settle
	ADCSRA |= (1<<ADSC); // Convert
	while (bit_is_set(ADCSRA,ADSC));
	result = ADCL;
	result |= ADCH<<8;
	result = 1126400L / result; // Back-calculate Vcc in mV
	ADCSRA &= ~(1<<ADEN); 
	PRR |= (1<<PRADC); // bitSet(PRR, PRADC); // Disable the ADC to save power
	return result;
} 


//--------------------------------------------------------------------------------------------------
// Read Internal Temperature , Return in Degree C
//--------------------------------------------------------------------------------------------------
long readTmpC() 
{
	PRR    &= ~(1<<PRADC); // bitClear(PRR, PRADC); 
	ADCSRA |= (1<<ADEN); // Enable the ADC
	long result;
	ADMUX = 0b00100010;                       // Select temperature sensor
	ADMUX &= ~(1<<ADLAR);                    // Right-adjust result
	ADMUX |= (1<< REFS1);                    // Set Ref voltage
	ADMUX &= ~(1<<REFS0);                    // to 1.1V
	// Configure ADCSRA
	ADCSRA &= ~( (1<<ADATE) | (1<<ADIE));    // Disable autotrigger, Disable Interrupt
	ADCSRA |= (1<<ADEN);                     // Enable ADC
	_delay_ms(2);
	ADCSRA |= (1<<ADSC); // Convert
	while (bit_is_set(ADCSRA,ADSC));
	result = ADCL;
	result |= ADCH<<8;
	ADCSRA &= ~ (1<<ADEN); 
	PRR |= (1<<PRADC); // bitSet(PRR, PRADC); // Disable the ADC to save power
	return result - 273 + TMP_OFFSET;
} 

//--------------------------------------------------------------------------------------------------
// Serial Functions Start
//--------------------------------------------------------------------------------------------------
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

void sputs(const void *s ) 
{
	uint8_t *s1 = (uint8_t*)s;
	while( *s1 )
		sputchar( *s1++ );
}

void sinit() {

	STX_PORT |= 1<<STX_BIT;
	STX_DDR |= 1<<STX_BIT; 

}

#endif
//--------------------------------------------------------------------------------------------------
// Serial Functions End
//--------------------------------------------------------------------------------------------------


