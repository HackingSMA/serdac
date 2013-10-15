/*
The MIT License (MIT)

Copyright (c) 2013 Shea Ako

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <avr/io.h>
#include <util/delay.h> 
#include <string.h>

#ifndef F_CPU
	#error F_CPU not defined // eg. #define F_CPU=16000000UL	
#endif
#define BAUD_TOL 1 // allow 1% tolerance
#define BAUD 115200
#include <util/setbaud.h>

#define TIMER1_PRELOAD (65536-(F_CPU/(1024*10))) // 100ms

/*
 Packet format:
 0xaa
 0xaa
 length of payload
 payload bytes...
 checksum (sum of payload bytes %256)
 */
typedef struct {
	uint8_t buffer[255];
	void (*payload_callback)(void *data, uint8_t len, void *ref);
	void *ref;
	uint8_t state;
	uint8_t len;
	uint8_t idx;
	uint8_t chk;
} packet_parser_t;

static inline void pp_parse_byte(packet_parser_t *p, uint8_t byte) {
	if(p->state==0) {
		if(byte==0xaa) p->state=1;
	} else
		
		if(p->state==1) {
			if(byte==0xaa) p->state=2;
		} else
			
			if(p->state==2) {
				if(byte>0) {
					p->len=byte;
					p->idx=0;
					p->chk=0;
					p->state=3;
				} else {
					// len must be > 0
					p->state=0;
				}
			} else
				
				if(p->state==3) {
					p->buffer[p->idx++]=byte;
					p->chk+=byte;
					if(p->idx==p->len) {
						p->state=4;
					}
				} else
					
					if(p->state==4) {
						if(p->chk==byte) {
							p->payload_callback(p->buffer, p->len, p->ref);
						} else {
							// bad chk
						}
						p->state=0;
					}
}

#define CS_LO PORTB&=~_BV(2);
#define CS_HI PORTB|=_BV(2);
#define LDAC_LO PORTB&=~_BV(1);
#define LDAC_HI PORTB|=_BV(1);

void
update_dac(int16_t x, int16_t y) {
	// convert signed/bipolar to unsigned/unipolar
	x-=0x8000;
	y-=0x8000;
	
	// send 12 msb of x/y to dac via SPI		
	
	// ch. A/X
	CS_LO
	SPDR=0x30 | ((x>>12) & 0x0f);
	loop_until_bit_is_set(SPSR, SPIF); // wait for SPI transmission to complete
	SPDR=(x>>4)&0xff;
	loop_until_bit_is_set(SPSR, SPIF); // wait for SPI transmission to complete
	CS_HI
	
	_delay_us(1);
	
	// ch. B/Y
	CS_LO
	SPDR=0xb0 | ((y>>12) & 0x0f);
	loop_until_bit_is_set(SPSR, SPIF); // wait for SPI transmission to complete
	SPDR=(y>>4)&0xff;
	loop_until_bit_is_set(SPSR, SPIF); // wait for SPI transmission to complete
	CS_HI
	
	// latch outputs
	_delay_us(1);
	LDAC_LO
	_delay_us(1);
	LDAC_HI
}


void
payload_callback(void *data, uint8_t len, void *ref) {
	static int16_t x, y;
	static uint8_t *bytes;
	
	if(len==4) {
		// payload contains 16-bit signed X and Y values, ie.
		// XMSB, XLSB, YMSB, YLSB
		bytes=(uint8_t*)data;
		x=(bytes[0]<<8) | bytes[1];
		y=(bytes[2]<<8) | bytes[3];
		
		update_dac(x, y);
		TCNT1=TIMER1_PRELOAD;	// reset timer1
	}
}


int
main (void) {
	static uint8_t byte, status;

	// init packet_parser
	static packet_parser_t pp;
	memset(&pp, 0, sizeof(pp));
	pp.payload_callback=payload_callback;
	pp.ref=NULL;

	// set baud uart rate from macros
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
	#if USE_2X
		UCSR0A |= _BV(U2X0);
	#else
		UCSR0A &= ~_BV(U2X0);
	#endif
	
	// PB1 output (LDAC)
	DDRB|=_BV(1);
	LDAC_HI
	
	// setup SPI, master mode, fck/4
	DDRB|= _BV(5); // PB5 output (SCK)
	DDRB&=~_BV(4); // PB4 input  (MISO)
	DDRB|= _BV(3); // PB3 output (MOSI)
	DDRB|= _BV(2); // PB2 output (SS)
	PORTB|=_BV(2); // set SS hi
	_delay_us(1);
	SPCR =	 _BV(SPE) 	// SPI enable
			|_BV(MSTR)	// master mode
			|_BV(CPOL)	// setup on falling edge
			|_BV(CPHA);	// latch on rising edge
	
	// init timer1
	TCCR1B = _BV(CS12) | _BV(CS10); 	// running at F_CPU/1024


	// in the case of a loss of communication or controlling system crash the wheelchair
	// is stopped.
	//
	// timer1 counts from TIMER1_PRELOAD to 0xffff which takes 100ms
	// when timer1 overflows (sets TOV1 flag) the center position (0,0) is sent to the dac
	// outputs and the sysem pauses for 2 seconds.
	// when a new valid data packet is received the dac outputs are updated and timer1 is
	// reset. under normal conditions data packets are received faster than every 100ms
	// and timer1 never overflows.
	
	// main loop
	for(;;) {

		// reset/timeout		
		update_dac(0, 0);		// center dac outputs
		UCSR0B &= ~_BV(RXEN0);	// disable receiver
		_delay_ms(2000);		// wait a couple seconds
		UCSR0B |= _BV(RXEN0);	// enable receiver
		TCNT1=TIMER1_PRELOAD;	// preload timer1
		TIFR1|=_BV(TOV1);		// clear timer1 overflow flag		
		
		
		// handle incoming bytes until timer1 overflows
		// each time a valid packet is received the timer is reset
		while( (TIFR1 & _BV(TOV1)) == 0 ) {
			status=UCSR0A;
			if( status & (_BV(FE0) | _BV(DOR0) | _BV(UPE0) | _BV(RXC0)) ) {
				byte=UDR0; // read receive byte and clears any errors
				if( status & (_BV(FE0) | _BV(DOR0) | _BV(UPE0)) ) {
					// a receive error occurred
					pp.state=0;
				} else {
					pp_parse_byte(&pp, byte);
				}
			}
		}
		
		// timer1 overflowed, reset
	}

	return 0;
}