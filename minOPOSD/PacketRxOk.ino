/**
 ******************************************************************************
 *
 * @file       PacketRxOk.ino
 * @author     Joerg-D. Rothfuchs
 * @brief      Implements packet receive ok detection and measurement
 * 	       on the Ardupilot Mega MinimOSD using INT1.
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/> or write to the 
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */


// !!! For using this, you have to solder a little bit, see the wiki !!!

// avr-libc library includes
#include <avr/io.h>
#include <avr/interrupt.h>

#include "PacketRxOk.h"


volatile int pin_value;
volatile int packet_ok = 0;
volatile int packet_nok = 0;

static float packet_rxok_percent = 0.0;


void setup_timer_int() {
	TCCR1A	= 0;			// initialize TCCR1A
	TCCR1B	= 0;			// initialize TCCR1B
	OCR1A	= 249;			// set compare match register to 1ms timer (@16MHz, 64 prescaler)
	TCCR1B |= (1 << WGM12);		// turn on CTC mode
	TCCR1B |= (1 << CS10);		// set CS10 bit for 64 prescaler
	TCCR1B |= (1 << CS11);		// set CS11 bit for 64 prescaler
	TIMSK1 |= (1 << OCIE1A);	// enable timer compare interrupt
}


ISR (TIMER1_COMPA_vect) {
	if (pin_value) {
		packet_ok++;
	} else {
		packet_nok++;
	}
}


void pin_int(void) {
	pin_value = digitalRead(PRO_PIN);
}


void PacketRxOk_init(void) {
	cli();						// disable global interrupts
	attachInterrupt(PRO_INT, pin_int, CHANGE);
	setup_timer_int();
	pin_int();
	sei();						// enable global interrupts
}


void PacketRxOk_read(void) {
	packet_rxok_percent = 10.0 / (packet_ok + packet_nok) * packet_ok + packet_rxok_percent * .9;
	packet_ok = 0;
	packet_nok = 0;
	osd_rssi = ((uint8_t) packet_rxok_percent) >= 99 ? 100 : (uint8_t) packet_rxok_percent;
}
