/**
 ******************************************************************************
 *
 * @file       FlightBatt.ino
 * @author     Joerg-D. Rothfuchs
 * @brief      Implements voltage and current measurement of the flight battery
 * 	       on the Ardupilot Mega MinimOSD using built-in ADC reference.
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


// !!! For using this, you have to solder a little bit on the MinimOSD, see the wiki !!!


#include "FlightBatt.h"


void flight_batt_init(void)
{
	analogReference(INTERNAL);			// INTERNAL: a built-in reference, equal to 1.1 volts on the ATmega168 or ATmega328
}


void flight_batt_read(void)
{
	static float voltage = LOW_VOLTAGE * 1.05;	// battery voltage, initialized above the low voltage threshold to pre-load the filter and prevent low voltage events at startup
	static float current_amps = 0;			// battery instantaneous currrent draw [A]
	static float current_total = 0;			// totalized battery current [mAh]
	static unsigned long loopTimer = 0;
	uint16_t delta_ms;

	if (loopTimer + MEASURE_PERIOD <= millis()) {
		delta_ms	= millis() - loopTimer;
		loopTimer      	= millis();
		
		voltage		= CURRENT_VOLTAGE(analogRead(VOLTAGE_PIN)) * .2 + voltage * .8;		// reads battery voltage pin
		osd_vbat_A	= voltage;
		if (curr_amp_per_volt > 0) {								// Consider Amp sensor disbled when Amp per Volt ratio is zero
			current_amps	= CURRENT_AMPS(analogRead(CURRENT_PIN)) * .2 + current_amps * .8; 	// reads battery sensor current pin
			current_total	+= current_amps * (float) delta_ms * 0.0002778;				// .0002778 is 1/3600 (conversion to hours)
			osd_curr_A	= current_amps * 100;
			osd_total_A	= current_total;
		}
	}
}
