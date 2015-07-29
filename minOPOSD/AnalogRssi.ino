/**
 ******************************************************************************
 *
 * @file       AnalogRssi.ino
 * @author     Philippe Vanhaesnedonck
 * @brief      Implements RSSI report on the Ardupilot Mega MinimOSD
 * 	       using built-in ADC reference.
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


#include "AnalogRssi.h"


void analog_rssi_init(void)
{
	analogReference(INTERNAL);			// INTERNAL: a built-in reference, equal to 1.1 volts on the ATmega168 or ATmega328
}


void analog_rssi_read(void)
{
	if (rssiraw_on) {
		//osd_rssi = analogRead(RSSI_PIN) / 4;          // Just raw value, 0-255. We use this range to better align
		//osd_rssi = (uint8_t)(float)((osd_chan7_raw - 992) * .31135f); 

    // This only works for Frsky X8R Rx
    // Currently this receiver has a RSSI port which outputs PWM signal of 1kHz whose duty cycle (%) is equivalent to RSSI (%)
    uint16_t rssipulselength;
    rssipulselength = (uint16_t)FastpulseIn(PWMRSSIPIN, HIGH,2048); //Returns the pulse length in microseconds
    //rssipulselength = PulseIn(PWMRSSIPIN, HIGH,1024); //Returns the pulse length in microseconds -> Not very precise
    
    //For a frequency of 1kHz the cycle duration is 1000us (1ms). 
    uint16_t dutycycle;
    
    dutycycle = (uint16_t)(float)(rssipulselength * .1f); // DutyCycle = PulseLength / CycleDuration * 100

    if (dutycycle==0) {
      osd_rssi=osd_rssi; //Keeps the last value
    }
    else if (dutycycle>254) {
      osd_rssi=255;
    } else {
      osd_rssi = (uint8_t)(dutycycle);
    }
    
	} else {
#ifdef JR_SPECIALS
// SEARCH GLITCH
		osd_rssi = analogRead(RSSI_PIN)       / 4;			// 1:1 input
#else
		osd_rssi = analogRead(RSSI_PIN) * .2  / 4 + osd_rssi * .8;	// Smooth input
    //osd_rssi = (uint8_t)(float)(((osd_chan7_raw - 992) * .31135f) * .2 + osd_rssi * .8);
#endif
		osd_rssi = constrain(osd_rssi, rssipersent, rssical);		// Ensure we stay in range
	}
}

uint16_t FastpulseIn(uint8_t pin, uint8_t state, unsigned long timeout)
{
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  uint8_t stateMask = (state ? bit : 0);
  uint16_t width = 0;
  unsigned long numloops = 0;
  unsigned long maxloops = timeout;
  
  while ((*portInputRegister(port) & bit) == stateMask)
    if (numloops++ == maxloops)
      return 0;
  
  while ((*portInputRegister(port) & bit) != stateMask)
    if (numloops++ == maxloops)
      return 0;
  
  while ((*portInputRegister(port) & bit) == stateMask) {
    if (numloops++ == maxloops)
      return 0;
    width++;
  }
  return width; 
}
