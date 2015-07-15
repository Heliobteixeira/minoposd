/**
 ******************************************************************************
 *
 * @file       PacketRxOk.h
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

/*
 * PacketRxOk is a digital version of RSSI.
 * It uses the green LED on 2.4GHz RC-RX which blinks when packets are lost.
 * It works well with the LED behavior of the GigaScan RX.
 * The Futaba RX begins to blink too late, just before total loss.
*/


#ifndef PACKETRXOK_H_
#define PACKETRXOK_H_


#define PRO_PIN				3
#define PRO_INT  			1

void PacketRxOk_init(void);
void PacketRxOk_read(void);


#endif /* PACKETRXOK_H_ */
