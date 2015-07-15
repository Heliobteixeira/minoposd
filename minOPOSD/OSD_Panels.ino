//
//
//

/*

refactoring started

TODO:

	refactor:
		switchPanels
		casts (double) -> (float) etc.
	
	maybe implement usage of panCallsign

*/


#include "OSD_Config.h"

#ifdef FLIGHT_BATT_ON_MINIMOSD
#include "FlightBatt.h"
#endif

#ifdef PACKETRXOK_ON_MINIMOSD
#include "PacketRxOk.h"
#endif


#define PWM_LO			1200	// [us]	PWM low value
#define PWM_HI			1800	// [us]	PWM high value
#define PWM_OFFSET		100	// [us]	PWM offset for detecting stick movement

#define SETUP_TIME		30000	// [ms]	time after boot while we can enter the setup menu
#define SETUP_DEBOUNCE_TIME	500	// [ms]	time for RC-TX stick debouncing
#define SETUP_LOWEST_MENU	1	//	lowest shown setup menue item
#ifndef FLIGHT_BATT_ON_MINIMOSD
#define SETUP_HIGHEST_MENU	2	//	highest shown setup menue item
#else
#define SETUP_HIGHEST_MENU	11	//	highest shown setup menue item
#endif

#define WARN_FLASH_TIME		1000	// [ms]	time with which the warnings are flashing
#define WARN_RECOVER_TIME	4000	// [ms]	time we stay in the first panel after last warning
#ifdef JR_SPECIALS
#define WARN_MAX		6	//	number of implemented warnings
#else
#define WARN_MAX		5	//	number of implemented warnings
#endif

#define MODE_SWITCH_TIME	2000	// [ms]	time for mode switching

#define TIME_RESET_AMPERE	2	// [A]	current above which the on time is set to 00:00


/******* GLOBAL VARS *******/

static boolean		setup_menu_active = false;
static boolean		warning_active = false;

static float		convert_speed = 0;
static float		convert_length = 0;
static int16_t		convert_length_large = 0;
static uint8_t		unit_speed = 0;
static uint8_t		unit_length = 0;
static uint8_t		unit_length_large = 0;

static int16_t		chan1_raw_middle = 0;
static int16_t		chan2_raw_middle = 0;


/******* MAIN FUNCTIONS *******/


/******************************************************************/
// Panel  : startPanels
// Output : Logo panel and initialization
/******************************************************************/
void startPanels() {
    osd.clear();
    panLogo();		// display logo
    set_converts();	// initialize the units
}


/******************************************************************/
// Panel  : writePanels
// Output : Write the panels
/******************************************************************/
void writePanels() {

#ifdef JR_SPECIALS
    // reset osd_home to current position when 2 TX switches are in a special position
    if (osd_chan6_raw > PWM_HI && osd_chan7_raw < PWM_LO) {
        osd_home_lat = osd_lat;
        osd_home_lon = osd_lon;
        osd_home_alt = osd_alt;
        osd.setPanel(10, 3);
        osd.openPanel();
        osd.printf_P(PSTR("reset home"));
        osd.closePanel();
    } else if (ch_toggle > 3) switchPanels();									// switch panels
#else
    if (ch_toggle > 3) switchPanels();										// switch panels
#endif

    if (setup_menu_active) {
        panSetup();
    } else {
        if (ISd(0,Warn_BIT)) panWarn(panWarn_XY[0][0], panWarn_XY[1][0]);					// ever check/display warnings
        if (panel < npanels) {											// first or second panel
	    // these GPS related panels are active under all circumstances
            if (ISa(panel,GPSats_BIT))		panGPSats(panGPSats_XY[0][panel], panGPSats_XY[1][panel]);	// number of visible sats
            if (ISa(panel,GPL_BIT))		panGPL(panGPL_XY[0][panel], panGPL_XY[1][panel]);		// sat fix type

	    // these GPS related panels are active if GPS was valid before
	    if (osd_got_home) {
		if (ISa(panel,GPS_BIT))		panGPS(panGPS_XY[0][panel], panGPS_XY[1][panel]);		// lat & lon
		if (ISb(panel,HDis_BIT))	panHomeDis(panHomeDis_XY[0][panel], panHomeDis_XY[1][panel]);
                if (ISb(panel,HDir_BIT))	panHomeDir(panHomeDir_XY[0][panel], panHomeDir_XY[1][panel]);
	    }

	    // these GPS related panels are active if GPS was valid before and we have a sat fix
	    if (osd_got_home && osd_fix_type > 1) {
                if (ISc(panel,Halt_BIT))	panHomeAlt(panHomeAlt_XY[0][panel], panHomeAlt_XY[1][panel]);
                if (ISc(panel,Alt_BIT))		panAlt(panAlt_XY[0][panel], panAlt_XY[1][panel]);
                if (ISc(panel,Vel_BIT))		panVel(panVel_XY[0][panel], panVel_XY[1][panel]);
//                if (ISe(panel,DIST_BIT))	panDistance(panDistance_XY[0][panel], panDistance_XY[1][panel]);
                if (ISd(panel,Climb_BIT))	panClimb(panClimb_XY[0][panel], panClimb_XY[1][panel]);
                if (ISb(panel,Head_BIT))	panHeading(panHeading_XY[0][panel], panHeading_XY[1][panel]);
                if (ISb(panel,Rose_BIT))	panRose(panRose_XY[0][panel], panRose_XY[1][panel]);
	    }

	    if (ISd(panel,RSSI_BIT))		panRSSI(panRSSI_XY[0][panel], panRSSI_XY[1][panel]);
            if (ISa(panel,Rol_BIT))		panRoll(panRoll_XY[0][panel], panRoll_XY[1][panel]);
            if (ISa(panel,Pit_BIT))		panPitch(panPitch_XY[0][panel], panPitch_XY[1][panel]);
            if (ISc(panel,Thr_BIT))		panThr(panThr_XY[0][panel], panThr_XY[1][panel]);
            if (ISc(panel,FMod_BIT))		panFlightMode(panFMod_XY[0][panel], panFMod_XY[1][panel]);
            if (ISa(panel,BatA_BIT))		panBatt_A(panBatt_A_XY[0][panel], panBatt_A_XY[1][panel]);
            if (ISc(panel,CurA_BIT))		panCur_A(panCur_A_XY[0][panel], panCur_A_XY[1][panel]);
            if (ISa(panel,Bp_BIT))		panBatteryPercent(panBatteryPercent_XY[0][panel], panBatteryPercent_XY[1][panel]);
            if (ISb(panel,Time_BIT))		panTime(panTime_XY[0][panel], panTime_XY[1][panel]);
	    
            if (ISc(panel,Hor_BIT))		panHorizon(panHorizon_XY[0][panel], panHorizon_XY[1][panel]);
#ifdef SHOW_RADAR
	    // these GPS related panels are active if GPS was valid before
	    if (osd_got_home) {
                if (ISc(panel,Hor_BIT))		panUAVPosition(panHorizon_XY[0][panel] + 6, panHorizon_XY[1][panel] + 2);
	    }
#endif
	} else { // off panel
		panOff();
	}
    }

#ifdef membug
    // OSD debug for development
    osd.setPanel(13,4);
    osd.openPanel();
    osd.printf("%i", freeMem()); 
    osd.closePanel();
#endif
}


/******************************************************************/
// Panel  : switchPanels
// Output : Switch between panels
// TODO   : REFACTOR
/******************************************************************/
void switchPanels() {
    static uint8_t      	osd_off_switch = 0;
    static uint8_t      	osd_switch_last = 100;
    static unsigned long	osd_switch_time = 0;
    static uint16_t		ch_raw = 0;

    if (ch_toggle == 4) {
        if (osd_mode < 4) {
            if (osd_off_switch != osd_mode) { 
                osd_off_switch = osd_mode;
                osd_switch_time = millis();

                if (osd_off_switch == osd_switch_last) {
                    switch (panel) {
			case 0:
                            panel = 1;                                                        
                            if (millis() <= SETUP_TIME) {
                                setup_menu_active = true;
                            } else {
                                setup_menu_active = false;
                            }                            
                           break;
			case 1:
                            panel = npanels;
                            setup_menu_active = false;
                           break;
			case npanels:
                            panel = 0;
                            break;
                    }
                    osd.clear();
                }
            }
            if ((millis() - osd_switch_time) > MODE_SWITCH_TIME) {
                osd_switch_last = osd_mode;
            }
        }
    }
    else {
        if (ch_toggle == 5) ch_raw = osd_chan5_raw;
        else if (ch_toggle == 6) ch_raw = osd_chan6_raw;
        else if (ch_toggle == 7) ch_raw = osd_chan7_raw;
        else if (ch_toggle == 8) ch_raw = osd_chan8_raw;

        if (switch_mode == 0) {
            if (ch_raw > PWM_HI) {
                if (millis() <= SETUP_TIME) {
                    setup_menu_active = true;
                }
                else if (!setup_menu_active && !warning_active) {
                    osd.clear();
                }
                panel = npanels;				// off panel
            }
            else if (ch_raw < PWM_LO && panel != 0) {
                setup_menu_active = false;
                osd.clear();
                panel = 0;					// first panel
            }
            else if (ch_raw >= PWM_LO && ch_raw <= PWM_HI && panel != 1 && !warning_active) {
                setup_menu_active = false;
                osd.clear();
                panel = 1;					// second panel
            }        
        } else {
            if (ch_raw > PWM_LO) {
                if (millis() <= SETUP_TIME && !setup_menu_active) {
                    if (osd_switch_time + MODE_SWITCH_TIME / 2 < millis()) {
                        setup_menu_active = true;
                        osd_switch_time = millis();
                    }
                } else {
                    if (osd_switch_time + MODE_SWITCH_TIME / 2 < millis()) {
                        setup_menu_active = false;
                        osd.clear();
                        if (panel == npanels) {
                            panel = 0;
                        } else {
                            panel++;
                        }
                        if (panel > 1) panel = npanels;
                        osd_switch_time = millis();
                    }
                }
	    }
        }    
    }
}


/******* SPECIAL PANELS *******/


/******************************************************************/
// Panel  : panOff
// Needs  : -
// Output : -
/******************************************************************/
void panOff(void) {
#ifdef JR_SPECIALS
// SEARCH GLITCH
    panGPS(panGPS_XY[0][0], panGPS_XY[1][0]);
    panRSSI(panRSSI_XY[0][0], panRSSI_XY[1][0]);
    panGPL(panGPL_XY[0][0], panGPL_XY[1][0]);
    panFlightMode(panFMod_XY[0][0], panFMod_XY[1][0]);
    panThr(panThr_XY[0][0], panThr_XY[1][0]);
#endif
}


/******************************************************************/
// Panel  : panWarn
// Needs  : X, Y locations
// Output : Warnings if there are any
/******************************************************************/
void panWarn(int first_col, int first_line) {
    static char* warning_string;
    static uint8_t last_warning_type = 1;
    static uint8_t warning_type = 0;
    static unsigned long warn_text_timer = 0;
    static unsigned long warn_recover_timer = 0;
    int cycle;

    if (millis() > warn_text_timer) {				// if the text or blank text has been shown for a while
        if (warning_type) {					// there was a warning, so we now blank it out for a while
            last_warning_type = warning_type;			// save the warning type for cycling
            warning_type = 0;
	    warning_string = "            ";			// blank the warning
	    warn_text_timer = millis() + WARN_FLASH_TIME / 2;	// set clear warning time
        } else {
            cycle = last_warning_type;				// start the warning checks cycle where we left it last time
            do {				                // cycle through the warning checks
                if (++cycle > WARN_MAX) cycle = 1;
                switch (cycle) {
                case 1:						// DISARMED
		    if (osd_armed < 2) {
			warning_type = cycle;
			warning_string = "  disarmed  ";
		    }
                    break;
                case 2:						// No telemetry communication
		    if (uavtalk_state() != TELEMETRYSTATS_STATE_CONNECTED) {
			warning_type = cycle;
			warning_string = " no tel com ";
		    }
                    break;
		case 3:						// NO GPS FIX
                    if (osd_fix_type < 2 && osd_got_home) {	// to allow flying in the woods (what I really like) without this annoying warning,
			warning_type = cycle;			// this warning is only shown if GPS was valid before (osd_got_home)
			warning_string = " no gps fix ";
		    }
                    break;
                case 4:						// BATT LOW
#if defined FLIGHT_BATT_ON_MINIMOSD || defined FLIGHT_BATT_ON_REVO
                    if (osd_vbat_A < battv/10.0) {
#else
                    if (osd_vbat_A < float(battv)/10.0 || osd_battery_remaining_A < batt_warn_level) {
#endif
			warning_type = cycle;
			warning_string = "  batt low  ";
		    }
                    break;
                case 5:						// RSSI LOW
                    if (rssi < rssi_warn_level && rssi != -99 && !rssiraw_on) {
			warning_type = cycle;
			warning_string = "  rssi low  ";
		    }
                    break;
#ifdef JR_SPECIALS
                case 6:						// FAILSAFE
                    if (osd_chan8_raw > PWM_HI) {
			warning_type = cycle;
			warning_string = "  failsafe  ";
		    }
                    break;
#endif
                }
            } while (!warning_type && cycle != last_warning_type);
	    if (warning_type) {					// if there a warning
		warning_active = true;				// then set warning active
		warn_text_timer = millis() + WARN_FLASH_TIME;	// set show warning time
		warn_recover_timer = millis() + WARN_RECOVER_TIME;
		if (panel > 0) osd.clear();
		panel = 0;					// switch to first panel if there is a warning
	    } else {						// if not, we do not want the delay, so a new error shows up immediately
		if (millis() > warn_recover_timer) {		// if recover time over since last warning
		    warning_active = false;			// no warning active anymore
		}
	    }
        }

	osd.setPanel(first_col, first_line);
	osd.openPanel();
        osd.printf("%s", warning_string);
	osd.closePanel();
    }
}


/******************************************************************/
// Panel  : panSetup
// Needs  : Nothing, uses whole screen
// Output : The settings menu
/******************************************************************/
void panSetup() {
    static unsigned long setup_debounce_timer = 0;
    static int8_t setup_menu = 0;
    int delta = 100;

    if (millis() > setup_debounce_timer) {			// RC-TX stick debouncing
	setup_debounce_timer = millis() + SETUP_DEBOUNCE_TIME;
	
        osd.clear();
        osd.setPanel(5, 3);
        osd.openPanel();
	
        osd.printf_P(PSTR("setup screen|||"));

        if (chan1_raw_middle == 0 || chan2_raw_middle == 0) {
            chan1_raw_middle = chan1_raw;
            chan2_raw_middle = chan2_raw;
        }

        if ((chan2_raw - PWM_OFFSET) > chan2_raw_middle ) setup_menu++;
        else if ((chan2_raw + PWM_OFFSET) < chan2_raw_middle ) setup_menu--;
	
        if (setup_menu < SETUP_LOWEST_MENU) setup_menu = SETUP_LOWEST_MENU;
        else if (setup_menu > SETUP_HIGHEST_MENU) setup_menu = SETUP_HIGHEST_MENU;

        switch (setup_menu) {
            case 1:
                osd.printf_P(PSTR("uavtalk "));
		if (op_uavtalk_mode & UAVTALK_MODE_PASSIVE) {
			osd.printf_P(PSTR("passive"));
			if (chan1_raw < chan1_raw_middle - PWM_OFFSET)
				op_uavtalk_mode &= ~UAVTALK_MODE_PASSIVE;
		} else {
			osd.printf_P(PSTR("active "));
			if (chan1_raw > chan1_raw_middle + PWM_OFFSET)
				op_uavtalk_mode |= UAVTALK_MODE_PASSIVE;
		}
                break;
            case 2:
                osd.printf_P(PSTR("battery warning "));
                osd.printf("%3.1f%c", float(battv)/10.0 , 0x76, 0x20);
                battv = change_val(battv, battv_ADDR);
                break;
#ifdef FLIGHT_BATT_ON_MINIMOSD
            case 5:
	        delta /= 10;
            case 4:
	        delta /= 10;
            case 3:
	        // volt_div_ratio
                osd.printf_P(PSTR("calibrate||measured volt: "));
                osd.printf("%c%5.2f%c", 0xE2, (float)osd_vbat_A, 0x8E);
                osd.printf("||volt div ratio:  %5i", volt_div_ratio);
                volt_div_ratio = change_int_val(volt_div_ratio, volt_div_ratio_ADDR, delta);
                break;
	    case 8:
	        delta /= 10;
	    case 7:
	        delta /= 10;
	    case 6:
	        // curr_amp_offset
                osd.printf_P(PSTR("calibrate||measured amp:  "));
                osd.printf("%c%5.2f%c", 0xE2, osd_curr_A * .01, 0x8F);
                osd.printf("||amp offset:      %5i", curr_amp_offset);
                curr_amp_offset = change_int_val(curr_amp_offset, curr_amp_offset_ADDR, delta);
                break;
	    case 11:
	        delta /= 10;
	    case 10:
	        delta /= 10;
	    case 9:
		// curr_amp_per_volt
                osd.printf_P(PSTR("calibrate||measured amp:  "));
                osd.printf("%c%5.2f%c", 0xE2, osd_curr_A * .01, 0x8F);
                osd.printf("||amp per volt:    %5i", curr_amp_per_volt);
                curr_amp_per_volt = change_int_val(curr_amp_per_volt, curr_amp_per_volt_ADDR, delta);
                break;
#endif
        }
        osd.closePanel();
    }
}


/******* PANELS *******/


/******************************************************************/
// Panel  : panBoot
// Needs  : X, Y locations
// Output : Booting up text and empty bar after that
/******************************************************************/
void panBoot(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf_P(PSTR("booting up:\xed\xf2\xf2\xf2\xf2\xf2\xf2\xf2\xf3")); 
    osd.closePanel();
}


/******************************************************************/
// Panel  : panLogo
// Needs  : X, Y locations
// Output : Startup OSD LOGO
/******************************************************************/
void panLogo() {
#ifdef USE_WITH_MINRXOSD
    osd.setPanel(5, 12);
    osd.openPanel();
    VERSION_STRING
#else
    osd.setPanel(3, 5);
    osd.openPanel();
    osd.printf_P(PSTR("\x20\x20\x20\x20\x20\xba\xbb\xbc\xbd\xbe|\x20\x20\x20\x20\x20\xca\xcb\xcc\xcd\xce|"));
    VERSION_STRING
#endif
#ifdef PACKETRXOK_ON_MINIMOSD
    osd.printf_P(PSTR(" prxok"));
#endif
#ifdef ANALOG_RSSI_ON_MINIMOSD
    osd.printf_P(PSTR(" arssi"));
#endif
#ifdef JR_SPECIALS
    osd.printf_P(PSTR(" jrs"));
#endif
    osd.closePanel();
}


/******************************************************************/
// Panel  : panGPSats
// Needs  : X, Y locations
// Output : 1 symbol and number of locked satellites
/******************************************************************/
void panGPSats(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
#ifdef JR_SPECIALS	// I like it more this way
    osd.printf("%c%3i", 0x0f, osd_satellites_visible);
#else
    osd.printf("%c%2i", 0x0f, osd_satellites_visible);
#endif
    osd.closePanel();
}


/******************************************************************/
// Panel  : panGPL
// Needs  : X, Y locations
// Output : 1 static symbol open lock or 2D or 3D sign
/******************************************************************/
void panGPL(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c", osd_fix_type<=1 ? osd_fix_type*0x10 : osd_fix_type-1);
#ifdef OP_DEBUG		// I use this place for debug info
    osd.printf(" %02x", op_alarm);
#endif
    osd.closePanel();
}


/******************************************************************/
// Panel  : panGPS
// Needs  : X, Y locations
// Output : two row numeric value of current GPS location with LAT/LON symbols
/******************************************************************/
void panGPS(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
#ifdef JR_SPECIALS	// I like it more one row style
    osd.printf("%c%10.6f     %c%10.6f", 0x83, (double)(osd_lat), 0x84, (double)(osd_lon));
#else
    osd.printf("%c%11.6f|%c%11.6f", 0x83, (double)osd_lat, 0x84, (double)osd_lon);
#endif
    osd.closePanel();
}


/******************************************************************/
// Panel  : panHomeDis
// Needs  : X, Y locations
// Output : Distance to home
/******************************************************************/
void panHomeDis(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%5.0f%c", 0x1F, (double)((osd_home_distance) * convert_length), unit_length);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panHomeDir
// Needs  : X, Y locations
// Output : 2 symbols that are combined as one arrow, shows direction to home
/******************************************************************/
void panHomeDir(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    showArrow((uint8_t)osd_home_direction);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panHomeAlt
// Needs  : X, Y locations
// Output : Hom altitude
/******************************************************************/
void panHomeAlt(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
#ifdef REVO_ADD_ONS
    osd.printf("%c%5.0f%c", 0xE7, (double)(revo_baro_alt * convert_length), unit_length);
#else
    osd.printf("%c%5.0f%c", 0xE7, (double)((osd_alt - osd_home_alt) * convert_length), unit_length);
#endif
    osd.closePanel();
}


/******************************************************************/
// Panel  : panAlt
// Needs  : X, Y locations
// Output : Altitude
/******************************************************************/
void panAlt(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%5.0f%c",0xE6, (double)(osd_alt * convert_length), unit_length);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panVel
// Needs  : X, Y locations
// Output : Velocity 
/******************************************************************/
void panVel(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
#ifdef JR_SPECIALS	// I like it more this way
    osd.printf("%c%5.0f%c", 0xE9, (double)(osd_groundspeed * convert_speed), unit_speed);
#else
    osd.printf("%c%3.0f%c", 0xE9, (double)(osd_groundspeed * convert_speed), unit_speed);
#endif
    osd.closePanel();
}


/******************************************************************/
// Panel  : panDistance
// Needs  : X, Y locations
// Output : travel distance
/******************************************************************/
void panDistance(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    if ((osd_travel_distance * convert_length) > 1000.0) {
        osd.printf("%c%5.2f%c", 0xFE, ((osd_travel_distance * convert_length) / convert_length_large), unit_length_large);
    } else {
        osd.printf("%c%5.0f%c", 0xFE, (osd_travel_distance * convert_length), unit_length);
    }
    osd.closePanel();
}


/******************************************************************/
// Panel  : panClimb
// Needs  : X, Y locations
// Output : Climb Rate
/******************************************************************/
void panClimb(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
#ifdef JR_SPECIALS	// I like it more this way
    osd.printf("%c%5.0f%c", 0x16, (double)(osd_climb), 0x88);
#else
    osd.printf("%c%3.0f%c", 0x16, (double)(osd_climb), 0x88);
#endif
    osd.closePanel();
}


/******************************************************************/
// Panel  : panHeading
// Needs  : X, Y locations
// Output : Symbols with numeric compass heading value
/******************************************************************/
void panHeading(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
#ifdef JR_SPECIALS	// show heading with compass point
    osd.printf("%4.0f%c%s", (double)osd_heading, 0xb0, CompassPoint);
#else
    osd.printf("%4.0f%c", (double)osd_heading, 0xb0);
#endif
    osd.closePanel();
}


/******************************************************************/
// Panel  : panRose
// Needs  : X, Y locations
// Output : a dynamic compass rose that changes along the heading information
/******************************************************************/
void panRose(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%s|%c%s%c", "\x20\xc0\xc0\xc0\xc0\xc0\xc7\xc0\xc0\xc0\xc0\xc0\x20", 0xd0, buf_show, 0xd1);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panRSSI
// Needs  : X, Y locations
// Output : RSSI %
/******************************************************************/
void panRSSI(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%3i%c", 0xE1, rssi, 0x25);
    osd.closePanel();
#ifdef REVO_ADD_ONS
    osd.setPanel(first_col-2, first_line-1);
    osd.openPanel();
    osd.printf("%4i%c%3i%c", oplm_rssi, 0x8B, oplm_linkquality, 0x8C);
    osd.closePanel();
#endif
}


/******************************************************************/
// Panel  : panRoll
// Needs  : X, Y locations
// Output : -+ value of current Roll from vehicle with degree symbols and roll symbol
/******************************************************************/
void panRoll(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%4i%c%c", osd_roll, 0xb0, 0xb2);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panPitch
// Needs  : X, Y locations
// Output : -+ value of current Pitch from vehicle with degree symbols and pitch symbol
/******************************************************************/
void panPitch(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%4i%c%c", osd_pitch, 0xb0, 0xb1);
    osd.closePanel();
}

  
/******************************************************************/
// Panel  : panThr
// Needs  : X, Y locations
// Output : Throttle 
/******************************************************************/
void panThr(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%3.0i%c", 0x87, osd_throttle, 0x25);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panFlightMode 
// Needs  : X, Y locations
// Output : current flight modes
/******************************************************************/
void panFlightMode(int first_col, int first_line) {
    char* mode_str="";
    
    osd.setPanel(first_col, first_line);
    osd.openPanel();
#if defined VERSION_RELEASE_12_10_1 || defined VERSION_RELEASE_12_10_2 || defined VERSION_RELEASE_13_06_1 || defined VERSION_RELEASE_13_06_2 || defined VERSION_RELEASE_14_01_1
    if      (osd_mode ==  0 ) mode_str = "man";	// MANUAL
    else if (osd_mode ==  1 ) mode_str = "st1";	// STABILIZED1
    else if (osd_mode ==  2 ) mode_str = "st2";	// STABILIZED2
    else if (osd_mode ==  3 ) mode_str = "st3";	// STABILIZED3
    else if (osd_mode ==  4 ) mode_str = "at ";	// AUTOTUNE
    else if (osd_mode ==  5 ) mode_str = "alh";	// ALTITUDEHOLD
    else if (osd_mode ==  6 ) mode_str = "alv";	// ALTITUDEVARIO
    else if (osd_mode ==  7 ) mode_str = "vc ";	// VELOCITYCONTROL
    else if (osd_mode ==  8 ) mode_str = "ph ";	// POSITIONHOLD
    else if (osd_mode ==  9 ) mode_str = "rtb";	// RETURNTOBASE
    else if (osd_mode == 10 ) mode_str = "lan";	// LAND
    else if (osd_mode == 11 ) mode_str = "pp ";	// PATHPLANNER
    else if (osd_mode == 12 ) mode_str = "poi";	// POI
#endif
#if defined VERSION_RELEASE_14_06_1 || defined VERSION_RELEASE_14_10_1
    if      (osd_mode ==  0) mode_str = "man";	// MANUAL
    else if (osd_mode ==  1) mode_str = "st1";	// STABILIZED1
    else if (osd_mode ==  2) mode_str = "st2";	// STABILIZED2
    else if (osd_mode ==  3) mode_str = "st3";	// STABILIZED3
    else if (osd_mode ==  4) mode_str = "st4";	// STABILIZED4
    else if (osd_mode ==  5) mode_str = "st5";	// STABILIZED5
    else if (osd_mode ==  6) mode_str = "st6";	// STABILIZED6
    else if (osd_mode ==  7) mode_str = "at ";	// AUTOTUNE
    else if (osd_mode ==  8) mode_str = "ph ";	// POSITIONHOLD
    else if (osd_mode ==  9) mode_str = "pvf";	// POSITIONVARIOFPV
    else if (osd_mode == 10) mode_str = "pvl";	// POSITIONVARIOLOS
    else if (osd_mode == 11) mode_str = "pvd";	// POSITIONVARIONSEW
    else if (osd_mode == 12) mode_str = "rtb";	// RETURNTOBASE
    else if (osd_mode == 13) mode_str = "lan";	// LAND
    else if (osd_mode == 14) mode_str = "pp ";	// PATHPLANNER
    else if (osd_mode == 15) mode_str = "poi";	// POI
    else if (osd_mode == 16) mode_str = "ac ";	// AUTOCRUISE
#endif
#if defined VERSION_RELEASE_15_01_1 || defined VERSION_RELEASE_15_02_1
    if      (osd_mode ==  0) mode_str = "man";	// MANUAL
    else if (osd_mode ==  1) mode_str = "st1";	// STABILIZED1
    else if (osd_mode ==  2) mode_str = "st2";	// STABILIZED2
    else if (osd_mode ==  3) mode_str = "st3";	// STABILIZED3
    else if (osd_mode ==  4) mode_str = "st4";	// STABILIZED4
    else if (osd_mode ==  5) mode_str = "st5";	// STABILIZED5
    else if (osd_mode ==  6) mode_str = "st6";	// STABILIZED6
    else if (osd_mode ==  7) mode_str = "ph ";	// POSITIONHOLD
    else if (osd_mode ==  8) mode_str = "cl ";	// COURSELOCK
    else if (osd_mode ==  9) mode_str = "pr ";	// POSITIONROAM
    else if (osd_mode == 10) mode_str = "hl ";	// HOMELEASH
    else if (osd_mode == 11) mode_str = "pa ";	// ABSOLUTEPOSITION
    else if (osd_mode == 12) mode_str = "rtb";	// RETURNTOBASE
    else if (osd_mode == 13) mode_str = "lan";	// LAND
    else if (osd_mode == 14) mode_str = "pp ";	// PATHPLANNER
    else if (osd_mode == 15) mode_str = "poi";	// POI
    else if (osd_mode == 16) mode_str = "ac ";	// AUTOCRUISE
#endif    
    osd.printf("%c%s", 0xE0, mode_str);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panBattery A (Voltage 1)
// Needs  : X, Y locations
// Output : Voltage value as in XX.X and symbol of over all battery status
/******************************************************************/
void panBatt_A(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%5.2f%c", 0xE2, (double)osd_vbat_A, 0x8E);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panCur_A
// Needs  : X, Y locations
// Output : Current
/******************************************************************/
void panCur_A(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%5.2f%c", 0xE4, osd_curr_A * .01, 0x8F);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panBatteryPercent
// Needs  : X, Y locations
// Output : Battery
//          (if defined FLIGHT_BATT_ON_MINIMOSD || defined FLIGHT_BATT_ON_REVO then not percent but consumed mAh)
/******************************************************************/
void panBatteryPercent(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
#if defined FLIGHT_BATT_ON_MINIMOSD || defined FLIGHT_BATT_ON_REVO
    osd.printf("%c%5i%c", 0xB9, osd_total_A, 0x82);
#else
    osd.printf("%c%3.0i%c", 0xB9, osd_battery_remaining_A, 0x25);
#endif
    osd.closePanel();
}


/******************************************************************/
// Panel  : panTime
// Needs  : X, Y locations
// Output : Time from bootup or start
/******************************************************************/
void panTime(int first_col, int first_line) {
    int start_time;

#ifdef JR_SPECIALS	// Time and travel distance reset when measured current > TIME_RESET_AMPERE for the 1st time
    static unsigned long engine_start_time = 0;
    
    if (engine_start_time == 0 && osd_curr_A > TIME_RESET_AMPERE * 100) {
        engine_start_time = millis();
	osd_travel_distance = 0;
    }
    start_time = (int) ((millis() - engine_start_time) / 1000);
#else
    start_time = (int) (millis() / 1000);
#endif
    
    osd.setPanel(first_col, first_line);
    osd.openPanel();
#if defined FLIGHT_BATT_ON_REVO
    osd.printf("%c%2i%c%02i|%c%2i%c%02i", 0xB3, ((int)(start_time/60))%60, 0x3A, start_time%60, 0x65, ((int)(osd_est_flight_time/60))%60, 0x3A, osd_est_flight_time%60);
#else
    osd.printf("%c%2i%c%02i", 0xB3, ((int)(start_time/60))%60, 0x3A, start_time%60);
#endif
    osd.closePanel();
}


/******************************************************************/
// Panel  : panHorizon
// Needs  : X, Y locations
// Output : artificial horizon
/******************************************************************/
void panHorizon(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf_P(PSTR("\xc8\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xc9|"));
    osd.printf_P(PSTR("\xc8\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xc9|"));
    osd.printf_P(PSTR("\xd8\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xd9|"));
    osd.printf_P(PSTR("\xc8\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xc9|"));
    osd.printf_P(PSTR("\xc8\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xc9"));
    osd.closePanel();
    showHorizon((first_col + 1), first_line);
}


#ifdef SHOW_RADAR
/******************************************************************/
// Panel  : panUAVPosition
// Needs  : X, Y locations of center
// Needs  : globals: osd_home_lat, osd_lat, osd_home_lon, osd_lon
// Output : shows the UAV position in a radar like style
// Status : do flight test
/******************************************************************/

#define USE_DIRECTED				// use directed UAV icons (special mcm file needed)

#define	STEP_WIDTH	250			// every STEP_WIDTH in [m] it is down-scaled

#define	SCALE_X		(7.0 / STEP_WIDTH)	// SCALE_X * 2 chars grid in which the uav is drawed
#define	SCALE_Y		(4.5 / STEP_WIDTH)	// SCALE_Y * 2 chars grid in which the uav is drawed

#define RADAR_CHAR	0xF9			// code of the radar symbol
#define UAV_CHAR_START	0x17			// code of the first of 8 directed UAV icons

void panUAVPosition(int center_col, int center_line) {
    static int last_x = 0;
    static int last_y = 0;
    
#ifdef USE_DIRECTED
    int index;
    index = (int)((osd_heading + 22.5) / 45.0);
    index = index > 7 ? 0 : index;
#endif
    
    // calculate distances from home in lat (y) and lon (x) direction in [m]
    int dy = (int)(-111319.5 * (osd_home_lat - osd_lat));
    int dx = (int)(-111319.5 * (osd_home_lon - osd_lon) * cos(fabs(osd_home_lat) * 0.0174532925));
    
    // calculate display offset in y and x direction
    int zoom = max((int)(abs(dy) / STEP_WIDTH), (int)(abs(dx) / STEP_WIDTH)) + 1;
    osd.setPanel(center_col + 8, center_line);
    osd.openPanel();
    osd.printf("%c%5i%c", RADAR_CHAR, (int)(zoom * STEP_WIDTH * convert_length), unit_length);
    osd.closePanel();
    int y = (int)(dy / (zoom / SCALE_Y));
    int x = (int)(dx / (zoom / SCALE_X) + 0.5);	// for even grid correction

    // clear UAV
    osd.openSingle(center_col + last_x, center_line - last_y);
    osd.printf_P(PSTR(" "));
    last_x = x;
    last_y = y;
    // show UAV
    osd.openSingle(center_col + x, center_line - y);
#ifdef USE_DIRECTED
    osd.printf("%c", UAV_CHAR_START + index);
#else
    osd.printf_P(PSTR("\xF4"));
#endif
    // show home
    osd.setPanel(center_col, center_line);
    osd.openPanel();
    osd.printf_P(PSTR("\xF5\xF6"));
    osd.closePanel();
}
#endif


#if 0
/******************************************************************/
// Panel  : panOtherUAV
// Needs  : X, Y locations
// Needs  : globals: oUAV_lat, oUAV_lon, oUAV_alt, osd_heading
// Output : shows some mystic info
// Size   : 3 x 6  (rows x chars)
// Status : just an idea and not to forget
//          not compiled
//          not tested
//          not ready
// ToDo   : refactor with function setHomeVars
/******************************************************************/
void panOtherUAV(int first_col, int first_line) {
    float dstlon, dstlat;
    long oUAV_distance;
    long oUAV_bearing;
    uint8_t oUAV_direction;
    
    // shrinking factor for longitude going to poles direction
    float rads = fabs(oUAV_lat) * 0.0174532925;
    double scaleLongDown = cos(rads);
    double scaleLongUp   = 1.0f/cos(rads);
   
    // DST to oUAV
    dstlat = fabs(oUAV_lat - osd_lat) * 111319.5;
    dstlon = fabs(oUAV_lon - osd_lon) * 111319.5 * scaleLongDown;
    oUAV_distance = sqrt(sq(dstlat) + sq(dstlon));
    
    // DIR to oUAV
    dstlon = (oUAV_lon - osd_lon);					// OffSet X
    dstlat = (oUAV_lat - osd_lat) * scaleLongUp;			// OffSet Y
    oUAV_bearing = 90 + (atan2(dstlat, -dstlon) * 57.295775);		// absolut oUAV direction
    if (oUAV_bearing < 0) oUAV_bearing += 360;				// normalization
    oUAV_bearing = oUAV_bearing - 180;					// absolut goal direction
    if (oUAV_bearing < 0) oUAV_bearing += 360;				// normalization
    oUAV_bearing = oUAV_bearing - osd_heading;				// relative oUAV direction
    if (oUAV_bearing < 0) oUAV_bearing += 360;				// normalization
    oUAV_direction = round((float)(oUAV_bearing/360.0f) * 16.0f) + 1;	// array of arrows
    if (oUAV_direction > 16) oUAV_direction = 0;
    
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("D%4.0f%c|", (double)((float)(oUAV_distance) * convert_length), unit_length);
    osd.printf("A%4.0f%c|  ", (double)((oUAV_alt - osd_alt) * convert_length), unit_length);
    showArrow((uint8_t)oUAV_direction);
    osd.closePanel();
}
#endif


/******* HELPER FUNCTIONS *******/


// Setup change function
int change_int_val(int value, int address, int delta) {
    int value_old = value;
    
    osd.printf_P(PSTR("|                   "));
    switch (delta) {
        case 100:
            osd.printf_P(PSTR("\xBF"));
	break;
        case 10:
            osd.printf_P(PSTR(" \xBF"));
	break;
        case 1:
            osd.printf_P(PSTR("  \xBF"));
	break;
    }
		
    if (chan1_raw > chan1_raw_middle + PWM_OFFSET) value -= delta;
    else if (chan1_raw < chan1_raw_middle - PWM_OFFSET) value += delta;

    if (value != value_old) {
	EEPROM.write(address, value&0xff);
	EEPROM.write(address+1, (value>>8)&0xff);
    }
    return value;
}


// Setup change function
int change_val(int value, int address) {
    uint8_t value_old = value;
    
    if (chan1_raw > chan1_raw_middle + PWM_OFFSET) value--;
    else if (chan1_raw < chan1_raw_middle - PWM_OFFSET) value++;

    if (value != value_old) EEPROM.write(address, value);
    return value;
}


// Show those fancy 2 char arrows
#define ARROW_CODE		(0x90 - 2)		// code of the first MAX7456 special arrow char -2
void showArrow(uint8_t rotate_arrow) {
    rotate_arrow = (rotate_arrow < 2) ? 2 : rotate_arrow * 2;
    osd.printf("%c%c", ARROW_CODE + rotate_arrow, ARROW_CODE + rotate_arrow + 1);
}


#ifdef AH_ORIGINAL_VERSION

// Calculate and shows Artificial Horizon
void showHorizon(int start_col, int start_row) { 

    int x, nose, row, minval, hit, subval = 0;
    const int cols = 12;
    const int rows = 5;
    int col_hit[cols];
    float  pitch, roll;

    (abs(osd_pitch) == 90)?pitch = 89.99 * (90/osd_pitch) * -0.017453293:pitch = osd_pitch * -0.017453293;
    (abs(osd_roll) == 90)?roll = 89.99 * (90/osd_roll) * 0.017453293:roll = osd_roll * 0.017453293;

    nose = round(tan(pitch) * (rows*9));
    for(int col=1;col <= cols;col++){
        x = (col * 12) - (cols * 6) - 6;//center X point at middle of each col
        col_hit[col-1] = (tan(roll) * x) + nose + (rows*9) - 1;//calculating hit point on Y plus offset to eliminate negative values
        //col_hit[(col-1)] = nose + (rows * 9);
    }

    for(int col=0;col < cols; col++){
        hit = col_hit[col];
        if(hit > 0 && hit < (rows * 18)){
            row = rows - ((hit-1)/18);
            minval = rows*18 - row*18 + 1;
            subval = hit - minval;
            subval = round((subval*9)/18);
            if(subval == 0) subval = 1;
            printHit(start_col + col, start_row + row - 1, subval);
        }
    }
}

void printHit(byte col, byte row, byte subval){
    osd.openSingle(col, row);
    char subval_char;
        switch (subval){
        case 1:
            //osd.printf_P(PSTR("\x06"));
            subval_char = 0x06;
            break;
        case 2:
            //osd.printf_P(PSTR("\x07"));
            subval_char = 0x07; 
            break;
        case 3:
            //osd.printf_P(PSTR("\x08"));
            subval_char = 0x08;
            break;
        case 4:
            //osd.printf_P(PSTR("\x09"));
            subval_char = 0x09;
            break;
        case 5:
            //osd.printf_P(PSTR("\x0a"));
            subval_char = 0x0a; 
            break;
        case 6:
            //osd.printf_P(PSTR("\x0b"));
            subval_char = 0x0b;
            break;
        case 7:
            //osd.printf_P(PSTR("\x0c"));
            subval_char = 0x0c;
            break;
        case 8:
            //osd.printf_P(PSTR("\x0d"));
            subval_char = 0x0d;
            break;
        case 9:
            //osd.printf_P(PSTR("\x0e"));
            subval_char = 0x0e;
            break;
        }
        osd.printf("%c", subval_char);

}

#endif // AH_ORIGINAL_VERSION


#ifdef AH_REFACTORED_VERSION
							// with different factors we can adapt do different cam optics
#define AH_PITCH_FACTOR		0.017453293		// conversion factor for pitch
#define AH_ROLL_FACTOR		0.017453293		// conversion factor for roll
#define AH_COLS			12			// number of artificial horizon columns
#define AH_ROWS			5			// number of artificial horizon rows
#define CHAR_COLS		12			// number of MAX7456 char columns
#define CHAR_ROWS		18			// number of MAX7456 char rows
#define CHAR_SPECIAL		9			// number of MAX7456 special chars for the artificial horizon
#define LINE_CODE		(0x06 - 1)		// code of the first MAX7456 special char -1
#define AH_TOTAL_LINES		AH_ROWS * CHAR_ROWS
#define AH_SPECIAL_LINES	AH_ROWS * CHAR_SPECIAL

// Calculate and show artificial horizon
void showHorizon(int start_col, int start_row) {
    int col, row, pitch_line, middle, hit, subval;
    
    pitch_line = round(tan(-AH_PITCH_FACTOR * osd_pitch) * AH_SPECIAL_LINES);
    for (col=1; col<=AH_COLS; col++) {
        middle = col * CHAR_COLS - (AH_COLS * CHAR_COLS/2) - CHAR_COLS/2;			// center X point at middle of each column
        hit = tan(AH_ROLL_FACTOR * osd_roll) * middle + pitch_line + AH_SPECIAL_LINES - 1;	// calculating hit point on Y plus offset to eliminate negative values
        if (hit > 0 && hit < AH_TOTAL_LINES) {
            row = AH_ROWS - ((hit - 1) / CHAR_ROWS);
            subval = hit - (AH_TOTAL_LINES - row * CHAR_ROWS + 1);
            subval = round(subval * CHAR_SPECIAL / CHAR_ROWS);
            if (subval == 0) subval = 1;
            osd.openSingle(start_col + col - 1, start_row + row - 1);
            osd.printf("%c", LINE_CODE + subval);
        }
    }
}

#endif // AH_REFACTORED_VERSION


#ifdef AH_ZERO_CENTERED

// For using this, you must load a special mcm file where the artificial horizon zero left/right arrows are centered!
// e.g. AH_ZeroCentered002.mcm
							// with different factors we can adapt do different cam optics
#define AH_PITCH_FACTOR		0.010471976		// conversion factor for pitch
#define AH_ROLL_FACTOR		0.017453293		// conversion factor for roll
#define AH_COLS			12			// number of artificial horizon columns
#define AH_ROWS			5			// number of artificial horizon rows
#define CHAR_COLS		12			// number of MAX7456 char columns
#define CHAR_ROWS		18			// number of MAX7456 char rows
#define CHAR_SPECIAL		9			// number of MAX7456 special chars for the artificial horizon
#define LINE_CODE		(0x06 - 1)		// code of the first MAX7456 special char -1
#define AH_TOTAL_LINES		AH_ROWS * CHAR_ROWS

// Calculate and show artificial horizon
// used formula: y = m * x + n <=> y = tan(a) * x + n
void showHorizon(int start_col, int start_row) {
    int col, row, pitch_line, middle, hit, subval;
    
    pitch_line = round(tan(-AH_PITCH_FACTOR * osd_pitch) * AH_TOTAL_LINES) + AH_TOTAL_LINES/2;	// 90 total lines
    for (col=1; col<=AH_COLS; col++) {
        middle = col * CHAR_COLS - (AH_COLS/2 * CHAR_COLS) - CHAR_COLS/2;	  // -66 to +66	center X point at middle of each column
        hit = tan(AH_ROLL_FACTOR * osd_roll) * middle + pitch_line;	          // 1 to 90	calculating hit point on Y plus offset
        if (hit >= 1 && hit <= AH_TOTAL_LINES) {
	    row = (hit-1) / CHAR_ROWS;						  // 1 to 5 bottom-up
	    subval = (hit - (row * CHAR_ROWS) + 1) / (CHAR_ROWS / CHAR_SPECIAL);  // 1 to 9
            osd.openSingle(start_col + col - 1, start_row + AH_ROWS - row - 1);
            osd.printf("%c", LINE_CODE + subval);
        }
    }
}

#endif // AH_ZERO_CENTERED


#ifdef AH_BETTER_RESOLUTION

// For using this, you must load a special mcm file with the new staggered artificial horizon chars!
// e.g. AH_BetterResolutionCharset002.mcm
							// with different factors we can adapt do different cam optics
#define AH_PITCH_FACTOR		0.010471976		// conversion factor for pitch
#define AH_ROLL_FACTOR		0.017453293		// conversion factor for roll
#define AH_COLS			12			// number of artificial horizon columns
#define AH_ROWS			5			// number of artificial horizon rows
#define CHAR_COLS		12			// number of MAX7456 char columns
#define CHAR_ROWS		18			// number of MAX7456 char rows
#define CHAR_SPECIAL		9			// number of MAX7456 special chars for the artificial horizon
#define AH_TOTAL_LINES		AH_ROWS * CHAR_ROWS	// helper define

#define LINE_SET_STRAIGHT__	(0x06 - 1)		// code of the first MAX7456 straight char -1
#define LINE_SET_STRAIGHT_O	(0x3B - 3)		// code of the first MAX7456 straight overflow char -3
#define LINE_SET_P___STAG_1	(0x3C - 1)		// code of the first MAX7456 positive staggered set 1 char -1
#define LINE_SET_P___STAG_2	(0x45 - 1)		// code of the first MAX7456 positive staggered set 2 char -1
#define LINE_SET_N___STAG_1	(0x4E - 1)		// code of the first MAX7456 negative staggered set 1 char -1
#define LINE_SET_N___STAG_2	(0x57 - 1)		// code of the first MAX7456 negative staggered set 2 char -1
#define LINE_SET_P_O_STAG_1	(0xD4 - 2)		// code of the first MAX7456 positive overflow staggered set 1 char -2
#define LINE_SET_P_O_STAG_2	(0xDA - 1)		// code of the first MAX7456 positive overflow staggered set 2 char -1
#define LINE_SET_N_O_STAG_1	(0xD6 - 2)		// code of the first MAX7456 negative overflow staggered set 1 char -2
#define LINE_SET_N_O_STAG_2	(0xDD - 1)		// code of the first MAX7456 negative overflow staggered set 2 char -1

#define OVERFLOW_CHAR_OFFSET	6			// offset for the overflow subvals

#define ANGLE_1			9			// angle above we switch to line set 1
#define ANGLE_2			25			// angle above we switch to line set 2

// Calculate and show artificial horizon
// used formula: y = m * x + n <=> y = tan(a) * x + n
void showHorizon(int start_col, int start_row) {
    int col, row, pitch_line, middle, hit, subval;
    int roll;
    int line_set = LINE_SET_STRAIGHT__;
    int line_set_overflow = LINE_SET_STRAIGHT_O;
    int subval_overflow = 9;
    
    // preset the line char attributes
    roll = osd_roll;
    if ((roll >= 0 && roll < 90) || (roll >= -179 && roll < -90)) {	// positive angle line chars
	roll = roll < 0 ? roll + 179 : roll;
        if (abs(roll) > ANGLE_2) {
	    line_set = LINE_SET_P___STAG_2;
	    line_set_overflow = LINE_SET_P_O_STAG_2;
            subval_overflow = 7;
	} else if (abs(roll) > ANGLE_1) {
	    line_set = LINE_SET_P___STAG_1;
	    line_set_overflow = LINE_SET_P_O_STAG_1;
            subval_overflow = 8;
	}
    } else {								// negative angle line chars
	roll = roll > 90 ? roll - 179 : roll;
        if (abs(roll) > ANGLE_2) {
	    line_set = LINE_SET_N___STAG_2;
	    line_set_overflow = LINE_SET_N_O_STAG_2;
            subval_overflow = 7;
	} else if (abs(roll) > ANGLE_1) {
	    line_set = LINE_SET_N___STAG_1;
	    line_set_overflow = LINE_SET_N_O_STAG_1;
            subval_overflow = 8;
	}
    }
    
    pitch_line = round(tan(-AH_PITCH_FACTOR * osd_pitch) * AH_TOTAL_LINES) + AH_TOTAL_LINES/2;	// 90 total lines
    for (col=1; col<=AH_COLS; col++) {
        middle = col * CHAR_COLS - (AH_COLS/2 * CHAR_COLS) - CHAR_COLS/2;	  // -66 to +66	center X point at middle of each column
        hit = tan(AH_ROLL_FACTOR * osd_roll) * middle + pitch_line;	          // 1 to 90	calculating hit point on Y plus offset
        if (hit >= 1 && hit <= AH_TOTAL_LINES) {
	    row = (hit-1) / CHAR_ROWS;						  // 0 to 4 bottom-up
	    subval = (hit - (row * CHAR_ROWS) + 1) / (CHAR_ROWS / CHAR_SPECIAL);  // 1 to 9
	    
	    // print the line char
            osd.openSingle(start_col + col - 1, start_row + AH_ROWS - row - 1);
            osd.printf("%c", line_set + subval);
	    
	    // check if we have to print an overflow line char
	    if (subval >= subval_overflow && row < 4) {	// only if it is a char which needs overflow and if it is not the upper most row
                osd.openSingle(start_col + col - 1, start_row + AH_ROWS - row - 2);
                osd.printf("%c", line_set_overflow + subval - OVERFLOW_CHAR_OFFSET);
	    }
        }
    }
}

#endif // AH_BETTER_RESOLUTION


void set_converts() {
    if (EEPROM.read(measure_ADDR) == 0) {
        convert_speed = 3.6;
        convert_length = 1.0;
        convert_length_large = 1000;
        unit_speed = 0x81;
        unit_length = 0x8D;
        unit_length_large = 0xFD;
    } else {
        convert_speed = 2.23;
        convert_length = 3.28;
        convert_length_large = 5280;
        unit_speed = 0xfb;
        unit_length = 0x66;
        unit_length_large = 0xFA;
    }
}
