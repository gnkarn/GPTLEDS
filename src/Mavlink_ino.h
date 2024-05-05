#ifndef MAVLINKINO_H
#define MAVLINKINO_H

#include <stdint.h>
#include <modes.h>
/*
 * *******************************************************
 * *** Mavlink Definitions:                            ***
 * *******************************************************
 */
/*
 * *******************************************************
 * *** Message #0  HEARTHBEAT                          ***
 * *******************************************************
 */
// uint8_t     ap_type               =  0; //original comentado
// uint8_t     ap_autopilot          =  0;//original comentado

extern uint8_t ap_base_mode;
extern int32_t ap_custom_mode; // = MANUAL 

extern int G_flightMode;

// uint8_t     ap_system_status      =  0;//original comentado
extern uint8_t ap_mavlink_version;
extern uint8_t ap_system_status;
//#define minimum_battery_volts (3400 * 4) 



/*
 * *******************************************************
 * *** Message #24  GPS_RAW_INT                        ***
 * *******************************************************
 */
extern uint8_t ap_fixtype;  // 0 = No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix, 4 = DGPS, 5 = RTK.
    // Some applications will not use the value of this field unless it is
    // at least two, so always correctly fill in the fix.
extern uint8_t ap_sat_visible;  // Number of visible Satelites

#endif 

