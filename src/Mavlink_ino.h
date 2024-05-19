// https://mavlink.io/en/messages/common.html#MAV_MODE_FLAG
// 

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




#endif 

