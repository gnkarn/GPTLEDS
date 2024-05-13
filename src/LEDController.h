#ifndef LEDCONTROLLER_H
#define LEDCONTROLLER_H

#include <Arduino.h>
#include <FastLED.h>
#include <modes.h>

#define RGBORDER GRB  // BRG para 2811, GRB para 2812
// #define LEDTYPE WS2812B
#define LEDTYPE APA102
#define NUM_ARMS 2 //4
#define NUM_LEDS_PER_STRIP 140  // 10
#define FRAMES_PER_SECOND 30
#define NUM_LEDS_PER_ARM 10 // 10 , cambiar la definicion de  LED_DEF
#define  NUM_LEDS  NUM_LEDS_PER_ARM * NUM_ARMS  // NUM_LEDS_PER_ARM * NUM_ARMS; // Number of LED's
// CRGB Vir_led[NUM_LEDS]     ;                      // simula una tira continua
// con todos los leds de cada brazo conectados en serie
#define BRIGHTNESS 50 // setear en 96 , 50 para debug

#define DATA_PIN 23 // wemos32 para tira 1
#define CLOCK_PIN 19
#define DATA_PIN2 13 // wemos32 para tira 2
#define CLOCK_PIN2 12




// #####################################################################################################
// ### LED VARIABLES ###
// #####################################################################################################
#define HB 0 // led asignado para heart bit
#define GPS 1
#define ARMED 2
#define FRONT 3
#define BACK 4
#define SIDE 5
#define FLASH 6

const byte LED_DEF[NUM_ARMS][NUM_LEDS_PER_ARM] = {
    {HB,GPS, GPS, FRONT, FRONT, FRONT,FLASH},
    {HB,GPS, GPS, FRONT, FRONT, FRONT,FLASH}
  };

/*
 * *******************************************************
 * *** Message #24  GPS_RAW_INT                        ***
 * *******************************************************
 */

  // Some applications will not use the value of this field unless it is
  // at least two, so always correctly fill in the fix.
extern uint8_t ap_sat_visible;  // Number of visible Satelites
extern int state_GPS;
extern int state_FLASH;
extern unsigned long currentmillis;       // current milliseconds
extern unsigned long lastmillis;          // milliseconds of last loop
extern unsigned long targetmillis_FLASH;  // target milliseconds
extern unsigned long targetmillis_GPS;

// #####################################################################################################
// ### DEFAULT VARIABLES ###
// #####################################################################################################
// antes en led_control.ino
#define RIGHT 0
#define LEFT 1
#define OFF 0
#define ON 1
#define AUTO 2
#define BLINK 3

/// GPS status codes
enum GPS_Status {
  NO_GPS = 0,              ///< No GPS connected/detected
  NO_FIX = 1,              ///< Receiving valid GPS messages but no lock
  GPS_OK_FIX_2D = 2,       ///< Receiving valid messages and 2D lock
  GPS_OK_FIX_3D = 3,       ///< Receiving valid messages and 3D lock
  GPS_OK_FIX_3D_DGPS = 4,  ///< Receiving valid messages and 3D lock with ///< differential improvements
  GPS_OK_FIX_3D_RTK = 5,   ///< Receiving valid messages and 3D lock, with ///< relative-positioning improvements
  };
extern uint8_t ap_fixtype;  // 0 = No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix, 4 = DGPS, 5 = RTK.

class LEDController {
  public:
  static void setup(); // Función para inicializar la tira LED APA102
  static void updateHeartbeat(bool mavlinkConnected); // Función para actualizar el LED de heartbeat
  static void updateFlightMode(int flightMode); // Función para actualizar el estado del modo de vuelo
  static void updateGPSStatus(bool has3DFix); // Función para actualizar el estado del GPS 3Dfix
  static void setLED(int arm, int index, CRGB color); // Función para establecer el color de un LED específico
  static void default_mode(int STATUS, float dim1);
  static void front_arms(int STATUS, float dim1);
  static void Leds_Test(void);
  static void blinkLED();
  static void get_gps_status(int STATUS, float dim);
  
  static void startBlinking(int row, int col); // Declaración de la función startBlinking
  static void stopBlinking(int row, int col); // Declaración de la función stopBlinking
  static void blinkLED(int row, int col, unsigned long onDuration, unsigned long offDuration); // Declaración de la función blinkLED

  //rear_arms(STATUS, dim);
  //flash_pos_light(STATUS, dim);
  // get_armed_status(ON, dim);
  // get_gps_status(ON, dim);
  private:
  static CRGB leds[NUM_ARMS][NUM_LEDS_PER_STRIP]; // Arreglo para almacenar los LEDs


  };

#endif
