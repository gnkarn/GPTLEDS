#ifndef LEDCONTROLLER_H
#define LEDCONTROLLER_H

#include <Arduino.h>
#include <FastLED.h>
#include <modes.h>

#define RGBORDER GRB  // BRG para 2811, GRB para 2812
// #define LEDTYPE WS2812B
#define LEDTYPE APA102
#define NUM_ARMS 2 //4
#define NUM_LEDS_PER_STRIP 7  // 10
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

const byte LED_DEF[NUM_ARMS][NUM_LEDS_PER_STRIP] = {
    {HB,GPS, FRONT, FRONT, FRONT, FRONT,FLASH},
    {HB,GPS, FRONT, FRONT, FRONT, FRONT,FLASH}
  };


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

  //rear_arms(STATUS, dim);
  //flash_pos_light(STATUS, dim);
  // get_armed_status(ON, dim);
  // get_gps_status(ON, dim);
  private:
  static CRGB leds[NUM_ARMS][NUM_LEDS_PER_STRIP]; // Arreglo para almacenar los LEDs

  };

#endif
