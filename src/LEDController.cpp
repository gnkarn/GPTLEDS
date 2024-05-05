#include "LEDController.h"

float dim = 100;  // variable global para dim
CRGB LEDController::leds[NUM_ARMS][NUM_LEDS_PER_STRIP];

void LEDController::setup() {
  // Configurar el número de LEDs y el tipo de tira LED APA102

  FastLED.addLeds<LEDTYPE, DATA_PIN, CLOCK_PIN, RGBORDER>(leds[0], NUM_LEDS); // wemos32
  FastLED.addLeds<LEDTYPE, DATA_PIN2, CLOCK_PIN2, RGBORDER>(leds[1], NUM_LEDS); // wemos32
  // Ajustar el brillo de la tira LED (opcional)
  FastLED.setBrightness(100);
  }

void LEDController::updateHeartbeat(bool mavlinkConnected) {
  // Si Mavlink está conectado, hacer que el LED de heartbeat titile una vez por segundo
  if (mavlinkConnected) {
    static bool heartbeatState = false;
    static unsigned long lastHeartbeatTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastHeartbeatTime >= 500) {
      heartbeatState = !heartbeatState;
      lastHeartbeatTime = currentTime;
      }
    if (heartbeatState) {
      leds[0][0] = CRGB::White; // LED 0 - Ala izquierda
      leds[1][0] = CRGB::White; // LED 0 - Ala izquierda
      }
    else {
      leds[0][0] = CRGB::Black; // Apagar LED 0 - Ala izquierda
      leds[1][0] = CRGB::Black; // Apagar LED 0 - Ala izquierda
      }
    }
  else {
    leds[0][0] = CRGB::Black; // Apagar LED 0 - Ala izquierda
    leds[1][0] = CRGB::Black; // Apagar LED 0 - Ala izquierda
    }

    // Mostrar los cambios en la tira LED
  FastLED.show();
  }

void LEDController::updateFlightMode(int flightMode) {
  // Apagar todos los LEDs
  FastLED.clear();

  // Asignar los modos de vuelo a los LEDs correspondientes
  switch (flightMode) {
      case AUTO:
        leds[0][1] = CRGB::Green; // Modo AUTO - Ala izquierda
        leds[1][1] = CRGB::Green; // Modo AUTO - Ala derecha
        break;
      case MANUAL:
        leds[0][2] = CRGB::Green; // Modo MANUAL - Ala izquierda
        leds[1][2] = CRGB::Green; // Modo MANUAL - Ala derecha
        break;
      case RTL:
        leds[0][3] = CRGB::Green; // Modo RTL - Ala izquierda
        leds[1][3] = CRGB::Green; // Modo RTL - Ala derecha
        break;
      // Agregar más casos según sea necesario para otros modos de vuelo
    }

    // Mostrar los cambios en la tira LED
  FastLED.show();
  }

void LEDController::updateGPSStatus(bool has3DFix) {
  // Actualizar el LED correspondiente para indicar si hay señal 3Dfix
  if (has3DFix) {
    leds[0][2] = CRGB::Blue; // LED 2 - Ala izquierda
    leds[1][2] = CRGB::Blue; // LED 2 - Ala derecha
    }
  else {
    leds[0][2] = CRGB::Black; // Apagar LED 2 - Ala izquierda
    leds[1][2] = CRGB::Black; // Apagar LED 2 - Ala derecha
    }
    // Mostrar los cambios en la tira LED
  FastLED.show();
  }

void LEDController::setLED(int arm, int index, CRGB color) {
  if (arm >= 0 && arm < NUM_ARMS && index >= 0 && index < NUM_LEDS_PER_ARM) {
    for (int i = 0; i < NUM_ARMS; i++) {
      for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
        if (LED_DEF[i][j] == ARMED) {
          leds[i][j] = CHSV(0, 255, dim);
          }
        }
      FastLED.show(); // Mostrar los cambios en la tira LED
      }
    }
  }


  //#####################################################################################################
//### SETUP DEFAULT MODE FOR LED LIGHTNING - SIDEARMS, FRONT-, BREAK-, LANDING-LIGHT always off     ###
//#####################################################################################################
void LEDController::default_mode(int STATUS, float dim1) {
  front_arms(STATUS, dim1); // uso una copia de dim en otra variable , debo pasar dim a esta funcion
  //rear_arms(STATUS, dim);
  //flash_pos_light(STATUS, dim);
 // get_armed_status(ON, dim);
 // get_gps_status(ON, dim);
  }


//#####################################################################################################
//### DEFAULT FRONT ARM LED MODE (CONST-Green)                                                        ###
//#####################################################################################################
void LEDController::front_arms(int STATUS, float dim1) {
  if (STATUS == 1) {
    for (int i = 0; i < NUM_ARMS; i++) {
      for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
        if (LED_DEF[i][j] == FRONT) {
          leds[i][j] = CHSV(96, 255, 255 * dim1);
          }
        }
      }
    }
  else {
    for (int i = 0; i < NUM_ARMS; i++) {
      for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
        if (LED_DEF[i][j] == FRONT) {
          leds[i][j] = CHSV(0, 0, 0);
          }
        }
      }
    }
  }

// Método para hacer parpadear el LED
void LEDController::blinkLED() {
  static unsigned long lastChange = 0;
  static bool isOn = false;
  unsigned long currentTime = millis();

  // Intervalo de 1 segundo para el parpadeo (250 ms encendido, 750 ms apagado)
  if (currentTime - lastChange >= 1000) {
    lastChange = currentTime;
    if (isOn) {
      leds[0][0] = CRGB::Black; // Apagar el LED
      isOn = false;
      }
    else {
      leds[0][0] = CRGB::White; // Encender el LED
      isOn = true;
      }
    }
  else if (isOn && currentTime - lastChange >= 250) {
    leds[0][0] = CRGB::Black; // Apagar el LED después de 250 ms
    isOn = false;
    }
  }


  // test de leds
void LEDController::Leds_Test(void) {
  Serial.println("Leds_Test apagaTodo");
  for (int j = 0;j < NUM_ARMS;j++) {
    for (int i = 0; i < 140; i++) {
      leds[i][j] = CHSV(0, 0, 0);
      FastLED.show();
      }
    for (int i = 0; i < 10; i++) {
      leds[i][j] = CHSV(0, 0, i);
      FastLED.show();
      FastLED.delay(20);
      }
    for (int i = 0; i < 10; i++) {
      leds[i][j] = CHSV(0, i, 0);
      FastLED.show();
      FastLED.delay(20);
      }
    for (int i = 0; i < 10; i++) {
      leds[i][j] = CRGB(i, 0, 0);
      FastLED.show();
      FastLED.delay(20);
      }
    }
  FastLED.clear(1);
  FastLED.show();
  Serial.println("FIN Leds_Test");
  } // fin test de leds