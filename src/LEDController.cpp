#include "LEDController.h"

float dim = 100;  // variable global para dim
CRGB LEDController::leds[NUM_ARMS][NUM_LEDS_PER_STRIP];

uint8_t ap_sat_visible;  // Number of visible Satelites
int state_GPS;
int state_FLASH;
unsigned long currentmillis;       // current milliseconds
unsigned long lastmillis;          // milliseconds of last loop
unsigned long targetmillis_FLASH;  // target milliseconds
unsigned long targetmillis_GPS;
uint8_t ap_fixtype;


bool isBlinking = false; // Variable de estado para indicar si el LED está parpadeando
unsigned long previousMillis = 0; // Variable para almacenar el tiempo anterior del parpadeo
int ledState = LOW; // Definición de la variable ledState


void LEDController::setup() {
 // Configurar el número de LEDs y el tipo de tira LED APA102

  FastLED.addLeds<LEDTYPE, DATA_PIN, CLOCK_PIN, RGBORDER>(leds[0], 140); // wemos32
  FastLED.addLeds<LEDTYPE, DATA_PIN2, CLOCK_PIN2, RGBORDER>(leds[1], 140); // wemos32
  // Ajustar el brillo de la tira LED (opcional)
  FastLED.setBrightness(100);
  }

void LEDController::updateHeartbeat(bool mavlinkConnected) {
  // Si Mavlink está conectado, hacer que el LED de heartbeat titile una vez por segundo
  if (mavlinkConnected) {
    startBlinking(0, 0);

    static bool heartbeatState = false;
    static unsigned long lastHeartbeatTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastHeartbeatTime >= 500) {
      heartbeatState = !heartbeatState;
      lastHeartbeatTime = currentTime;
      }
    if (heartbeatState) {
      //leds[0][0] = CRGB::White; // LED 0 - Ala izquierda
      //leds[1][0] = CRGB::White; // LED 0 - Ala izquierda
      }
    else {
      //leds[0][0] = CRGB::Black; // Apagar LED 0 - Ala izquierda
     // leds[1][0] = CRGB::Black; // Apagar LED 0 - Ala izquierda
      }
    }
  else {
    // Si no hay comunicación, detener el parpadeo del LED
    stopBlinking(0, 0);

    //leds[0][0] = CRGB::Black; // Apagar LED 0 - Ala izquierda
    //leds[1][0] = CRGB::Black; // Apagar LED 0 - Ala izquierda
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
      for (int j = 0; j < NUM_LEDS_PER_ARM; j++) {
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
      for (int j = 0; j < NUM_LEDS_PER_ARM; j++) {
        if (LED_DEF[i][j] == FRONT) {
          leds[i][j] = CHSV(96, 255, 255 * dim1);
          }
        }
      }
    }
  else {
    for (int i = 0; i < NUM_ARMS; i++) {
      for (int j = 0; j < NUM_LEDS_PER_ARM; j++) {
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
  for (int i = 0;i < NUM_ARMS;i++) {
    for (int j = 0; j < 140; j++) {
      leds[i][j] = CHSV(0, 0, 0);
      FastLED.show();
      }
    for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
      leds[i][j] = CHSV(0, 0, j);
      FastLED.show();
      FastLED.delay(20);
      }
    for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
      leds[i][j] = CHSV(0, j, 0);
      FastLED.show();
      FastLED.delay(20);
      }
    for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
      leds[i][j] = CRGB(j, 0, 0);
      FastLED.show();
      FastLED.delay(20);
      }
    }
  FastLED.clear();
  FastLED.show();
  Serial.println("FIN Leds_Test");
  } // fin test de leds

  // #####################################################################################################
  // ### GET GPS STATUS FROM ARDUPILOT AND PRINT CORESPONDING LED(s) ###
  // #####################################################################################################
void LEDController::get_gps_status(int STATUS, float dim) {
  /* from MavLink_FrSkySPort.ino
   *  ap_sat_visible  => numbers of visible satellites
   *  ap_fixtype    => 0 = No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
   */

  CRGB COLOR;
  int FREQ;
  if (STATUS == 1) {
    // Serial.print(ap_fixtype);
    switch (ap_fixtype) {
        case NO_GPS:  // blink red
          COLOR = CHSV(0, 255, dim);
          FREQ = 750;
          break;
        case NO_FIX:  // blink orange
          COLOR = CHSV(39, 255, dim);
          FREQ = 600;
          break;
        case GPS_OK_FIX_2D:  // blue
          COLOR = CHSV(160, 255, dim);
          FREQ = 500;
          break;
        case GPS_OK_FIX_3D:  // const blue
          COLOR = CHSV(160, 255, dim);
          FREQ = 0;
          state_GPS = 0;
          break;
        case GPS_OK_FIX_3D_DGPS:  // const blue
          COLOR = CHSV(160, 255, dim);
          FREQ = 0;
          state_GPS = 0;
          break;
        default:  // off
          COLOR = CHSV(0, 0, 0);
          FREQ = 0;
      }
    if (currentmillis >= targetmillis_GPS) {
      if (state_GPS == 0) {
        for (int i = 0; i < NUM_ARMS; i++) {
          for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
            if (LED_DEF[i][j] == GPS) {
              leds[i][j] = COLOR;
              }
            }
          }
        state_GPS = 1;
        }
      else {
        for (int i = 0; i < NUM_ARMS; i++) {
          for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
            if (LED_DEF[i][j] == GPS) {
              leds[i][j] = CHSV(0, 0, 0);
              }
            }
          }
        state_GPS = 0;
        }
      targetmillis_GPS = lastmillis + FREQ;
      }
    }
  else {
    for (int i = 0; i < NUM_ARMS; i++) {
      for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
        if (LED_DEF[i][j] == GPS) {
          leds[i][j] = CHSV(0, 0, 0);
          }
        }
      }
    }
  }


//  para blink 

void LEDController::startBlinking(int row, int col) {
  // Iniciar el parpadeo del LED si no está parpadeando
  Serial.println("startBlinking called");
  if (!isBlinking) {
    isBlinking = true;
    previousMillis = millis(); // Reiniciar el tiempo anterior del parpadeo
    Serial.println("Blinking started");
    }

    // Llamar a la función blinkLED para mantener el parpadeo
  blinkLED(row, col, 250, 750); // Parpadeo de 250 ms en ON y 750 ms en OFF
  }


void LEDController::stopBlinking(int row, int col) {
  // Detener el parpadeo del LED
  isBlinking = false;
  // Lógica adicional si es necesario para asegurar que el LED esté apagado
  leds[0][0] = CRGB::Black;  //  cambiar detectando los leds de la funcion HB 
  FastLED.show();
  }

void LEDController::blinkLED(int row, int col, unsigned long onDuration, unsigned long offDuration) {
  unsigned long currentMillis = millis();

    // Mensajes de depuración para las variables clave
  Serial.print("currentMillis: ");
  Serial.println(currentMillis);
  Serial.print("previousMillis: ");
  Serial.println(previousMillis);
  Serial.print("ledState: ");
  Serial.println(ledState == LOW ? "LOW" : "HIGH");
  Serial.print("offDuration: ");
  Serial.println(offDuration);
  Serial.print("onDuration: ");
  Serial.println(onDuration);
  Serial.print("isBlinking: ");
  Serial.println(isBlinking);

  if ((ledState == LOW && ((currentMillis - previousMillis) >= offDuration)) ||
    (ledState == HIGH && ((currentMillis - previousMillis) >= onDuration))) {
    previousMillis = currentMillis;
    Serial.print("isBlinking: ");
    Serial.println(isBlinking);
    // Cambiar el estado del LED solo si está parpadeando
    if (isBlinking) {
      ledState = (ledState == LOW) ? HIGH : LOW;

      // Escribir el estado del LED
      leds[0][0] = (ledState == LOW) ? CRGB::Black : CRGB::White; //  cambiar detectando los leds de la funcion HB 
      FastLED.show();
      Serial.print("LED state: ");
      Serial.println(ledState == LOW ? "OFF" : "ON");
      }
    }
  }