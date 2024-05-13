#include <Arduino.h>

// #include <stdint.h>
// Version 0.1
// 
#include "MavlinkHandler.h"
#include "LEDController.h"
// #include <modes.h>

#define RXD2 16  // para serial 2 en wemos 32 
#define TXD2 17

bool MavLink_Connected = 0;
bool test;
unsigned long lastCommunicationTime = 0; // Variable para almacenar el tiempo de la última comunicación recibida

void setup() {
  Serial.println("Setup de arduplane leds iniciado ");
  Serial.begin(115200);
  Serial2.begin(57600, SERIAL_8N1, RXD2, TXD2);   // Initialise Serial2 port at 57600bps
  pinMode(LED_BUILTIN, OUTPUT);
  LEDController::setup(); // Inicializar la tira LED APA102
  Serial.print("Test leds");
  LEDController::Leds_Test();
  Serial.print("Fin de Setup , inicio de Loop");
  delay(3000);
  }

void loop() {
  MavlinkHandler::receiveMessages(); // Recibir mensajes MAVLink
  LEDController::updateFlightMode(G_flightMode); // Actualizar el estado del modo de vuelo en la tira LED
  LEDController::get_gps_status(ON, 200);
  LEDController::updateHeartbeat(MavLink_Connected);  // Connected or Not);
  // Verificar si ha pasado más de 5 segundos desde la última comunicación
  if (millis() - lastCommunicationTime > 5000) {
    // Detener el parpadeo del LED (LED fijo)
    LEDController::stopBlinking(0, 0);
    }
  else {
 // Continuar el parpadeo del LED
    LEDController::updateHeartbeat(MavLink_Connected);
    }

  }