#include <Arduino.h>
// #include <stdint.h>
#include "MavlinkHandler.h"
#include "LEDController.h"
// #include <modes.h>

#define RXD2 16  // para serial 2 en wemos 32 
#define TXD2 17

void setup() {
  Serial.println("Setup de arduplane leds iniciado ");
  Serial.begin(115200);
  Serial2.begin(57600, SERIAL_8N1, RXD2, TXD2);   // Initialise Serial2 port at 57600bps
  LEDController::setup(); // Inicializar la tira LED APA102
  Serial.print("Test leds");
  LEDController::Leds_Test();
  Serial.print("Fin de Setup , inicio de Loop");
  delay(3000);
  }

void loop() {
  MavlinkHandler::receiveMessages(); // Recibir mensajes MAVLink
  LEDController::updateFlightMode(G_flightMode); // Actualizar el estado del modo de vuelo en la tira LED
  }