#ifndef MAVLINKHANDLER_H
#define MAVLINKHANDLER_H

#include <Arduino.h>
// #include <mavlink.h>
#include <GCS_MAVLink.h> 
#include <Mavlink_ino.h>


class MavlinkHandler {

  private:
  static bool messageReceived; // Variable para indicar si se ha recibido un mensaje
  static unsigned long lastMessageTime; // Variable para almacenar el tiempo del último mensaje recibido
  static const unsigned long TIMEOUT_MS; // Tiempo de espera en milisegundos antes de considerar que no se han recibido mensajes
 

  static void processHeartbeat(mavlink_message_t message); // Función para procesar el mensaje de latido
  static void processGPSStatus(mavlink_message_t message); // Función para procesar el estado del GPS
  static void processSysStatus(mavlink_message_t message); // Función para procesar el estado del sistema
      // Otras declaraciones de funciones y variables...
  public:
  static void receiveMessages(); // Función para recibir mensajes MAVLink
  static void decodeMessage(mavlink_message_t message); // Función para decodificar mensajes MAVLink
  };

#endif