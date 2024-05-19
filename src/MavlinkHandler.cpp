#include "MavlinkHandler.h"
#include "LEDController.h"
#include "Mavlink_ino.h" // Incluir Mavlink_ino.h aquí

#define MAVLINK_DEBUG
#define DEBUG_APM_GPS_RAW
int G_flightMode = MANUAL;
#define debugSerial Serial



// Inicializar las variables estáticas en el ámbito de la clase
bool MavlinkHandler::messageReceived = false;
unsigned long MavlinkHandler::lastMessageTime = 0;
const unsigned long MavlinkHandler::TIMEOUT_MS = 5000; // Tiempo de espera predeterminado de 5 segundos

unsigned long MavlinkHandler::lastCommunicationTime = 0; // Definición de la variable lastCommunicationTime

void MavlinkHandler::receiveMessages() {
  bool messageReceived = false; // Variable para indicar si se ha recibido un mensaje
  while (Serial2.available() > 0) {
    uint8_t byte = Serial2.read();
    mavlink_message_t message;
    mavlink_status_t status;
    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &message, &status)) {
      messageReceived = true; // Indicar que se ha recibido un mensaje
      decodeMessage(message);
      }
    }
    // Actualizar el tiempo de la última comunicación recibida
  if (messageReceived) {
    lastCommunicationTime = millis();
    }


    // Verificar si no se ha recibido ningún mensaje durante más de 5 segundos
  if (!messageReceived && millis() - lastCommunicationTime > TIMEOUT_MS) {
    // Detener el parpadeo del LED
    LEDController::stopBlinking(0, 0);
    messageReceived = false;
    MavLink_Connected = false;
    }
  }

void MavlinkHandler::decodeMessage(mavlink_message_t message) {
  switch (message.msgid) {
      case MAVLINK_MSG_ID_HEARTBEAT: //  #0  https://mavlink.io/en/messages/common.html#HEARTBEAT
        mavlink_heartbeat_t hb;
        static int ledState;
        MavLink_Connected = 1;
      // mavlink_msg_heartbeat_decode(&msg, &hb);
        processHeartbeat(message);
            // Some more actions to execute to show loop() is running...
      // Non blocking LED toggler
        if ((millis() % 2000) > 200) ledState = 1;
        else                         ledState = 0;
        digitalWrite(LED_BUILTIN, ledState);
        Serial.print("LED_BUILTIN: ");
        Serial.println(ledState);
#ifdef MAVLINK_DEBUG
        Serial.print("=HEARTBEAT");
        Serial.print(" Type:");
        Serial.print(hb.type);
        Serial.print(" APclass:");
        Serial.print(hb.autopilot);
        Serial.print(" BaseMode:");
        Serial.print(hb.base_mode);
        if ((hb.base_mode == 1) || (hb.base_mode == 65))  // https://ardupilot.org/rover/docs/parameters.html#mode1
          Serial.print(" DISarmed");
        if ((hb.base_mode == 129) || (hb.base_mode == 193))
          Serial.print(" !!ARMED!!");
        Serial.print(" Custom/Flightmode:");
        Serial.print(hb.custom_mode);
        if (hb.custom_mode == 0)
          Serial.print(" MANUAL");
        if (hb.custom_mode == 4)
          Serial.print(" HOLD");
        if (hb.custom_mode == 10)
          Serial.print(" !!AUTO!!");
        Serial.print(" SysStat:");
        Serial.print(hb.system_status);
        Serial.print(" MavVer:");
        Serial.print(hb.mavlink_version);
#endif
        break;
      case MAVLINK_MSG_ID_GPS_STATUS:
        processGPSStatus(message);
        break;
      case MAVLINK_MSG_ID_SYS_STATUS:
        processSysStatus(message);
        break;
      // Agregar más casos según sea necesario para otros tipos de mensajes MAVLink

    }
  // Restablecer el estado de messageReceived después de un cierto tiempo sin recibir mensajes
// Por ejemplo, si no se recibe ningún mensaje durante 1 segundo, se restablece messageReceived a false
/*  if (millis() - lastMessageTime > TIMEOUT_MS) {
    messageReceived = false;
    MavLink_Connected = false;
    }
*/
  }


void MavlinkHandler::processHeartbeat(mavlink_message_t message) {

  lastMessageTime = millis();
     // Actualizar la variable messageReceived cuando se recibe un mensaje
  //MavlinkHandler::messageReceived = true;

/*
  if (messageReceived) {

    static unsigned long lastHeartbeatTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastHeartbeatTime >= 5000) {
      static bool heartbeatState = false;
      heartbeatState = !heartbeatState;
      lastHeartbeatTime = currentTime;
      // MavLink_Connected = heartbeatState; // Estas dos variables funcioinan igual , ver de eliminar una

      if (heartbeatState) {
        // LEDController::setLED(0, 0, CRGB::White);//
        }
      else {
                    // Apagar el LED si no está en estado de latido
        LEDController::setLED(0, 0, CRGB::Black);
        }
      }
*/
  mavlink_heartbeat_t heartbeat;
  mavlink_msg_heartbeat_decode(&message, &heartbeat);
  // Aquí puedes actualizar la variable flightMode con el estado del modo de vuelo leído del mensaje heartbeat
  G_flightMode = heartbeat.custom_mode;
  //}
  }


// **** MAVLINK Message #24 - GPS_RAW_INT ***

void MavlinkHandler::processGPSStatus(mavlink_message_t message) {
  mavlink_gps_raw_int_t gpsRaw;
  mavlink_msg_gps_raw_int_decode(&message, &gpsRaw);
  // Verificar si hay una señal 3Dfix
  ap_fixtype = mavlink_msg_gps_raw_int_get_fix_type(&message);  // 0 = No GPS, 1 =No Fix, 2 = 2D Fix, 3 = 3D Fix, 4 = DGPS, 5 = RTK
  ap_sat_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&message);          // numbers of visible satelites

  bool has3DFix = (gpsRaw.fix_type == 3);
  // Actualizar los LEDs correspondientes para indicar si hay señal 3Dfix
  if (has3DFix) {
    // LEDController::setLED(0, 2, CRGB::Blue); // LED 2 - Ala izquierda
    // LEDController::setLED(1, 2, CRGB::Blue); // LED 2 - Ala derecha
    }
  else {
    // LEDController::setLED(0, 2, CRGB::Black); // Apagar LED 2 - Ala izquierda
    // LEDController::setLED(1, 2, CRGB::Black); // Apagar LED 2 - Ala derecha
    }
  LEDController::get_gps_status(gpsRaw.fix_type, 200);
#ifdef DEBUG_APM_GPS_RAW
  debugSerial.print(millis());
  debugSerial.print("\tMAVLINK_MSG_ID_GPS_RAW_INT: fixtype: ");
  debugSerial.println(ap_fixtype);
  debugSerial.print(", visiblesats: ");
  debugSerial.println(ap_sat_visible);
  debugSerial.print(", status: ");
  //debugSerial.println(gps_status);
  //debugSerial.print(", gpsspeed: ");
  //debugSerial.print(mavlink_msg_gps_raw_int_get_vel(&msg)/100.0);
  //debugSerial.print(", hdop: ");
  //debugSerial.print(mavlink_msg_gps_raw_int_get_eph(&msg)/100.0);
  //debugSerial.print(", alt: ");
  //debugSerial.print(mavlink_msg_gps_raw_int_get_alt(&msg));
  debugSerial.println();
#endif
  }

void MavlinkHandler::processSysStatus(mavlink_message_t message) {
  mavlink_heartbeat_t heartbeat;
  mavlink_msg_heartbeat_decode(&message, &heartbeat);
    // Verificar si el sistema está ARMED o DESARMED
  bool isArmed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
  // Actualizar los LEDs correspondientes para indicar si el sistema está ARMED o DESARMED
  if (isArmed) {
    LEDController::setLED(0, 3, CRGB::Green);// LED 3 - Ala izquierda
    LEDController::setLED(1, 3, CRGB::Green);// LED 3 - Ala derecha

    }
  else {
    LEDController::setLED(0, 3, CRGB::Red);// LED 3 - Ala izquierda
    LEDController::setLED(1, 3, CRGB::Red);// LED 3 - Ala derecha
    }
   // Si Mavlink está conectado, hacemos que el LED de heartbeat titile una vez por segundo
  if (messageReceived) {
    static bool heartbeatState = false;
    static unsigned long lastHeartbeatTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastHeartbeatTime >= 500) {
      heartbeatState = !heartbeatState;
      lastHeartbeatTime = currentTime;
      }
    if (heartbeatState) {
      LEDController::setLED(0, 0, CRGB::White);// 
      }
    }

  }
