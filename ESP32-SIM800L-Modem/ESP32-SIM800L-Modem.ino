/*
  ESP32-SIM800L-Modem.ino

  This is a MSPv2 Library, made for Bullet Ground Control Station System
  https://github.com/danarrib/BulletGCSS
  
  Written by Daniel Ribeiro - https://github.com/danarrib
  
* This program is free software: you can redistribute it and/or modify  
* it under the terms of the GNU General Public License as published by  
* the Free Software Foundation, version 3.
*
* This program is distributed in the hope that it will be useful, but 
* WITHOUT ANY WARRANTY; without even the implied warranty of 
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License 
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * SETTINGS FOR ARDUINO IDE
 * Board: DOIT ESP32 DEVKIT V1
 * Flash Frequency: 80MHz
 * Upload Speed: 921600
 * Core Debug Level: None
 */

#define LED 2
#define GPS_DEGREES_DIVIDER 10000000.0f
#define SERIAL_PIN_RX 23
#define SERIAL_PIN_TX 22
#define TASK_MSP_READ_MS 200
#define MSP_PORT_RECOVERY_THRESHOLD (TASK_MSP_READ_MS * 5)

#define WP_MISSION_MESSAGE_INTERVAL 30
#define HOME_POINT_FETCH_INTERVAL 10

#include <WiFi.h>
#include <PubSubClient.h>
#include "Config.h"
#include <HardwareSerial.h>
#include "msp_library.h"
#include "uav_status.h"

HardwareSerial mspSerial(2);

WiFiClient espClient;
PubSubClient client(espClient);
MSPLibrary msp;
uav_status uavstatus;
uav_status lastStatus;
msp_set_wp_t currentWPMission[255];

uint32_t lastMessageTimer = 0;  // Used to control the Message sending task
uint32_t lastTelemetryPoolTimer = 0; // Used to control the Telemetry pooling task
uint32_t msgCounter = 0; // Incremental number that is sent on all messages

// Box IDs (used to determine which flight modes are active)
uint8_t boxIdArm = 0;
uint8_t boxIdFailsafe = 0;
uint8_t boxIdManual = 0;
uint8_t boxIdAngle = 0;
uint8_t boxIdRth = 0;
uint8_t boxIdPosHold = 0;
uint8_t boxIdCruise = 0;
uint8_t boxIdAltHold = 0;
uint8_t boxIdWaypoint = 0;
uint8_t boxIdHorizon = 0;

 // Flags that prevents some routines to run more than one time
bool boxIdsFetched = 0;
bool callsignFetched = 0;
uint8_t waypointFetchCounter = 0;
uint8_t waypointMessageCounter = 0;

uint32_t lastMspCommunicationTs = 0; // Used to check if MSP protocol is working fine

void connectToTheInternet() {

  // Connect to WiFi network. This code will probably be changed sometime
  connectToWifiNetwork();
  
}

void connectToWifiNetwork() {
  // If WiFi is not connected, then try to connect
  if(WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED, LOW); // Set the led off so we can know that there's no connection
    WiFi.begin(ssid, password);
  }
  
  // Wait for WiFi to connect
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500); // Give WiFi some time to connect
    Serial.println("Connecting WiFi..");
  }

  // WiFi connected - Turn the led ON
  Serial.println("WiFi connected!");
  digitalWrite(LED, HIGH);
}

void setup()
{
  pinMode(LED, OUTPUT); // Set the LED Pin to Output
  
  msp.begin(mspSerial, 750);
  mspSerial.begin(115200, SERIAL_8N1, SERIAL_PIN_RX, SERIAL_PIN_TX, false, 1000L);

  Serial.begin(115200);

}

void loop()
{
  getTelemetryDataTask();
  sendMessageTask();
}

void getTelemetryDataTask() {
  uint32_t timer = millis();
  
  if(timer >= lastTelemetryPoolTimer + poolTelemetryInterval) {
    lastTelemetryPoolTimer = timer;

    getTelemetryData();
  }
}

void getTelemetryData() 
{
  msp_get_boxnames();
  msp_get_gps();
  msp_get_gps_comp();
  msp_get_attitude();
  msp_get_altitude();
  msp_get_sensor_status();
  msp_get_activeboxes();
  msp2_get_inav_analog();
  msp_get_wp_getinfo();
  msp_get_nav_status();
  msp_get_callsign();
  
  get_all_waypoints();
  
  get_cellSignalStrength();
  
  // Recovery routine for MSP serial port
  if (millis() - lastMspCommunicationTs > MSP_PORT_RECOVERY_THRESHOLD)
  {
      msp.reset();
      Serial.println("MSP reset!");
  }
}

void get_cellSignalStrength()
{
    // Simulate Cellular network strength (for now)
    long rnum = random(1, 10);
    if(rnum >= 1 && rnum <= 2 )
      uavstatus.cellSignalStrength = 1;
    else if(rnum >= 3 && rnum <= 5 )
      uavstatus.cellSignalStrength = 2;
    else
      uavstatus.cellSignalStrength = 3;

}

void msp_get_gps() 
{
    MSP_RAW_GPS_t gpsdata;
    if (msp.request(MSP_RAW_GPS, &gpsdata, sizeof(gpsdata)))
    {
      uavstatus.gpsSatCount = gpsdata.numSat;
      uavstatus.gpsHDOP = gpsdata.hdop / 100.0f;
      uavstatus.gpsLatitude = gpsdata.lat / GPS_DEGREES_DIVIDER;
      uavstatus.gpsLongitude = gpsdata.lon / GPS_DEGREES_DIVIDER;
      uavstatus.altitudeSeaLevel = gpsdata.alt;
      uavstatus.groundSpeed = gpsdata.groundSpeed;
      uavstatus.heading = gpsdata.groundCourse / 10.0f;
      uavstatus.gps3Dfix = gpsdata.fixType == 2 ? 1 : 0;

      lastMspCommunicationTs = millis();
    }
    else
    {
      Serial.println("GPS RAW returned false!");
    }
}

void msp_get_gps_comp() {
    MSP_COMP_GPS_t gpsdata;
    if (msp.request(MSP_COMP_GPS, &gpsdata, sizeof(gpsdata)))
    {
      uavstatus.homeDistance = gpsdata.distanceToHome;
      uavstatus.homeDirection = gpsdata.directionToHome;

      lastMspCommunicationTs = millis();
    }
    else
    {
      Serial.println("GPS COMP returned false!");
    }
}

void msp_get_attitude() {
    MSP_ATTITUDE_t inavdata;
    if (msp.request(MSP_ATTITUDE, &inavdata, sizeof(inavdata)))
    {
      uavstatus.rollAngle = inavdata.roll;
      uavstatus.pitchAngle = inavdata.pitch;

      lastMspCommunicationTs = millis();
    }
    else
    {
      Serial.println("ATTITUDE returned false!");
    }
}

void msp_get_altitude() {
    MSP_ALTITUDE_t inavdata;
    if (msp.request(MSP_ALTITUDE, &inavdata, sizeof(inavdata)))
    {
      uavstatus.verticalSpeed = inavdata.estimatedVelocity;
      uavstatus.altitude = inavdata.estimatedAltitude;

      lastMspCommunicationTs = millis();
    }
    else
    {
      Serial.println("ALTITUDE returned false!");
    }
}

void msp_get_wp_getinfo() {
    MSP_WP_GETINFO_t inavdata;
    if (msp.request(MSP_WP_GETINFO, &inavdata, sizeof(inavdata)))
    {
      uavstatus.isWpMissionValid = inavdata.isWaypointListValid;
      uavstatus.waypointCount = inavdata.waypointCount;

      lastMspCommunicationTs = millis();
    }
    else
    {
      Serial.println("ALTITUDE returned false!");
    }
}

void msp_get_nav_status() {
    MSP_NAV_STATUS_t inavdata;
    if (msp.request(MSP_NAV_STATUS, &inavdata, sizeof(inavdata)))
    {
      //Serial.printf("mode: %d | state: %d | activeWpAction: %d | activeWpNumber: %d | error: %d| HeadingHoldTarget: %d\n", inavdata.mode, inavdata.state, inavdata.activeWpAction, inavdata.activeWpNumber, inavdata.error, inavdata.HeadingHoldTarget);

      uavstatus.currentWaypointNumber = inavdata.activeWpNumber;
      //uavstatus.waypointCount = inavdata.waypointCount;

      lastMspCommunicationTs = millis();
    }
    else
    {
      Serial.println("ALTITUDE returned false!");
    }
}

void msp2_get_inav_analog() {
    MSP2_INAV_ANALOG_t inavdata;
    if (msp.request(MSP2_INAV_ANALOG, &inavdata, sizeof(inavdata)))
    {
      uavstatus.batteryVoltage = inavdata.batteryVoltage / 100.0f;
      uavstatus.capacityDraw = inavdata.mAhDraw;
      uavstatus.currentDraw = inavdata.currentDraw / 100.0f;
      uavstatus.rssiPercent = inavdata.rssi / 10.0f;
      uavstatus.fuelPercent = inavdata.batteryPercentage;

      uint8_t cellCount = inavdata.batteryStats >> 4;
      uavstatus.batteryCellCount = cellCount;
      uavstatus.averageCellVoltage = uavstatus.batteryVoltage / cellCount;

      lastMspCommunicationTs = millis();
    }
    else
    {
      Serial.println("MSP2 ANALOG returned false!");
    }
}

void msp_get_sensor_status() {
    MSP_SENSOR_STATUS_t inavdata;
    if (msp.request(MSP_SENSOR_STATUS, &inavdata, sizeof(inavdata)))
    {
      uavstatus.isHardwareHealthy = inavdata.isHardwareHealthy;

      lastMspCommunicationTs = millis();
    }
    else
    {
      Serial.println("MSP SENSOR STATUS returned false!");
    }
}

void get_all_waypoints()
{
  // Run this routine only each 10 cycles
  if(waypointFetchCounter < HOME_POINT_FETCH_INTERVAL)
  {
    waypointFetchCounter++;
  }
  else{
    waypointFetchCounter = 0;

    // Get home point
    msp_get_wp(0);

    // Get waypoints
    for(uint8_t i = 1; i <= uavstatus.waypointCount; i++)
      msp_get_wp(i);
  }
}

void msp_get_wp(uint8_t wp_no) {
    msp_set_wp_t inavdata;
    inavdata.waypointNumber = wp_no;
    
    if (msp.requestWithPayload(MSP_WP, &inavdata, sizeof(inavdata)))
    {
      Serial.printf("A waypointNumber: %d, action: %d, lat: %d, lon: %d, alt: %d, p1: %d, p2: %d, p3: %d, flag: %d\n", inavdata.waypointNumber, inavdata.action, inavdata.lat, inavdata.lon, inavdata.alt, inavdata.p1, inavdata.p2, inavdata.p3, inavdata.flag);
      // uavstatus.isHardwareHealthy = inavdata.isHardwareHealthy;

      if(wp_no == 0)
      {
        uavstatus.homeLatitude = inavdata.lat / GPS_DEGREES_DIVIDER;
        uavstatus.homeLongitude =  inavdata.lon / GPS_DEGREES_DIVIDER;
        uavstatus.homeAltitudeSL = inavdata.alt;
      }

      memcpy(&currentWPMission[wp_no], &inavdata, sizeof(inavdata));

      lastMspCommunicationTs = millis();
    }
    else
    {
      Serial.println("MSP SENSOR STATUS returned false!");
    }
}

void msp_get_boxnames() {
    // Only needs to run once
    if(boxIdsFetched)
      return;
      
    char inavdata[512];
    uint16_t dataLen = 0;
    if (msp.requestText(MSP_BOXNAMES, &inavdata, &dataLen))
    {
      uint8_t boxIndex = 0;

      char* chars_array = strtok(inavdata, ";");
      
      while(chars_array)
      {
        if(strcmp(chars_array, "ARM") == 0)
          boxIdArm = boxIndex;
        else if(strcmp(chars_array, "FAILSAFE") == 0)
          boxIdFailsafe = boxIndex;
        else if(strcmp(chars_array, "MANUAL") == 0)
          boxIdManual = boxIndex;
        else if(strcmp(chars_array, "ANGLE") == 0)
          boxIdAngle = boxIndex;
        else if(strcmp(chars_array, "NAV RTH") == 0)
          boxIdRth = boxIndex;
        else if(strcmp(chars_array, "NAV POSHOLD") == 0)
          boxIdPosHold = boxIndex;
        else if(strcmp(chars_array, "NAV CRUISE") == 0)
          boxIdCruise = boxIndex;
        else if(strcmp(chars_array, "NAV ALTHOLD") == 0)
          boxIdAltHold = boxIndex;
        else if(strcmp(chars_array, "NAV WP") == 0)
          boxIdWaypoint = boxIndex;
        else if(strcmp(chars_array, "HORIZON") == 0)
          boxIdHorizon = boxIndex;

        chars_array = strtok(NULL, ";");
        boxIndex++;
      }

      // Set it to 1 so it doesn't run next time
      boxIdsFetched = 1;

      lastMspCommunicationTs = millis();
    }
    else
    {
      Serial.println("MSP BOXNAMES returned false!");
    }
}

void msp_get_callsign() {
    // Only needs to run once
    if(callsignFetched)
      return;
      
    char inavdata[32];
    uint16_t dataLen = 0;
    if (msp.requestText(MSP_NAME, &inavdata, &dataLen))
    {
      // Allow only chars that are valid
      char allowedChars[] = "ABCDEFGHIJKLMNOPQRSTUWVXYZabcdefghijklmnopqrstuwvxyz0123456789_-";
      char callsign[32] = "";
      uint8_t callsignLen = 0;

      for(uint16_t k = 0; k < dataLen; k++)
      {
        for(uint8_t i = 0; i < sizeof(allowedChars); i++)
        {
          if(allowedChars[i] == inavdata[k])
          {
            // Char is allowed
            callsign[callsignLen] = inavdata[k];
            callsignLen++;
          }
        }
      }
      
      uavstatus.callsign = callsign;

      // Set it to 1 so it doesn't run next time
      callsignFetched = 1;

      lastMspCommunicationTs = millis();
    }
    else
    {
      Serial.println("MSP BOXNAMES returned false!");
    }
}

void msp_get_activeboxes() {
    boxBitmask_t inavdata;
    if (msp.request(MSP_ACTIVEBOXES, &inavdata, sizeof(inavdata)))
    {
      unsigned char boxes[sizeof(inavdata)];
      
      for(uint8_t i = 0; i < sizeof(inavdata); i++)
        boxes[i] = ((char*)&inavdata)[i];

      // Print stuff for debug purposes
      /*
      for(int k = sizeof(inavdata)-1; k >= 0; k--)
      {
        for(int j = 7; j >= 0; j--) {
          bool bfm = (boxes[k] & (1 << j)) != 0;
          Serial.printf("%d", bfm);
        }
        Serial.printf(" ");
      }
      Serial.println("");
      */
      
      uint64_t boxes64 = 0;
      memcpy(&boxes64, boxes, sizeof(uint64_t));

      // Building decision tree
      bool fmArm      = (boxes64 & (1LL << boxIdArm)) != 0;
      bool fmFailsafe = (boxes64 & (1LL << boxIdFailsafe)) != 0;
      bool fmManual   = (boxes64 & (1LL << boxIdManual)) != 0;
      bool fmAngle    = (boxes64 & (1LL << boxIdAngle)) != 0;
      bool fmRth      = (boxes64 & (1LL << boxIdRth)) != 0;
      bool fmPosHold  = (boxes64 & (1LL << boxIdPosHold)) != 0;
      bool fmCruise   = (boxes64 & (1LL << boxIdCruise)) != 0;
      bool fmAltHold  = (boxes64 & (1LL << boxIdAltHold)) != 0;
      bool fmWaypoint = (boxes64 & (1LL << boxIdWaypoint)) != 0;
      bool fmHorizon  = (boxes64 & (1LL << boxIdHorizon)) != 0;

      if(fmManual)
        uavstatus.flightMode= "MANU";
      else if(fmRth)
        uavstatus.flightMode = "RTH";
      else if(fmPosHold && fmAltHold)
        uavstatus.flightMode = "P+AH";
      else if(fmPosHold)
        uavstatus.flightMode = "P H";
      else if(fmCruise && fmAltHold)
        uavstatus.flightMode = "3CRS";
      else if(fmCruise)
        uavstatus.flightMode = "CRS";
      else if(fmWaypoint)
        uavstatus.flightMode = "WP";
      else if(fmAltHold && fmAngle)
        uavstatus.flightMode = "A H";
      else if(fmAngle)
        uavstatus.flightMode = "ANGL";
      else if(fmHorizon)
        uavstatus.flightMode = "HOR";
      else
        uavstatus.flightMode = "ACRO";

      uavstatus.uavIsArmed = fmArm;
      uavstatus.isFailsafeActive = fmFailsafe;

      lastMspCommunicationTs = millis();
    }
    else
    {
      Serial.println("MSP ACTIVEBOXES returned false!");
    }
}

void sendMessageTask() {
  uint32_t timer = millis();
  
  if(timer >= lastMessageTimer + messageSendInterval) {
    lastMessageTimer = timer;
    
    connectToTheInternet();
    connectToTheBroker();
  
    char message[512];
    buildTelemetryMessage(message);
  
    Serial.print("Sending message: ");
    Serial.println(message);
    sendMessage(message);

    // Check if there's a Waypoint mission, and send the message
    if(uavstatus.waypointCount > 0)
      sendWaypointsMessage();
  }
}

void sendWaypointsMessage() {
  if(waypointMessageCounter < WP_MISSION_MESSAGE_INTERVAL)
  {
    waypointMessageCounter++;
    return;
  }
  
  char wpsg[256];

  for(uint8_t i = 0; i <= uavstatus.waypointCount; i++)
  {
    sprintf(wpsg, "wpno:%d,", currentWPMission[i].waypointNumber);
    sprintf(wpsg, "%sla:%.8f,", wpsg, currentWPMission[i].lat / GPS_DEGREES_DIVIDER);
    sprintf(wpsg, "%slo:%.8f,", wpsg, currentWPMission[i].lon / GPS_DEGREES_DIVIDER);
    sprintf(wpsg, "%sal:%d,", wpsg, currentWPMission[i].alt);
    sprintf(wpsg, "%sac:%d,", wpsg, currentWPMission[i].action);
    sprintf(wpsg, "%sp1:%d,", wpsg, currentWPMission[i].p1);
    sprintf(wpsg, "%sp2:%d,", wpsg, currentWPMission[i].p2);
    sprintf(wpsg, "%sp3:%d,", wpsg, currentWPMission[i].p3);
    sprintf(wpsg, "%sf:%d,", wpsg, currentWPMission[i].flag);

    Serial.println(wpsg);
    sendMessage(wpsg);
  }

  waypointMessageCounter = 0;
}

void buildTelemetryMessage(char* message) {
  msgCounter++;
  sprintf(message, "id:%d,", msgCounter); // messageCounter

  // Make 10 groups of messages, to force the values of this groups to be sent even if they didn't change.
  uint8_t msgGroup = (msgCounter % 10);

  if(lastStatus.rollAngle != uavstatus.rollAngle || msgGroup == 0)
    sprintf(message, "%sran:%d,", message, uavstatus.rollAngle); // rollAngle

  if(lastStatus.pitchAngle != uavstatus.pitchAngle || msgGroup == 0)
    sprintf(message, "%span:%d,", message, uavstatus.pitchAngle); // pitchAngle

  if(lastStatus.heading != uavstatus.heading || msgGroup == 1)
    sprintf(message, "%shea:%d,", message, uavstatus.heading); // heading

  if(lastStatus.altitudeSeaLevel != uavstatus.altitudeSeaLevel || msgGroup == 1)
    sprintf(message, "%sasl:%d,", message, uavstatus.altitudeSeaLevel); // altitudeSeaLevel

  if(lastStatus.altitude != uavstatus.altitude || msgGroup == 2)
    sprintf(message, "%salt:%d,", message, uavstatus.altitude); // relative altitude

  if(lastStatus.groundSpeed != uavstatus.groundSpeed || msgGroup == 2)
    sprintf(message, "%sgsp:%d,", message, uavstatus.groundSpeed); // groundSpeed

  if(lastStatus.verticalSpeed != uavstatus.verticalSpeed || msgGroup == 3)
    sprintf(message, "%svsp:%d,", message, uavstatus.verticalSpeed); // verticalSpeed

  if(lastStatus.homeDirection != uavstatus.homeDirection || msgGroup == 3)
    sprintf(message, "%shdr:%d,", message, uavstatus.homeDirection); // homeDirection

  if(lastStatus.homeDistance != uavstatus.homeDistance || msgGroup == 4)
    sprintf(message, "%shds:%d,", message, uavstatus.homeDistance); // homeDistance

  if(lastStatus.averageCellVoltage != uavstatus.averageCellVoltage || msgGroup == 4)
    sprintf(message, "%sacv:%.2f,", message, uavstatus.averageCellVoltage); // batteryVoltage

  if(lastStatus.batteryVoltage != uavstatus.batteryVoltage || msgGroup == 5)
    sprintf(message, "%sbpv:%.2f,", message, uavstatus.batteryVoltage); // batteryVoltage

  if(lastStatus.batteryCellCount != uavstatus.batteryCellCount || msgGroup == 5)
    sprintf(message, "%sbcc:%d,", message, uavstatus.batteryCellCount); // batteryVoltage

  if(lastStatus.fuelPercent != uavstatus.fuelPercent || msgGroup == 6)
    sprintf(message, "%sbfp:%d,", message, uavstatus.fuelPercent); // fuelPercent

  if(lastStatus.currentDraw != uavstatus.currentDraw || msgGroup == 6)
    sprintf(message, "%scud:%.2f,", message, uavstatus.currentDraw); // currentDraw

  if(lastStatus.capacityDraw != uavstatus.capacityDraw || msgGroup == 7)
    sprintf(message, "%scad:%d,", message, uavstatus.capacityDraw); // capacityDraw

  if(lastStatus.rssiPercent != uavstatus.rssiPercent || msgGroup == 7)
    sprintf(message, "%srsi:%d,", message, uavstatus.rssiPercent); // rssiPercent

  if(lastStatus.gpsLatitude != uavstatus.gpsLatitude || msgGroup == 8)
    sprintf(message, "%sgla:%.8f,", message, uavstatus.gpsLatitude); // gpsLatitude

  if(lastStatus.gpsLongitude != uavstatus.gpsLongitude || msgGroup == 8)
    sprintf(message, "%sglo:%.8f,", message,uavstatus.gpsLongitude); // gpsLongitude

  if(lastStatus.gpsSatCount != uavstatus.gpsSatCount || msgGroup == 9)
    sprintf(message, "%sgsc:%d,", message, uavstatus.gpsSatCount); // gpsSatCount

  if(lastStatus.gpsHDOP != uavstatus.gpsHDOP || msgGroup == 9)
    sprintf(message, "%sghp:%.1f,", message, uavstatus.gpsHDOP); // gpsHDOP

  if(lastStatus.cellSignalStrength != uavstatus.cellSignalStrength || msgGroup == 0)
    sprintf(message, "%scss:%d,", message, uavstatus.cellSignalStrength); // cellSignalStrength

  if(lastStatus.gps3Dfix != uavstatus.gps3Dfix || msgGroup == 0)
    sprintf(message, "%s3df:%d,", message, uavstatus.gps3Dfix); // gps3Dfix

  if(lastStatus.isHardwareHealthy != uavstatus.isHardwareHealthy || msgGroup == 1)
    sprintf(message, "%shwh:%d,", message, uavstatus.isHardwareHealthy); // isHardwareHealthy

  if(lastStatus.flightMode != uavstatus.flightMode || msgGroup == 1)
    sprintf(message, "%sftm:%s,", message, uavstatus.flightMode); // flightMode

  if(lastStatus.uavIsArmed != uavstatus.uavIsArmed || msgGroup == 2)
    sprintf(message, "%sarm:%d,", message, uavstatus.uavIsArmed); // uavIsArmed

  if(lastStatus.waypointCount != uavstatus.waypointCount || msgGroup == 2)
    sprintf(message, "%swpc:%d,", message, uavstatus.waypointCount); // waypointCount

  if(lastStatus.currentWaypointNumber != uavstatus.currentWaypointNumber || msgGroup == 3)
    sprintf(message, "%scwn:%d,", message, uavstatus.currentWaypointNumber); // currentWaypointNumber

  if(lastStatus.isWpMissionValid != uavstatus.isWpMissionValid || msgGroup == 3)
    sprintf(message, "%swpv:%d,", message, uavstatus.isWpMissionValid); // isWpMissionValid

  if(lastStatus.callsign != uavstatus.callsign || msgGroup == 4)
    sprintf(message, "%scs:%s,", message, uavstatus.callsign); // callsign

  if(lastStatus.isFailsafeActive != uavstatus.isFailsafeActive || msgGroup == 4)
    sprintf(message, "%sfs:%d,", message, uavstatus.isFailsafeActive); // isFailsafeActive

  if(lastStatus.homeLatitude != uavstatus.homeLatitude || msgGroup == 5)
    sprintf(message, "%shla:%.8f,", message, uavstatus.homeLatitude); // homeLatitude

  if(lastStatus.homeLongitude != uavstatus.homeLongitude || msgGroup == 5)
    sprintf(message, "%shlo:%.8f,", message, uavstatus.homeLongitude); // homeLongitude
  
  if(lastStatus.homeAltitudeSL != uavstatus.homeAltitudeSL || msgGroup == 6)
    sprintf(message, "%shal:%d,", message, uavstatus.homeAltitudeSL); // homeAltitudeSL
  

  lastStatus = uavstatus;
}

void sendMessage(char* message) {
  client.publish(mqttTopic, message);
  Serial.println("Message sent sucessfully...");
}

void connectToTheBroker()
{
  client.setServer(mqttServer, mqttPort);
  while (!client.connected())
  {
    Serial.println("Connecting to the broker MQTT...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword ))
    {
      Serial.println("Connected to the broker!");
    }
    else
    {
      Serial.print("Error connecting to the broker - State: ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}
