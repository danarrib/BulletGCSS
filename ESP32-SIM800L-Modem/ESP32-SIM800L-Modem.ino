/*
  ESP32-SIM800L-Modem.ino

  This is part of Bullet Ground Control Station System
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

#define TINY_GSM_MODEM_SIM800 // Modem is SIM800L

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands
#define SerialAT Serial1

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

// Define the serial console for debug prints, if needed
// #define DUMP_AT_COMMANDS

#include <PubSubClient.h>

#include <HardwareSerial.h>

#include "Config.h"
#include "msp_library.h"
#include "uav_status.h"

HardwareSerial mspSerial(2);

#ifdef USE_WIFI
  #include <WiFi.h>
  WiFiClient espClient;
  PubSubClient client(espClient);
#else
  #include <Wire.h>
  #include <TinyGsmClient.h>
  #ifdef DUMP_AT_COMMANDS
    #include <StreamDebugger.h>
    StreamDebugger debugger(SerialAT, SerialMon);
    TinyGsm modem(debugger);
  #else
    TinyGsm modem(SerialAT);
  #endif
  TinyGsmClient gsmClient(modem);
  PubSubClient client(gsmClient);
#endif

// TTGO T-Call pins
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22

// Pins that will used for the MSP UART.
#define SERIAL_PIN_RX 19
#define SERIAL_PIN_TX 18

#define GPS_DEGREES_DIVIDER 10000000.0f
#define TASK_MSP_READ_MS 200
#define MSP_PORT_RECOVERY_THRESHOLD (TASK_MSP_READ_MS * 5)


#ifndef USE_WIFI
  // I2C for SIM800 (to keep it running when powered from battery)
  TwoWire I2CPower = TwoWire(0);
#endif

#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

MSPLibrary msp;
uav_status uavstatus;
uav_status lastStatus;
msp_set_wp_t currentWPMission[255];

uint32_t lastMessageTimer = 0;  // Used to control the Message sending task
uint32_t lastLowPriorityMessageTimer = 0;  // Used to control the Low Priority Message sending task
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

  #ifdef USE_WIFI
    // Connect to WiFi network.
    connectToWifiNetwork();
  #else
    // Connect to Gprs network.
    connectToGprsNetwork();
  #endif
}

#ifdef USE_WIFI
  // Add only functions that works with WiFi
  void connectToWifiNetwork() {
    if(WiFi.status() == WL_CONNECTED)
      return; // Wifi Already connected, skipping...
    else    
      WiFi.begin(ssid, password);
    
    // Wait for WiFi to connect
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500); // Give WiFi some time to connect
      SerialMon.println("Connecting WiFi..");
    }
  
    SerialMon.println("WiFi connected!");
  }
#else
  // Add only functions that works with GPRS
  bool setPowerBoostKeepOn(int en){
    I2CPower.beginTransmission(IP5306_ADDR);
    I2CPower.write(IP5306_REG_SYS_CTL0);
    if (en) {
      I2CPower.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
    } else {
      I2CPower.write(0x35); // 0x37 is default reg value
    }
    return I2CPower.endTransmission() == 0;
  }

  void connectToGprsNetwork()
  {
    if (modem.isGprsConnected())
      return; // Gprs already connected. Skipping

    // Start I2C communication
    I2CPower.begin(I2C_SDA, I2C_SCL, 400000);
    
    // Keep power when running from battery
    bool isOk = setPowerBoostKeepOn(1);
    SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));
  
    // Set modem reset, enable, power pins
    pinMode(MODEM_PWKEY, OUTPUT);
    pinMode(MODEM_RST, OUTPUT);
    pinMode(MODEM_POWER_ON, OUTPUT);
    digitalWrite(MODEM_PWKEY, LOW);
    digitalWrite(MODEM_RST, HIGH);
    digitalWrite(MODEM_POWER_ON, HIGH);
    
    SerialMon.println("Wait...");
  
    // Set GSM module baud rate and UART pins
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
    delay(6000);
  
    // Restart takes quite some time
    // To skip it, call init() instead of restart()
    SerialMon.println("Initializing modem...");
    modem.restart();
    //modem.init();
  
    String modemInfo = modem.getModemInfo();
    SerialMon.print("Modem Info: ");
    SerialMon.println(modemInfo);
  
    // Unlock your SIM card with a PIN if needed
    if ( GSM_PIN && modem.getSimStatus() != 3 ) {
      modem.simUnlock(GSM_PIN);
    }
  
    SerialMon.print("Connecting to APN: ");
    SerialMon.print(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
      SerialMon.println(" fail");
      ESP.restart();
    }
    else {
      SerialMon.println(" OK");
    }
    
    if (modem.isGprsConnected()) {
      SerialMon.println("GPRS connected");
    }    
  }
#endif

void setup()
{
  SerialMon.begin(115200);
  delay(100);

  #ifndef USE_WIFI
    connectToGprsNetwork();
  #endif
  
  msp.begin(mspSerial, 750);
  mspSerial.begin(115200, SERIAL_8N1, SERIAL_PIN_RX, SERIAL_PIN_TX, false, 1000L);

}

void loop()
{
  getTelemetryDataTask();
  sendMessageTask();
}

void getTelemetryDataTask() {
  uint32_t timer = millis();
  
  if(timer >= lastTelemetryPoolTimer + MESSAGE_SEND_INTERVAL) {
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
  msp2_get_misc2();
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
      SerialMon.println("MSP reset!");
  }
}

void get_cellSignalStrength()
{
    #ifdef USE_WIFI
      long rssi = abs(WiFi.RSSI());

      if(rssi < 50)
        uavstatus.cellSignalStrength = 3;
      else if(rssi < 60)
        uavstatus.cellSignalStrength = 2;
      else if(rssi < 70)
        uavstatus.cellSignalStrength = 1;
      else
        uavstatus.cellSignalStrength = 0;
    #else
      int rssi = modem.getSignalQuality();

      if(rssi == 99)
        uavstatus.cellSignalStrength = 0;
      else if(rssi >= 20)
        uavstatus.cellSignalStrength = 3;
      else if(rssi >= 15)
        uavstatus.cellSignalStrength = 2;
      else if(rssi >= 10)
        uavstatus.cellSignalStrength = 1;
      else
        uavstatus.cellSignalStrength = 0;
    #endif
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
      uavstatus.gpsGroundCourse = gpsdata.groundCourse / 10.0f;
      uavstatus.gps3Dfix = gpsdata.fixType == 2 ? 1 : 0;

      lastMspCommunicationTs = millis();
    }
    else
    {
      SerialMon.println("GPS RAW returned false!");
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
      SerialMon.println("GPS COMP returned false!");
    }
}

void msp_get_attitude() {
    MSP_ATTITUDE_t inavdata;
    if (msp.request(MSP_ATTITUDE, &inavdata, sizeof(inavdata)))
    {
      uavstatus.rollAngle = inavdata.roll;
      uavstatus.pitchAngle = inavdata.pitch;
      uavstatus.heading = inavdata.yaw;

      lastMspCommunicationTs = millis();
    }
    else
    {
      SerialMon.println("ATTITUDE returned false!");
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
      SerialMon.println("ALTITUDE returned false!");
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
      SerialMon.println("ALTITUDE returned false!");
    }
}

void msp_get_nav_status() {
    MSP_NAV_STATUS_t inavdata;
    if (msp.request(MSP_NAV_STATUS, &inavdata, sizeof(inavdata)))
    {
      // SerialMon.printf("mode: %d | state: %d | activeWpAction: %d | activeWpNumber: %d | error: %d| HeadingHoldTarget: %d\n", inavdata.mode, inavdata.state, inavdata.activeWpAction, inavdata.activeWpNumber, inavdata.error, inavdata.HeadingHoldTarget);

      uavstatus.currentWaypointNumber = inavdata.activeWpNumber;
      uavstatus.navState = inavdata.state;
      //uavstatus.waypointCount = inavdata.waypointCount;

      lastMspCommunicationTs = millis();
    }
    else
    {
      SerialMon.println("ALTITUDE returned false!");
    }
}

void msp2_get_inav_analog() {
    MSP2_INAV_ANALOG_t inavdata;
    if (msp.request(MSP2_INAV_ANALOG, &inavdata, sizeof(inavdata)))
    {
      uavstatus.batteryVoltage = inavdata.batteryVoltage / 100.0f;
      uavstatus.capacityDraw = inavdata.mAhDraw;
      uavstatus.mWhDraw = inavdata.mWhDraw;
      uavstatus.currentDraw = inavdata.currentDraw / 100.0f;
      uavstatus.rssiPercent = inavdata.rssi / 10.0f;
      uavstatus.fuelPercent = inavdata.batteryPercentage;

      uint8_t cellCount = inavdata.batteryStats >> 4;
      if(cellCount==0)
        cellCount = 1;
      uavstatus.batteryCellCount = cellCount;
      uavstatus.averageCellVoltage = uavstatus.batteryVoltage / cellCount;

      lastMspCommunicationTs = millis();
    }
    else
    {
      SerialMon.println("MSP2 ANALOG returned false!");
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
      SerialMon.println("MSP SENSOR STATUS returned false!");
    }
}

void msp2_get_misc2() {
    MSP2_INAV_MISC2_t inavdata;
    if (msp.request(MSP2_INAV_MISC2, &inavdata, sizeof(inavdata)))
    {
      //SerialMon.printf("onTime: %d, flightTime: %d, throttlePercent: %d, autoThrottle: %d\n", inavdata.onTime, inavdata.flightTime, inavdata.throttlePercent, inavdata.autoThrottle);
      uavstatus.onTime = inavdata.onTime;
      uavstatus.flightTime = inavdata.flightTime;
      uavstatus.throttlePercent = inavdata.throttlePercent;
      uavstatus.autoThrottle = inavdata.autoThrottle;

      lastMspCommunicationTs = millis();
    }
    else
    {
      SerialMon.println("MSP2 MISC2 returned false!");
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
      // SerialMon.printf("A waypointNumber: %d, action: %d, lat: %d, lon: %d, alt: %d, p1: %d, p2: %d, p3: %d, flag: %d\n", inavdata.waypointNumber, inavdata.action, inavdata.lat, inavdata.lon, inavdata.alt, inavdata.p1, inavdata.p2, inavdata.p3, inavdata.flag);
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
      SerialMon.println("MSP SENSOR STATUS returned false!");
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
      SerialMon.println("MSP BOXNAMES returned false!");
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
      SerialMon.println("MSP BOXNAMES returned false!");
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
          SerialMon.printf("%d", bfm);
        }
        SerialMon.printf(" ");
      }
      SerialMon.println("");
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
      SerialMon.println("MSP ACTIVEBOXES returned false!");
    }
}

void sendMessageTask() {
  uint32_t timer = millis();
  
  if(timer >= lastMessageTimer + MESSAGE_SEND_INTERVAL)
  {
    lastMessageTimer = timer;
    
    connectToTheInternet();
    connectToTheBroker();
  
    char message[512];
    buildTelemetryMessage(message);
  
    SerialMon.print("Sending message: ");
    SerialMon.println(message);
    sendMessage(message);
    
    if(timer >= lastLowPriorityMessageTimer + (LOW_PRIORITY_MESSAGE_INTERVAL * 1000) )
    {
      lastLowPriorityMessageTimer = timer;
      buildLowPriorityMessage(message);
    
      SerialMon.print("Sending Low priority message: ");
      SerialMon.println(message);
      sendMessage(message);
    }
    
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
    if(currentWPMission[i].p1 != 0)
      sprintf(wpsg, "%sp1:%d,", wpsg, currentWPMission[i].p1);
    if(currentWPMission[i].p2 != 0)
      sprintf(wpsg, "%sp2:%d,", wpsg, currentWPMission[i].p2);
    if(currentWPMission[i].p3 != 0)
      sprintf(wpsg, "%sp3:%d,", wpsg, currentWPMission[i].p3);
    if(currentWPMission[i].flag != 0)
      sprintf(wpsg, "%sf:%d,", wpsg, currentWPMission[i].flag);

    SerialMon.print("Sending message: ");
    SerialMon.println(wpsg);
    sendMessage(wpsg);
  }

  waypointMessageCounter = 0;
}

void buildTelemetryMessage(char* message) {
  msgCounter++;
  sprintf(message, ""); // messageCounter

  // Make 10 groups of messages, to force the values of this groups to be sent even if they didn't change.
  uint8_t msgGroup = (msgCounter % 10);

  if(lastStatus.rollAngle != uavstatus.rollAngle || msgGroup == 0)
    sprintf(message, "%sran:%d,", message, uavstatus.rollAngle); // rollAngle

  if(lastStatus.pitchAngle != uavstatus.pitchAngle || msgGroup == 0)
    sprintf(message, "%span:%d,", message, uavstatus.pitchAngle); // pitchAngle

  if(lastStatus.heading != uavstatus.heading || msgGroup == 0)
    sprintf(message, "%shea:%d,", message, uavstatus.heading); // heading

  if(lastStatus.altitudeSeaLevel != uavstatus.altitudeSeaLevel || msgGroup == 1)
    sprintf(message, "%sasl:%d,", message, uavstatus.altitudeSeaLevel); // altitudeSeaLevel

  if(lastStatus.altitude != uavstatus.altitude || msgGroup == 1)
    sprintf(message, "%salt:%d,", message, uavstatus.altitude); // relative altitude

  if(lastStatus.groundSpeed != uavstatus.groundSpeed || msgGroup == 1)
    sprintf(message, "%sgsp:%d,", message, uavstatus.groundSpeed); // groundSpeed

  if(lastStatus.verticalSpeed != uavstatus.verticalSpeed || msgGroup == 2)
    sprintf(message, "%svsp:%d,", message, uavstatus.verticalSpeed); // verticalSpeed

  if(lastStatus.homeDirection != uavstatus.homeDirection || msgGroup == 2)
    sprintf(message, "%shdr:%d,", message, uavstatus.homeDirection); // homeDirection

  if(lastStatus.homeDistance != uavstatus.homeDistance || msgGroup == 2)
    sprintf(message, "%shds:%d,", message, uavstatus.homeDistance); // homeDistance

  if(lastStatus.averageCellVoltage != uavstatus.averageCellVoltage || msgGroup == 3)
    sprintf(message, "%sacv:%.2f,", message, uavstatus.averageCellVoltage); // batteryVoltage

  if(lastStatus.batteryVoltage != uavstatus.batteryVoltage || msgGroup == 3)
    sprintf(message, "%sbpv:%.2f,", message, uavstatus.batteryVoltage); // batteryVoltage

  if(lastStatus.fuelPercent != uavstatus.fuelPercent || msgGroup == 3)
    sprintf(message, "%sbfp:%d,", message, uavstatus.fuelPercent); // fuelPercent

  if(lastStatus.currentDraw != uavstatus.currentDraw || msgGroup == 4)
    sprintf(message, "%scud:%.2f,", message, uavstatus.currentDraw); // currentDraw

  if(lastStatus.capacityDraw != uavstatus.capacityDraw || msgGroup == 4)
    sprintf(message, "%scad:%d,", message, uavstatus.capacityDraw); // capacityDraw

  if(lastStatus.rssiPercent != uavstatus.rssiPercent || msgGroup == 4)
    sprintf(message, "%srsi:%d,", message, uavstatus.rssiPercent); // rssiPercent

  if(lastStatus.gpsLatitude != uavstatus.gpsLatitude || msgGroup == 5)
    sprintf(message, "%sgla:%.8f,", message, uavstatus.gpsLatitude); // gpsLatitude

  if(lastStatus.gpsLongitude != uavstatus.gpsLongitude || msgGroup == 5)
    sprintf(message, "%sglo:%.8f,", message,uavstatus.gpsLongitude); // gpsLongitude

  if(lastStatus.gpsSatCount != uavstatus.gpsSatCount || msgGroup == 5)
    sprintf(message, "%sgsc:%d,", message, uavstatus.gpsSatCount); // gpsSatCount

  if(lastStatus.gpsHDOP != uavstatus.gpsHDOP || msgGroup == 6)
    sprintf(message, "%sghp:%.1f,", message, uavstatus.gpsHDOP); // gpsHDOP

  if(lastStatus.cellSignalStrength != uavstatus.cellSignalStrength || msgGroup == 6)
    sprintf(message, "%scss:%d,", message, uavstatus.cellSignalStrength); // cellSignalStrength

  if(lastStatus.gps3Dfix != uavstatus.gps3Dfix || msgGroup == 6)
    sprintf(message, "%s3df:%d,", message, uavstatus.gps3Dfix); // gps3Dfix

  if(lastStatus.isHardwareHealthy != uavstatus.isHardwareHealthy || msgGroup == 7)
    sprintf(message, "%shwh:%d,", message, uavstatus.isHardwareHealthy); // isHardwareHealthy

  if(lastStatus.uavIsArmed != uavstatus.uavIsArmed || msgGroup == 7)
    sprintf(message, "%sarm:%d,", message, uavstatus.uavIsArmed); // uavIsArmed

  if(lastStatus.waypointCount != uavstatus.waypointCount || msgGroup == 8)
    sprintf(message, "%swpc:%d,", message, uavstatus.waypointCount); // waypointCount

  if(lastStatus.currentWaypointNumber != uavstatus.currentWaypointNumber || msgGroup == 8)
    sprintf(message, "%scwn:%d,", message, uavstatus.currentWaypointNumber); // currentWaypointNumber

  if(lastStatus.isWpMissionValid != uavstatus.isWpMissionValid || msgGroup == 8)
    sprintf(message, "%swpv:%d,", message, uavstatus.isWpMissionValid); // isWpMissionValid

  if(lastStatus.isFailsafeActive != uavstatus.isFailsafeActive || msgGroup == 9)
    sprintf(message, "%sfs:%d,", message, uavstatus.isFailsafeActive); // isFailsafeActive

  if(lastStatus.throttlePercent != uavstatus.throttlePercent || msgGroup == 9)
    sprintf(message, "%strp:%d,", message, uavstatus.throttlePercent); // throttlePercent

  if(lastStatus.autoThrottle != uavstatus.autoThrottle || msgGroup == 9)
    sprintf(message, "%satt:%d,", message, uavstatus.autoThrottle); // autoThrottle

  if(lastStatus.gpsGroundCourse != uavstatus.gpsGroundCourse || msgGroup == 0)
    sprintf(message, "%sggc:%d,", message, uavstatus.gpsGroundCourse); // gpsGroundCourse

  if(lastStatus.navState != uavstatus.navState || msgGroup == 0)
    sprintf(message, "%snvs:%d,", message, uavstatus.navState); // navState

  if(lastStatus.mWhDraw != uavstatus.mWhDraw || msgGroup == 0)
    sprintf(message, "%swhd:%d,", message, uavstatus.mWhDraw); // mWhDraw



  // This values will only be sent if changed... Otherwise they'll be sent by the Low priority message
  if(lastStatus.homeLatitude != uavstatus.homeLatitude)
    sprintf(message, "%shla:%.8f,", message, uavstatus.homeLatitude); // homeLatitude

  if(lastStatus.homeLongitude != uavstatus.homeLongitude)
    sprintf(message, "%shlo:%.8f,", message, uavstatus.homeLongitude); // homeLongitude
  
  if(lastStatus.homeAltitudeSL != uavstatus.homeAltitudeSL)
    sprintf(message, "%shal:%d,", message, uavstatus.homeAltitudeSL); // homeAltitudeSL
  
  if(lastStatus.flightMode != uavstatus.flightMode)
    sprintf(message, "%sftm:%s,", message, uavstatus.flightMode); // flightMode

  lastStatus = uavstatus;
}

void buildLowPriorityMessage(char* message) {
  sprintf(message, ""); // messageCounter

  sprintf(message, "%sbcc:%d,", message, uavstatus.batteryCellCount); // batteryCellCount

  sprintf(message, "%scs:%s,", message, uavstatus.callsign); // callsign

  sprintf(message, "%shla:%.8f,", message, uavstatus.homeLatitude); // homeLatitude

  sprintf(message, "%shlo:%.8f,", message, uavstatus.homeLongitude); // homeLongitude

  sprintf(message, "%shal:%d,", message, uavstatus.homeAltitudeSL); // homeAltitudeSL

  sprintf(message, "%sont:%d,", message, uavstatus.onTime); // onTime

  sprintf(message, "%sflt:%d,", message, uavstatus.flightTime); // flightTime

  sprintf(message, "%sftm:%s,", message, uavstatus.flightMode); // flightMode
  
}

void sendMessage(char* message) {
  client.publish(mqttTopic, message);
  SerialMon.println("Message sent sucessfully...");
}

void connectToTheBroker()
{
  client.setServer(mqttServer, mqttPort);
  while (!client.connected())
  {
    SerialMon.println("Connecting to the broker MQTT...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword ))
    {
      SerialMon.println("Connected to the broker!");
    }
    else
    {
      SerialMon.print("Error connecting to the broker - State: ");
      SerialMon.println(client.state());
      delay(2000);
    }
  }
}
