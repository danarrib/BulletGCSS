/*
  Config.h

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

// COMMENT NEXT LINE IF YOU WANT TO USE GPRS, UNCOMMENT IF YOU WANT TO USE WIFI
#define USE_WIFI

// UNCOMMENT THE LINE CORRESPONDING TO YOUR CELLULAR MODEM 
#define TINY_GSM_MODEM_SIM800 // If you are using a T-Call board
// #define TINY_GSM_MODEM_SIM7600 // If you are using a T-PCIE board with a SIM7600 module

// WiFi details
const char* ssid = ""; // WiFi Network Name (SSID)
const char* password =  ""; // WiFi Network Password

// MQTT Broker Details
const char* mqttServer = "broker.emqx.io";
const int mqttPort = 1883;
const char* mqttUser = "";
const char* mqttPassword = "";
const char* mqttTopic = "revspace/sensors/dnrbtelem"; // Needs to change

// Pooling settings
#define MESSAGE_SEND_INTERVAL         1000  // milliseconds
#define TELEMETRY_FETCH_DUTY_CYCLE    200   // Time (in milliseconds) that takes to fetch the telemetry data
#define LOW_PRIORITY_MESSAGE_INTERVAL 60    // seconds
#define HOME_POINT_FETCH_INTERVAL     10    // how many iterations to fetch the home point from FC
#define WP_MISSION_MESSAGE_INTERVAL   30    // how many iterations to fetch the WP mission from FC

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[] = "claro.com.br";
const char gprsUser[] = "claro";
const char gprsPass[] = "claro";

// SIM card PIN (leave empty, if not defined)
const char simPIN[]   = ""; 
