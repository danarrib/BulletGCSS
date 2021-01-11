/*
  Config.h

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

// COMMENT NEXT LINE IF YOU WANT TO USE GPRS, UNCOMMENT IF YOU WANT TO USE WIFI
#define USE_WIFI

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
const uint32_t messageSendInterval = 2000;
const uint32_t poolTelemetryInterval = 2000;

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[] = "claro.com.br";
const char gprsUser[] = "claro";
const char gprsPass[] = "claro";

// SIM card PIN (leave empty, if not defined)
const char simPIN[]   = ""; 
