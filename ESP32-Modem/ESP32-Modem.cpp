/*
  ESP32-Modem.cpp

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
 * SETTINGS (PlatformIO)
 * Build environments defined in platformio.ini:
 *   esp32-sim7600  — T-PCIE board with SIM7600 module
 *   esp32-sim800   — T-Call board with SIM800 module
 */

#include <Arduino.h>

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands
#define SerialAT Serial1

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

// Define the serial console for debug prints, if needed
// #define DUMP_AT_COMMANDS

#include <PubSubClient.h>
#include <Ed25519.h>
#include <Preferences.h>

#include <HardwareSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "Config.h"
#include "msp_library.h"
#include "uav_status.h"

// Protocol version — increment this only on breaking changes to the telemetry format.
// The UI uses this value to gate version-specific parsing.
// Version 1 = current protocol. Missing pv field is also treated as version 1 by the UI.
#define PROTOCOL_VERSION 1

HardwareSerial mspSerial(2);

#ifdef USE_WIFI
  #ifdef USE_TLS
    #include <WiFiClientSecure.h>
    WiFiClientSecure espClient;
  #else
    #include <WiFi.h>
    WiFiClient espClient;
  #endif
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
  #ifdef USE_TLS
    #include <SSLClient.h>
    SSLClient sslClient(&gsmClient);
    PubSubClient client(sslClient);
  #else
    PubSubClient client(gsmClient);
  #endif
#endif

// TTGO T-Call pins
#define MODEM_TX             27
#define MODEM_RX             26
#define MODEM_PWKEY          4

#ifdef TINY_GSM_MODEM_SIM7600
#undef MODEM_POWER_ON
#define MODEM_POWER_ON       25
#elif defined(TINY_GSM_MODEM_SIM800)
#define MODEM_RST            5
#undef MODEM_POWER_ON
#define MODEM_POWER_ON       23
#define I2C_SDA              21
#define I2C_SCL              22
#endif

// Pins that will used for the MSP UART.
#define SERIAL_PIN_RX 19
#define SERIAL_PIN_TX 18

// Forward declarations required for PlatformIO compatibility.
// Arduino IDE generates these automatically; PlatformIO does not handle
// functions defined inside #ifdef/#else blocks correctly.
#ifdef USE_WIFI
  void connectToWifiNetwork();
#else
  bool setPowerBoostKeepOn(int en);
  void connectToGprsNetwork();
#endif

#define TASK_MSP_READ_MS 160
#define MSP_PORT_RECOVERY_THRESHOLD (TASK_MSP_READ_MS * 5)
// How long to wait between FC ready-probes during cold boot (ms)
#define FC_PROBE_INTERVAL_MS 2000

#ifndef USE_WIFI
  // I2C for SIM800 (to keep it running when powered from battery)
  TwoWire I2CPower = TwoWire(0);
#endif

#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

MSPLibrary msp;
uav_status uavstatus;
uav_status lastStatus;
msp_set_wp_t currentWPMission[256];

// ── Mission upload staging buffer ─────────────────────────────────────────────
// Incoming setwp commands are buffered here; only forwarded to the FC once the
// last WP (f:0xA5) is received and the full mission passes validation.
// Heap-allocated on WP 1 so the size can match the FC-reported maxWaypoints.
#define NAV_MAX_WAYPOINTS_FIRMWARE 15   // fallback when FC has not reported max yet
static msp_set_wp_t *stagedMission = NULL;
static uint8_t       stagedCount   = 0;   // highest wpno received so far
static bool          stagingActive = false; // true after wpno:1 received

uint32_t lastMessageTimer = 0;  // Used to control the Message sending task
uint32_t lastLowPriorityMessageTimer = 0;  // Used to control the Low Priority Message sending task
uint32_t msgCounter = 0; // Incremental number that is sent on all messages
uint16_t failureCounter = 0;    // Count how many times it fails sending the messages
uint8_t  gprsFailureCounter = 0; // Count GPRS-level reconnect failures (separate escalation ladder)
bool     modemInitialized = false; // True after first successful GPRS connection

// Box IDs for flight modes used only for telemetry detection (not remotely commandable).
// Populated once from MSP_BOXNAMES; read in msp_get_activeboxes() only.
// ARM is intentionally kept here and not made commandable — cutting motors mid-flight
// requires dedicated safety confirmation logic that is not yet implemented.
uint8_t boxIdArm        = 0;
uint8_t boxIdFailsafe   = 0;
uint8_t boxIdManual     = 0;
uint8_t boxIdAngle      = 0;
uint8_t boxIdHorizon    = 0;
uint8_t boxIdMspOverride = 0;

// Current RC channel values — updated every telemetry cycle from MSP_RC, used when building MSP_SET_RAW_RC
#define RC_CHANNEL_MAX 34
uint16_t rcChannels[RC_CHANNEL_MAX] = {0};
uint8_t  rcChannelCount = 0;

// MSP RC Override configuration — populated once from MSP2_COMMON_SETTING
uint32_t mspOverrideChannelsMask = 0;
bool mspOverrideFetched = false;

// ── Remotely commandable flight modes ────────────────────────────────────────
// Each FlightMode bundles what were previously four separate per-mode variables:
//   modeRange*, cmdAvailable*, cmdState*, and boxId*.
// ARM is intentionally excluded — see boxIdArm comment above.
FlightMode modeAltHold  = {};
FlightMode modeRth      = {};
FlightMode modeBeeper   = {};
FlightMode modeWp       = {};
FlightMode modeCruise   = {};
FlightMode modePosHold  = {};

// Table of all commandable flight modes.
// Drives mode-range parsing, override-channel setup, RC override sending,
// command dispatch, and conflict resolution — eliminating per-mode repetition.
struct FlightModeEntry {
    const char* cmdName;  // MQTT command string (e.g. "rth")
    uint8_t     permId;   // MSP_PERM_ID_* constant for MSP_BOXIDS / MSP_MODE_RANGES matching
    FlightMode* mode;
};
static FlightModeEntry cmdModes[] = {
    { "rth",     MSP_PERM_ID_RTH,     &modeRth     },
    { "althold", MSP_PERM_ID_ALTHOLD, &modeAltHold },
    { "cruise",  MSP_PERM_ID_CRUISE,  &modeCruise  },
    { "beeper",  MSP_PERM_ID_BEEPER,  &modeBeeper  },
    { "wp",      MSP_PERM_ID_WP,      &modeWp      },
    { "poshold", MSP_PERM_ID_POSHOLD, &modePosHold },
};
static const int CMD_MODE_COUNT = sizeof(cmdModes) / sizeof(cmdModes[0]);
// ─────────────────────────────────────────────────────────────────────────────

 // Flight controller ready flag — set after first successful MSP probe at boot
bool fcReady = false;
uint32_t lastFcProbeTs = 0;

 // Flags that prevents some routines to run more than one time
bool boxIdsFetched = 0;
bool modeRangesFetched = false;
bool fcVersionFetched = false;
bool callsignFetched = 0;
uint8_t  waypointMessageCounter = 0;
uint32_t wpFetchNextTs          = 0;   // when to start the next full WP fetch
int16_t  wpFetchIndex           = -1;  // -1 = idle; 0..waypointCount = fetch in progress

uint32_t lastMspCommunicationTs = 0; // Used to check if MSP protocol is working fine

Preferences prefs;
uint32_t lastSeq = 0; // Last accepted command sequence number (loaded from NVS, replay protection)

// ── FreeRTOS shared state ─────────────────────────────────────────────────────
// dataMutex: protects publishedStatus and publishedMission[] between fcTask
// (writer, end of each 200 ms cycle) and the MQTT loop task (reader, message build).
SemaphoreHandle_t dataMutex;

// cmdMutex: protects cmdState* flags between mqttCommandCallback (MQTT task,
// writer) and msp_send_rc_override / clearAllCommandStates (fcTask, reader/writer).
SemaphoreHandle_t cmdMutex;

// Snapshot of uavstatus and currentWPMission published by fcTask at the end of
// every cycle. The MQTT task reads exclusively from these copies.
uav_status    publishedStatus;
msp_set_wp_t  publishedMission[256];

// Downlink flag: set by the MQTT task when it subscribes to the command topic.
// Read by fcTask to keep uavstatus.downlinkStatus current without a mutex
// (volatile bool read/write is atomic on ESP32).
volatile bool downlinkActive = false;

// ── Forward declarations ──────────────────────────────────────────────────────
// Required because .cpp files do not get Arduino IDE's auto-prototype injection.

void mqttCommandCallback(char* topic, byte* payload, unsigned int length);
bool base64Decode64(const char* input, uint8_t* output);

#ifdef USE_WIFI
void connectToWifiNetwork();
#else
bool setPowerBoostKeepOn(int en);
void connectToGprsNetwork();
#endif

void fcTask(void* param);
void getTelemetryData();
void get_cellSignalStrength();
void msp_get_gps();
void msp_get_gps_comp();
void msp_get_attitude();
void msp_get_altitude();
void msp_get_wp_getinfo();
void msp_get_nav_status();
void msp2_get_inav_analog();
void msp_get_sensor_status();
void msp2_get_misc2();
void msp_get_wp(uint8_t wp_no);
void msp_get_fc_version();
void msp_get_boxids();
void msp_get_mode_ranges();
void msp_get_override_channels();
bool msp_get_setting_u32(const char* name, uint32_t &value);
bool msp_set_setting_u32(const char* name, uint32_t value);
void msp_get_rc();
void msp_send_rc_override();
void clearAllCommandStates();
void resolveChannelConflicts(uint8_t channelIndex);
void msp_get_callsign();
void msp_get_activeboxes();
void sendMessageTask();
void sendWaypointsMessage();
void buildTelemetryMessage(char* message);
void buildLowPriorityMessage(char* message);
void base64Encode32(const uint8_t* input, char* output);
void sendMessage(char* message);
void connectToTheBroker();
// ─────────────────────────────────────────────────────────────────────────────

// Decode a standard base64 character to its 6-bit value, or -1 if invalid.
static int b64CharVal(char c) {
  if (c >= 'A' && c <= 'Z') return c - 'A';
  if (c >= 'a' && c <= 'z') return c - 'a' + 26;
  if (c >= '0' && c <= '9') return c - '0' + 52;
  if (c == '+') return 62;
  if (c == '/') return 63;
  return -1;
}

// Decode exactly 88 base64 characters to 64 bytes (one Ed25519 signature).
// Returns true on success, false if the input is malformed.
bool base64Decode64(const char* input, uint8_t* output) {
  int out = 0;
  // 21 full 4-char groups → 63 bytes
  for (int i = 0; i < 84; i += 4) {
    int a = b64CharVal(input[i]);
    int b = b64CharVal(input[i+1]);
    int c = b64CharVal(input[i+2]);
    int d = b64CharVal(input[i+3]);
    if (a < 0 || b < 0 || c < 0 || d < 0) return false;
    output[out++] = (uint8_t)((a << 2) | (b >> 4));
    output[out++] = (uint8_t)(((b & 0x0f) << 4) | (c >> 2));
    output[out++] = (uint8_t)(((c & 0x03) << 6) | d);
  }
  // Last group: 2 data chars + "==" padding → 1 byte
  int a = b64CharVal(input[84]);
  int b = b64CharVal(input[85]);
  if (a < 0 || b < 0 || input[86] != '=' || input[87] != '=') return false;
  output[out++] = (uint8_t)((a << 2) | (b >> 4));
  return (out == 64);
}

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
      SerialMon.println("Connecting WiFi..");
      delay(5000); // Give WiFi some time to connect
      if(WiFi.status() != WL_CONNECTED)
      {
        failureCounter++;
        if(failureCounter>=10)
        {
          SerialMon.println("Too many fails connecting to WiFi. Restarting...");
          ESP.restart();
        }
      }
    }
  
    SerialMon.println("WiFi connected!");
    #ifdef USE_TLS
      espClient.setInsecure(); // Encrypt without certificate verification
    #endif
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
    if (modem.isGprsConnected()) {
      gprsFailureCounter = 0; // connection is healthy — reset counter
      return;
    }

    if (!modemInitialized) {
      // ── Cold start: runs once at boot (or after a full power cycle) ────────
      // Full hardware setup, modem restart, and blocking network wait are all
      // acceptable here — nothing is flying yet.

      pinMode(MODEM_PWKEY, OUTPUT);
      pinMode(MODEM_POWER_ON, OUTPUT);

      #ifdef TINY_GSM_MODEM_SIM800
      pinMode(MODEM_RST, OUTPUT);
      I2CPower.begin(I2C_SDA, I2C_SCL, 400000);
      bool isOk = setPowerBoostKeepOn(1);
      SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));
      digitalWrite(MODEM_RST, HIGH);
      #endif

      #ifdef TINY_GSM_MODEM_SIM7600
      digitalWrite(MODEM_POWER_ON, HIGH);
      delay(100);
      digitalWrite(MODEM_PWKEY, HIGH);
      delay(500);
      #endif

      digitalWrite(MODEM_PWKEY, LOW);
      digitalWrite(MODEM_POWER_ON, HIGH);

      SerialMon.println("Wait...");
      delay(3000);
      SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
      delay(6000);

      SerialMon.println("Initializing modem...");
      modem.restart();

      #ifdef TINY_GSM_MODEM_SIM7600
      modem.setNetworkMode(38);
      delay(1000);
      String modemName = modem.getModemName();
      SerialMon.print("Modem Name: ");
      SerialMon.println(modemName);
      #endif

      String modemInfo = modem.getModemInfo();
      SerialMon.print("Modem Info: ");
      SerialMon.println(modemInfo);

      if (strlen(GSM_PIN) > 0) {
        SimStatus simStatus = modem.getSimStatus();
        if (simStatus == SIM_LOCKED) {
          SerialMon.println("SIM card is PIN-locked. Attempting to unlock...");
          if (modem.simUnlock(GSM_PIN)) {
            SerialMon.println("SIM card unlocked successfully.");
          } else {
            SerialMon.println("ERROR: SIM card unlock failed. Check GSM_PIN in Config.h.");
            delay(5000);
            ESP.restart();
          }
        } else if (simStatus == SIM_READY) {
          SerialMon.println("SIM card is ready (no PIN required).");
        }
      }

      // Block until registered — acceptable at boot, nothing is flying yet
      while (!modem.waitForNetwork(600000L)) {
        SerialMon.println("Waiting for network...");
        delay(10000);
      }
      SerialMon.println("Network connected");

    } else {
      // ── Reconnect path: GPRS dropped mid-flight ─────────────────────────────
      // Do NOT restart the modem on every drop — that causes the 2-minute hang.
      // Instead, count failures and escalate progressively.
      gprsFailureCounter++;
      SerialMon.printf("GPRS disconnected (failure %d)\n", gprsFailureCounter);

      // Every 5 failures, do a soft modem restart to recover from states like
      // CSQ 99,99 (network registration lost) without a full hardware power cycle.
      if (gprsFailureCounter % 5 == 0) {
        SerialMon.println("Restarting modem for GPRS recovery...");
        modem.restart();
        #ifdef TINY_GSM_MODEM_SIM7600
        modem.setNetworkMode(38);
        delay(1000);
        #endif
        failureCounter++; // counts toward the ESP32 restart threshold too
      }

      // Hard ceiling: if we can't recover after 15 attempts, restart the ESP32
      if (gprsFailureCounter >= 15) {
        SerialMon.println("GPRS unrecoverable after 15 attempts. Restarting ESP32...");
        ESP.restart();
      }

      // Wait briefly for network re-registration — short enough that the main
      // loop keeps running and the FC task is not starved for long.
      if (!modem.isNetworkConnected()) {
        SerialMon.println("Waiting for network re-registration (60s)...");
        if (!modem.waitForNetwork(60000L)) {
          SerialMon.println("Network not available yet. Will retry next cycle.");
          return; // come back next 1-second tick
        }
      }
    }

    // ── Connect GPRS (shared by cold-start and reconnect paths) ─────────────
    SerialMon.print("Connecting to APN: ");
    SerialMon.print(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
      SerialMon.println(" fail");
      if (!modemInitialized) {
        // Cold-start GPRS failure — hard restart (nothing is flying yet)
        ESP.restart();
      }
      // Reconnect failure: already counted above, retry next cycle
      return;
    }
    SerialMon.println(" OK");

    if (modem.isGprsConnected()) {
      SerialMon.println("GPRS connected");
      modemInitialized = true; // set only on first successful GPRS connection
      gprsFailureCounter = 0;
    }
  }
#endif

// High-priority FreeRTOS task: owns all MSP/FC communication.
// Runs every 200 ms using vTaskDelayUntil so the period is exact regardless
// of how long individual MSP calls take. Pinned to Core 1 at priority 2,
// above the Arduino loop task (priority 1), so it preempts network operations
// whenever it needs to run.
void fcTask(void* param) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(TASK_MSP_READ_MS);
    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        getTelemetryData();
    }
}

void setup()
{
  SerialMon.begin(115200);
  delay(100);

  // Load the last accepted command sequence number from NVS (replay protection).
  prefs.begin("bulletgcss", true); // read-only
  lastSeq = prefs.getUInt("lastSeq", 0);
  prefs.end();
  SerialMon.print("Loaded lastSeq from NVS: ");
  SerialMon.println(lastSeq);

  #ifndef USE_WIFI
    connectToGprsNetwork();
  #endif

  msp.begin(mspSerial, 750);
  mspSerial.begin(115200, SERIAL_8N1, SERIAL_PIN_RX, SERIAL_PIN_TX, false, 1000L);

  client.setServer(mqttServer, mqttPort);
  client.setCallback(mqttCommandCallback);

  // Create shared-state mutexes, then start the FC task. All MSP hardware
  // is initialised above, so the task can safely begin probing the FC.
  dataMutex = xSemaphoreCreateMutex();
  cmdMutex  = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(fcTask, "fcTask", 4096, NULL, 2, NULL, 1);
}

// Called by PubSubClient when a message arrives on the downlink (command) topic.
void mqttCommandCallback(char* topic, byte* payload, unsigned int length) {
  if (length == 0 || length >= 256) return;

  char buf[256];
  memcpy(buf, payload, length);
  buf[length] = '\0';

  SerialMon.print("Command received: ");
  SerialMon.println(buf);

  // ── Step 1: Reject immediately if no public key is configured ───────────────
  static const uint8_t zeroKey[32] = {0};
  if (memcmp(commandPublicKey, zeroKey, 32) == 0) {
    SerialMon.println("Command rejected: no public key configured in Config.h");
    return;
  }

  // ── Step 2: Parse all fields ─────────────────────────────────────────────────
  char cmd[32]    = "";
  char cid[8]     = "";
  char seqStr[12] = "";  // uint32 max = 10 digits
  char sig[89]    = "";  // Ed25519 base64 signature = 88 chars + null
  char stateStr[4]   = ""; // "0" or "1" for RC channel commands
  char headingStr[8] = ""; // degrees 0-359 for setheading
  char wpStr[4]      = ""; // waypoint index 0-255 for jumpwp
  char altStr[12]    = ""; // altitude in cm (signed) for setalt
  // setwp fields
  char wpnoStr[4]  = "";
  char wplaStr[13] = "";
  char wploStr[13] = "";
  char wpalStr[12] = "";
  char wpacStr[4]  = "";
  char wpp1Str[8]  = "";
  char wpp2Str[8]  = "";
  char wpp3Str[8]  = "";
  char wpfStr[4]   = "";

  char* token = strtok(buf, ",");
  while (token != NULL) {
    char* colon = strchr(token, ':');
    if (colon != NULL) {
      *colon = '\0';
      char* key   = token;
      char* value = colon + 1;
      if (strcmp(key, "cmd")     == 0) strncpy(cmd,        value, sizeof(cmd)        - 1);
      if (strcmp(key, "cid")     == 0) strncpy(cid,        value, sizeof(cid)        - 1);
      if (strcmp(key, "seq")     == 0) strncpy(seqStr,     value, sizeof(seqStr)     - 1);
      if (strcmp(key, "sig")     == 0) strncpy(sig,        value, sizeof(sig)        - 1);
      if (strcmp(key, "state")   == 0) strncpy(stateStr,   value, sizeof(stateStr)   - 1);
      if (strcmp(key, "heading") == 0) strncpy(headingStr, value, sizeof(headingStr) - 1);
      if (strcmp(key, "wp")      == 0) strncpy(wpStr,      value, sizeof(wpStr)      - 1);
      if (strcmp(key, "alt")     == 0) strncpy(altStr,     value, sizeof(altStr)     - 1);
      if (strcmp(key, "wpno")    == 0) strncpy(wpnoStr,    value, sizeof(wpnoStr)    - 1);
      if (strcmp(key, "la")      == 0) strncpy(wplaStr,    value, sizeof(wplaStr)    - 1);
      if (strcmp(key, "lo")      == 0) strncpy(wploStr,    value, sizeof(wploStr)    - 1);
      if (strcmp(key, "al")      == 0) strncpy(wpalStr,    value, sizeof(wpalStr)    - 1);
      if (strcmp(key, "ac")      == 0) strncpy(wpacStr,    value, sizeof(wpacStr)    - 1);
      if (strcmp(key, "p1")      == 0) strncpy(wpp1Str,    value, sizeof(wpp1Str)    - 1);
      if (strcmp(key, "p2")      == 0) strncpy(wpp2Str,    value, sizeof(wpp2Str)    - 1);
      if (strcmp(key, "p3")      == 0) strncpy(wpp3Str,    value, sizeof(wpp3Str)    - 1);
      if (strcmp(key, "f")       == 0) strncpy(wpfStr,     value, sizeof(wpfStr)     - 1);
    }
    token = strtok(NULL, ",");
  }

  // Defer the default ACK for setwp last-waypoint and getmission — handlers send ACK or NACK conditionally
  bool deferAck = ((strcmp(cmd, "setwp") == 0) && (atoi(wpfStr) == 165)) || (strcmp(cmd, "getmission") == 0);

  // ── Step 3: Require all security fields to be present ────────────────────────
  if (strlen(cmd) == 0 || strlen(cid) == 0 || strlen(seqStr) == 0 || strlen(sig) != 88) {
    SerialMon.println("Command rejected: missing or malformed security fields");
    return;
  }

  // ── Step 4: Replay protection — seq must be strictly greater than lastSeq ────
  uint32_t seq = (uint32_t)strtoul(seqStr, NULL, 10);
  if (seq <= lastSeq) {
    SerialMon.print("Command rejected: seq ");
    SerialMon.print(seq);
    SerialMon.print(" <= lastSeq ");
    SerialMon.println(lastSeq);
    return;
  }

  // ── Step 5: Verify Ed25519 signature ─────────────────────────────────────────
  // The signed payload is the canonical string: "cmd:<cmd>,cid:<cid>,seq:<seq>"
  char canonical[128];
  snprintf(canonical, sizeof(canonical), "cmd:%s,cid:%s,seq:%u", cmd, cid, seq);

  uint8_t sigBytes[64];
  if (!base64Decode64(sig, sigBytes)) {
    SerialMon.println("Command rejected: signature base64 decode failed");
    return;
  }

  if (!Ed25519::verify(sigBytes, commandPublicKey, canonical, strlen(canonical))) {
    SerialMon.println("Command rejected: invalid signature");
    return;
  }

  // ── Step 6: Accept — update lastSeq and acknowledge ──────────────────────────
  lastSeq = seq;
  prefs.begin("bulletgcss", false); // read/write
  prefs.putUInt("lastSeq", lastSeq);
  prefs.end();

  // setwp with f:165 (last WP) sends ACK or NACK conditionally in step 7
  if (!deferAck) {
    char ack[32];
    snprintf(ack, sizeof(ack), "cmd:ack,cid:%s,", cid);
    SerialMon.print("Command verified and accepted. Sending ack: ");
    SerialMon.print(ack);
    sendMessage(ack);
  }

  // ── Step 7: Execute the command ───────────────────────────────────────────────
  if (strcmp(cmd, "ping") == 0) {
      // Stateless — ack already sent above. Nothing else to do.

  } else if (strcmp(cmd, "setheading") == 0) {
      // Set heading hold target while in Cruise mode.
      // Payload field: heading:<degrees 0-359>
      // MSP_SET_HEAD expects centidegrees (U16).
      if (strlen(headingStr) == 0) {
          SerialMon.println("setheading: missing heading field");
          return;
      }
      int deg = atoi(headingStr);
      if (deg < 0 || deg > 359) {
          SerialMon.printf("setheading: invalid heading %d\n", deg);
          return;
      }
      int32_t centideg = (int32_t)(deg * 100);
      msp.send(MSP2_INAV_SET_CRUISE_HEADING, &centideg, sizeof(centideg));
      SerialMon.printf("setheading: sent %d° (%d centideg)\n", deg, centideg);

  } else if (strcmp(cmd, "jumpwp") == 0) {
      // Jump to waypoint N during an active WP mission.
      // Payload field: wp:<index 0-based>
      // MSP2_INAV_SET_WP_INDEX expects a U8.
      if (strlen(wpStr) == 0) {
          SerialMon.println("jumpwp: missing wp field");
          return;
      }
      int wpIdx = atoi(wpStr);
      if (wpIdx < 0 || wpIdx > 254) {
          SerialMon.printf("jumpwp: invalid wp index %d\n", wpIdx);
          return;
      }
      uint8_t wpIdxU8 = (uint8_t)wpIdx;
      msp.send(MSP2_INAV_SET_WP_INDEX, &wpIdxU8, sizeof(wpIdxU8));
      SerialMon.printf("jumpwp: sent index %d\n", wpIdxU8);

  } else if (strcmp(cmd, "setalt") == 0) {
      // Set target altitude while altitude-hold / cruise / WP mode is active.
      // Payload field: alt:<centimetres, signed, relative to home>
      // MSP2_INAV_SET_ALT_TARGET expects I32.
      if (strlen(altStr) == 0) {
          SerialMon.println("setalt: missing alt field");
          return;
      }
      int32_t altCm = (int32_t)atoi(altStr);
      msp.send(MSP2_INAV_SET_ALT_TARGET, &altCm, sizeof(altCm));
      SerialMon.printf("setalt: sent %d cm\n", altCm);

  } else if (strcmp(cmd, "getmission") == 0) {
      // Fetch the full published mission and send each WP as a dlwp: telemetry message,
      // then send ACK. If no mission is loaded, send NACK reason:nomission.
      // Get the WP count under mutex before allocating
      uint8_t wpCount = 0;
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      wpCount = publishedStatus.waypointCount;
      xSemaphoreGive(dataMutex);

      if (wpCount == 0) {
          SerialMon.println("getmission: no mission loaded, sending NACK");
          char nack[64];
          snprintf(nack, sizeof(nack), "cmd:nack,cid:%s,reason:nomission,", cid);
          sendMessage(nack);
          return;
      }

      // Heap-allocate the snapshot to avoid blowing the loopTask stack
      msp_set_wp_t *wpSnapshot = (msp_set_wp_t *)malloc(wpCount * sizeof(msp_set_wp_t));
      if (!wpSnapshot) {
          SerialMon.println("getmission: malloc failed, sending NACK");
          char nack[64];
          snprintf(nack, sizeof(nack), "cmd:nack,cid:%s,reason:oom,", cid);
          sendMessage(nack);
          return;
      }

      xSemaphoreTake(dataMutex, portMAX_DELAY);
      // Clamp in case waypointCount changed between the two mutex acquisitions
      if (publishedStatus.waypointCount < wpCount) wpCount = publishedStatus.waypointCount;
      memcpy(wpSnapshot, publishedMission, wpCount * sizeof(msp_set_wp_t));
      xSemaphoreGive(dataMutex);

      SerialMon.printf("getmission: sending %d WP(s)\n", wpCount);
      for (uint8_t i = 0; i < wpCount; i++) {
          msp_set_wp_t *wp = &wpSnapshot[i];
          char dlwpMsg[128];
          snprintf(dlwpMsg, sizeof(dlwpMsg),
                   "dlwp:%d,la:%ld,lo:%ld,al:%d,ac:%d,p1:%d,p2:%d,p3:%d,f:%d,",
                   (int)wp->waypointNumber,
                   (long)wp->lat, (long)wp->lon,
                   (int)wp->alt, (int)wp->action,
                   (int)wp->p1, (int)wp->p2, (int)wp->p3,
                   (int)wp->flag);
          sendMessage(dlwpMsg);
          SerialMon.printf("getmission: sent dlwp %d\n", i + 1);
      }
      free(wpSnapshot);

      char ackMsg[32];
      snprintf(ackMsg, sizeof(ackMsg), "cmd:ack,cid:%s,", cid);
      sendMessage(ackMsg);
      SerialMon.println("getmission: ACK sent");

  } else if (strcmp(cmd, "setwp") == 0) {
      // Upload a waypoint to the firmware staging buffer.
      // Payload fields: wpno, la (lat×1e7), lo (lon×1e7), al (alt cm), ac (action), p1, p2, p3, f (flag)
      // Non-last WPs (f:0) are staged and ACKed immediately.
      // Last WP (f:165) triggers full validation + FC upload; ACK or NACK is sent after validation.

      if (strlen(wpnoStr) == 0 || strlen(wplaStr) == 0 || strlen(wploStr) == 0 || strlen(wpalStr) == 0) {
          SerialMon.println("setwp: missing required fields");
          if (stagedMission) { free(stagedMission); stagedMission = NULL; }
          stagedCount = 0; stagingActive = false;
          if (deferAck) {
              char nack[64];
              snprintf(nack, sizeof(nack), "cmd:nack,cid:%s,reason:badfields,", cid);
              sendMessage(nack);
          }
          return;
      }

      uint8_t wpno = (uint8_t)atoi(wpnoStr);
      int32_t wpLat = (int32_t)strtol(wplaStr, nullptr, 10);
      int32_t wpLon = (int32_t)strtol(wploStr, nullptr, 10);
      int32_t wpAlt = (int32_t)strtol(wpalStr, nullptr, 10);
      uint8_t wpAc  = (uint8_t)(strlen(wpacStr) ? atoi(wpacStr) : 1);
      int16_t wpP1  = (int16_t)(strlen(wpp1Str) ? atoi(wpp1Str) : 0);
      int16_t wpP2  = (int16_t)(strlen(wpp2Str) ? atoi(wpp2Str) : 0);
      int16_t wpP3  = (int16_t)(strlen(wpp3Str) ? atoi(wpp3Str) : 0);
      uint8_t wpF   = (uint8_t)(strlen(wpfStr)  ? atoi(wpfStr)  : 0);

      uint8_t maxWp = (uavstatus.maxWaypoints > 0) ? uavstatus.maxWaypoints : NAV_MAX_WAYPOINTS_FIRMWARE;
      if (wpno == 0 || wpno > maxWp) {
          SerialMon.printf("setwp: wpno %d out of range (max %d)\n", wpno, maxWp);
          if (stagedMission) { free(stagedMission); stagedMission = NULL; }
          stagedCount = 0; stagingActive = false;
          if (deferAck) {
              char nack[64];
              snprintf(nack, sizeof(nack), "cmd:nack,cid:%s,reason:overflow,", cid);
              sendMessage(nack);
          }
          return;
      }

      // wpno:1 always resets the staging buffer (start of a new upload attempt)
      if (wpno == 1) {
          if (stagedMission) { free(stagedMission); stagedMission = NULL; }
          stagedMission = (msp_set_wp_t *)malloc(maxWp * sizeof(msp_set_wp_t));
          if (!stagedMission) {
              SerialMon.println("setwp: malloc failed for staging buffer");
              stagedCount = 0; stagingActive = false;
              if (deferAck) {
                  char nack[64];
                  snprintf(nack, sizeof(nack), "cmd:nack,cid:%s,reason:oom,", cid);
                  sendMessage(nack);
              }
              return;
          }
          memset(stagedMission, 0, maxWp * sizeof(msp_set_wp_t));
          stagedCount = 0;
          stagingActive = true;
          SerialMon.println("setwp: staging buffer reset for new upload");
      }

      if (!stagingActive) {
          SerialMon.printf("setwp: received wpno:%d but staging not active — missing wpno:1\n", wpno);
          if (deferAck) {
              char nack[64];
              snprintf(nack, sizeof(nack), "cmd:nack,cid:%s,reason:gap,", cid);
              sendMessage(nack);
          }
          return;
      }

      // Store into staging buffer (wpno is 1-based)
      stagedMission[wpno - 1].waypointNumber = wpno;
      stagedMission[wpno - 1].action         = wpAc;
      stagedMission[wpno - 1].lat            = wpLat;
      stagedMission[wpno - 1].lon            = wpLon;
      stagedMission[wpno - 1].alt            = wpAlt;
      stagedMission[wpno - 1].p1             = wpP1;
      stagedMission[wpno - 1].p2             = wpP2;
      stagedMission[wpno - 1].p3             = wpP3;
      stagedMission[wpno - 1].flag           = wpF;
      if (wpno > stagedCount) stagedCount = wpno;

      SerialMon.printf("setwp: staged WP %d (action:%d lat:%ld lon:%ld alt:%ld f:%d)\n",
                       wpno, wpAc, (long)wpLat, (long)wpLon, (long)wpAlt, wpF);

      if (wpF != 0xA5) {
          // Non-last WP — ACK was already sent in step 6. Done.
          return;
      }

      // ── Last WP received — validate before forwarding to FC ──────────────────
      // Refuse if WP Mission mode is currently active on the FC
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
          uint8_t fmWpSnap = publishedStatus.fmWp;
          xSemaphoreGive(dataMutex);
          if (fmWpSnap != 0) {
              SerialMon.println("setwp: rejected — WP Mission mode is currently active");
              char nack[64];
              snprintf(nack, sizeof(nack), "cmd:nack,cid:%s,reason:busy,", cid);
              sendMessage(nack);
              if (stagedMission) { free(stagedMission); stagedMission = NULL; }
              stagedCount = 0; stagingActive = false;
              return;
          }
      }

      // Check all indices 1..stagedCount are filled (no gaps)
      for (uint8_t i = 0; i < stagedCount; i++) {
          if (stagedMission[i].waypointNumber != i + 1) {
              SerialMon.printf("setwp: gap in staged mission at slot %d (expected wpno %d)\n", i, i + 1);
              char nack[64];
              snprintf(nack, sizeof(nack), "cmd:nack,cid:%s,reason:gap,", cid);
              sendMessage(nack);
              if (stagedMission) { free(stagedMission); stagedMission = NULL; }
              stagedCount = 0; stagingActive = false;
              return;
          }
      }

      // All checks passed — upload to FC via MSP_SET_WP
      SerialMon.printf("setwp: uploading %d WP(s) to FC...\n", stagedCount);
      for (uint8_t i = 0; i < stagedCount; i++) {
          msp.send(MSP_SET_WP, &stagedMission[i], sizeof(msp_set_wp_t));
          SerialMon.printf("  WP %d → FC\n", stagedMission[i].waypointNumber);
      }

      // Send success ACK (deferred from step 6)
      char ackWp[32];
      snprintf(ackWp, sizeof(ackWp), "cmd:ack,cid:%s,", cid);
      sendMessage(ackWp);
      SerialMon.println("setwp: upload complete, ACK sent");

      free(stagedMission); stagedMission = NULL;
      stagedCount = 0;
      stagingActive = false;

  } else {
      // Look up the command in the flight mode table.
      FlightModeEntry* entry = nullptr;
      for (int i = 0; i < CMD_MODE_COUNT; i++) {
          if (strcmp(cmd, cmdModes[i].cmdName) == 0) {
              entry = &cmdModes[i];
              break;
          }
      }

      if (entry == nullptr) {
          SerialMon.printf("Unknown command type: '%s'\n", cmd);
          return;
      }

      if (strlen(stateStr) == 0) {
          SerialMon.printf("RC command '%s': missing state field\n", cmd);
          return;
      }
      int state = atoi(stateStr);
      if (state != 0 && state != 1) {
          SerialMon.printf("RC command '%s': invalid state '%s'\n", cmd, stateStr);
          return;
      }
      if (!entry->mode->available) {
          SerialMon.printf("RC command '%s': unavailable (mode not found or channel not overrideable)\n", cmd);
          return;
      }

      xSemaphoreTake(cmdMutex, portMAX_DELAY);
      if (state == 1) {
          // New command wins — clear any other active command on the same channel first.
          resolveChannelConflicts(entry->mode->range.rcChannelIndex);
          entry->mode->active = true;
      } else {
          entry->mode->active = false;
      }
      xSemaphoreGive(cmdMutex);

      if (state == 1)
          SerialMon.printf("%s ON -> RC_CH%d will be held at %d\n", cmd,
                           entry->mode->range.rcChannelIndex + 1, entry->mode->range.onValue);
      else
          SerialMon.printf("%s OFF -> RC_CH%d released to radio\n", cmd,
                           entry->mode->range.rcChannelIndex + 1);
  }
}

void loop()
{
  sendMessageTask();
  client.loop();

  // Check the failure counter and reset everything if it's above 10.
  if(failureCounter >= 10)
  {
    SerialMon.println("Failure count is too high. Restarting everything...");
    ESP.restart();
  }
}

// Wrap a single MSP call with timing printed to serial (uncomment to enable).
// Usage: MSP_TIME("msp_get_gps", msp_get_gps());
#define MSP_TIME(name, call) do { \
    uint32_t _t0 = millis(); \
    call; \
    /*SerialMon.printf("  [timing] %-32s %lu ms\n", name, millis() - _t0);*/ \
} while(0)

void getTelemetryData()
{
  //uint32_t stopWatch = millis();

  // Reflect the MQTT task's downlink subscription state into uavstatus.
  // downlinkActive is volatile bool — atomic read on ESP32, no mutex needed.
  uavstatus.downlinkStatus = downlinkActive ? 1 : 0;

  // ── FC ready probe ────────────────────────────────────────────────────────
  // During cold boot the FC takes longer to start than the ESP32.
  // Until the FC responds, skip all MSP calls to avoid blocking the main loop
  // with cascading 750 ms timeouts. Probe once every FC_PROBE_INTERVAL_MS,
  // flushing the serial buffer first to discard any boot-output garbage.
  if (!fcReady) {
    if (millis() - lastFcProbeTs < FC_PROBE_INTERVAL_MS)
      return;
    lastFcProbeTs = millis();
    msp.reset(); // discard garbage from FC boot output
    char name[32];
    uint16_t nameLen = 0;
    if (msp.requestText(MSP_NAME, name, &nameLen)) {
      SerialMon.println("MSP connected");
      fcReady = true;
      lastMspCommunicationTs = millis();
    } else {
      SerialMon.println("Waiting for flight controller...");
      return;
    }
  }
  // ─────────────────────────────────────────────────────────────────────────

  // Startup-only — each has an internal flag and returns immediately once done.
  msp_get_fc_version();
  msp_get_boxids();
  msp_get_mode_ranges();
  msp_get_override_channels();

  // Always: keep RC values fresh and send any active channel overrides.
  // These two must run every cycle to stay within INAV's 200 ms freshness window.
  uint32_t _cycleStart = millis();
  MSP_TIME("msp_get_rc",          msp_get_rc());
  MSP_TIME("msp_send_rc_override", msp_send_rc_override());

  // Round-robin: spread remaining telemetry across 6 cycles (one group per cycle)
  // so each cycle completes well within the 160 ms period.
  // At 160 ms/cycle each group fires every ~960 ms — fast enough for display.
  static uint8_t rrCycle = 0;
  //SerialMon.printf("[timing] cycle %d\n", rrCycle);
  switch (rrCycle) {
    case 0:
      MSP_TIME("msp_get_gps",           msp_get_gps());
      break;
    case 1:
      MSP_TIME("msp_get_gps_comp",      msp_get_gps_comp());
      MSP_TIME("msp_get_attitude",      msp_get_attitude());
      break;
    case 2:
      MSP_TIME("msp_get_altitude",      msp_get_altitude());
      MSP_TIME("msp_get_sensor_status", msp_get_sensor_status());
      break;
    case 3:
      MSP_TIME("msp2_get_inav_analog",  msp2_get_inav_analog());
      MSP_TIME("msp2_get_misc2",        msp2_get_misc2());
      break;
    case 4:
      MSP_TIME("msp_get_wp_getinfo",    msp_get_wp_getinfo());
      MSP_TIME("msp_get_nav_status",    msp_get_nav_status());
      break;
    case 5:
      MSP_TIME("msp_get_activeboxes",   msp_get_activeboxes());
      MSP_TIME("get_cellSignalStrength", get_cellSignalStrength());
      break;
  }
  // Waypoint fetch state machine — one WP per cycle (~30 ms each) so the
  // 160 ms cycle budget is never exceeded regardless of mission size.
  // wpFetchNextTs=0 at init → first fetch begins immediately on boot.
  if (wpFetchIndex < 0) {
      if (millis() >= wpFetchNextTs)
          wpFetchIndex = 0;  // kick off: always start with home point (WP 0)
  } else {
      MSP_TIME("msp_get_wp", msp_get_wp(wpFetchIndex));
      if (++wpFetchIndex > uavstatus.waypointCount) {
          wpFetchIndex  = -1;
          wpFetchNextTs = millis() + 30000;
      }
  }

  uint32_t _cycleElapsed = millis() - _cycleStart;
  if (_cycleElapsed > TASK_MSP_READ_MS)
      SerialMon.printf("WARNING: cycle %d overrun (%lu ms)\n", rrCycle, _cycleElapsed);
  if (++rrCycle > 5) rrCycle = 0;
  //SerialMon.printf("[timing] total cycle ms: %lu\n", _cycleElapsed);

  // Slow-poll: callsign doesn't change often — refresh every 10 s.
  static uint32_t lastSlowPollTs = 0;
  if (millis() - lastSlowPollTs >= 10000) {
      lastSlowPollTs = millis();
      MSP_TIME("msp_get_callsign", msp_get_callsign());
  }

  // ── MSP recovery routine ──────────────────────────────────────────────────
  // If no successful MSP exchange for too long, reset the serial port and all
  // startup flags so the full init sequence re-runs when the FC comes back.
  if (millis() - lastMspCommunicationTs > MSP_PORT_RECOVERY_THRESHOLD)
  {
      SerialMon.println("MSP connection lost - resetting...");
      msp.reset();
      fcReady           = false;
      boxIdsFetched     = false;
      modeRangesFetched = false;
      fcVersionFetched  = false;
      mspOverrideFetched = false;
      callsignFetched   = false;
      rcChannelCount    = 0;
      for (int i = 0; i < CMD_MODE_COUNT; i++)
          cmdModes[i].mode->available = false;
      // FC disconnect — pilot must regain manual control; clear all active overrides.
      clearAllCommandStates();
  }
  // ─────────────────────────────────────────────────────────────────────────

  // Publish a snapshot of uavstatus and currentWPMission for the MQTT task.
  // The memcpy is fast (~microseconds), so the MQTT task is not blocked for long.
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  memcpy(&publishedStatus,  &uavstatus,        sizeof(uav_status));
  memcpy(publishedMission,   currentWPMission,  sizeof(currentWPMission));
  xSemaphoreGive(dataMutex);
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
      uavstatus.gpsHDOP = gpsdata.hdop;
      uavstatus.gpsLatitude = gpsdata.lat;
      uavstatus.gpsLongitude = gpsdata.lon;
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
      //SerialMon.printf("estimatedVelocity: %d, estimatedAltitude: %d, baroAltitude: %d\n", inavdata.estimatedVelocity, inavdata.estimatedAltitude, inavdata.baroAltitude);
      
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
      uavstatus.maxWaypoints     = inavdata.maxWaypoints;

      if (inavdata.waypointCount != uavstatus.waypointCount) {
          uavstatus.waypointCount = inavdata.waypointCount;
          // Mission changed — cancel any in-progress fetch and start a fresh one immediately.
          wpFetchIndex  = -1;
          wpFetchNextTs = 0;
      }

      lastMspCommunicationTs = millis();
    }
    else
    {
      SerialMon.println("WP GETINFO returned false!");
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
      SerialMon.println("NAV STATUS returned false!");
    }
}

void msp2_get_inav_analog() {
    MSP2_INAV_ANALOG_t inavdata;
    if (msp.request(MSP2_INAV_ANALOG, &inavdata, sizeof(inavdata)))
    {
      uavstatus.batteryVoltage = inavdata.batteryVoltage;
      uavstatus.capacityDraw = inavdata.mAhDraw;
      uavstatus.mWhDraw = inavdata.mWhDraw;
      uavstatus.currentDraw = inavdata.currentDraw;
      uavstatus.rssiPercent = inavdata.rssi / 10.0f;
      uavstatus.fuelPercent = inavdata.batteryPercentage;

      if(uavstatus.rssiPercent > 99)
        uavstatus.rssiPercent = 99;

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

void msp_get_wp(uint8_t wp_no) {
    msp_set_wp_t inavdata;
    inavdata.waypointNumber = wp_no;
    
    if (msp.requestWithPayload(MSP_WP, &inavdata, sizeof(inavdata)))
    {
      // SerialMon.printf("A waypointNumber: %d, action: %d, lat: %d, lon: %d, alt: %d, p1: %d, p2: %d, p3: %d, flag: %d\n", inavdata.waypointNumber, inavdata.action, inavdata.lat, inavdata.lon, inavdata.alt, inavdata.p1, inavdata.p2, inavdata.p3, inavdata.flag);

      if(wp_no == 0)
      {
        uavstatus.homeLatitude = inavdata.lat;
        uavstatus.homeLongitude =  inavdata.lon;
        uavstatus.homeAltitudeSL = inavdata.alt;
      }

      memcpy(&currentWPMission[wp_no], &inavdata, sizeof(inavdata));

      lastMspCommunicationTs = millis();
    }
    else
    {
      SerialMon.println("MSP WP returned false!");
    }
}

void msp_get_fc_version() {
    if (fcVersionFetched)
        return;

    MSP_FC_VERSION_t fcVer;
    uint16_t recvSize = 0;
    if (msp.request(MSP_FC_VERSION, &fcVer, sizeof(fcVer), &recvSize) && recvSize == sizeof(fcVer)) {
        uavstatus.fcVersionMajor = fcVer.versionMajor;
        uavstatus.fcVersionMinor = fcVer.versionMinor;
        uavstatus.fcVersionPatch = fcVer.versionPatchLevel;
        SerialMon.printf("FC version: %d.%d.%d\n", fcVer.versionMajor, fcVer.versionMinor, fcVer.versionPatchLevel);
        fcVersionFetched = true;
        lastMspCommunicationTs = millis();
    } else {
        SerialMon.println("MSP FC_VERSION returned false!");
    }
}

void msp_get_boxids() {
    // Only needs to run once
    if (boxIdsFetched)
        return;

    // MSP_BOXIDS returns one permanentId byte per active box, in boxIndex order.
    // This is far smaller than MSP_BOXNAMES (text) and never overflows INAV's
    // 512-byte MSP output buffer regardless of how many modes are configured.
    uint8_t boxIds[64];
    uint16_t dataLen = 0;
    if (msp.request(MSP_BOXIDS, boxIds, sizeof(boxIds), &dataLen)) {
        for (uint8_t boxIndex = 0; boxIndex < dataLen; boxIndex++) {
            uint8_t permId = boxIds[boxIndex];
            bool matched = false;
            for (int i = 0; i < CMD_MODE_COUNT; i++) {
                if (permId == cmdModes[i].permId) {
                    cmdModes[i].mode->boxId = boxIndex;
                    matched = true;
                    break;
                }
            }
            // Non-commandable modes (used for flight mode detection only)
            if (!matched) {
                if      (permId == MSP_PERM_ID_ARM)         boxIdArm         = boxIndex;
                else if (permId == MSP_PERM_ID_FAILSAFE)    boxIdFailsafe    = boxIndex;
                else if (permId == MSP_PERM_ID_MANUAL)      boxIdManual      = boxIndex;
                else if (permId == MSP_PERM_ID_ANGLE)       boxIdAngle       = boxIndex;
                else if (permId == MSP_PERM_ID_HORIZON)     boxIdHorizon     = boxIndex;
                else if (permId == MSP_PERM_ID_MSPOVERRIDE) boxIdMspOverride = boxIndex;
            }
        }
        boxIdsFetched = 1;
        lastMspCommunicationTs = millis();
    } else {
        SerialMon.println("MSP BOXIDS request failed!");
    }
}

void msp_get_mode_ranges() {
    // Only needs to run once at startup
    if (modeRangesFetched)
        return;

    // INAV supports up to 20 mode activation conditions; each entry is 4 bytes
    const int MAX_ENTRIES = 20;
    modeRangeEntry_t entries[MAX_ENTRIES];
    uint16_t dataLen = 0;

    if (msp.request(MSP_MODE_RANGES, entries, sizeof(entries), &dataLen)) {
        int entryCount = dataLen / sizeof(modeRangeEntry_t);
        SerialMon.printf("MSP_MODE_RANGES: %d entries received\n", entryCount);

        for (int i = 0; i < entryCount; i++) {
            modeRangeEntry_t &e = entries[i];
            // Skip empty/invalid entries (no activation range, or garbage auxChannel value)
            if (e.startStep >= e.endStep || e.auxChannelIndex > 17)
                continue;

            uint8_t rcCh    = e.auxChannelIndex + 4; // 0-based; CH1-CH4 are sticks
            uint16_t onVal  = 900 + ((e.startStep + e.endStep) / 2) * 25;
            uint16_t pwmLo  = 900 + e.startStep * 25;
            uint16_t pwmHi  = 900 + e.endStep   * 25;

            SerialMon.printf("  id=%d auxCh=%d steps=%d-%d => RC_CH%d (%d-%d) onVal=%d\n",
                e.permanentId, e.auxChannelIndex,
                e.startStep, e.endStep,
                rcCh + 1, pwmLo, pwmHi, onVal);

            modeRangeInfo_t info = {rcCh, onVal, pwmLo, pwmHi, true};

            for (int j = 0; j < CMD_MODE_COUNT; j++) {
                if (e.permanentId == cmdModes[j].permId) {
                    cmdModes[j].mode->range = info;
                    break;
                }
            }
        }

        SerialMon.println("Mode ranges summary:");
        for (int j = 0; j < CMD_MODE_COUNT; j++) {
            FlightMode* m = cmdModes[j].mode;
            if (m->range.found)
                SerialMon.printf("  %-8s RC_CH%d onValue=%d\n", cmdModes[j].cmdName,
                                 m->range.rcChannelIndex + 1, m->range.onValue);
            else
                SerialMon.printf("  %-8s not found\n", cmdModes[j].cmdName);
        }

        modeRangesFetched = true;
        lastMspCommunicationTs = millis();
    } else {
        SerialMon.println("MSP_MODE_RANGES request failed!");
    }
}

// Read a uint32_t setting by name via MSP2_COMMON_SETTING (0x1003).
// Request payload: null-terminated setting name.
// Response payload: raw value bytes (4 bytes for uint32_t).
bool msp_get_setting_u32(const char* name, uint32_t &value) {
    uint8_t reqBuf[64];
    uint8_t nameLen = strlen(name) + 1; // include null terminator
    if (nameLen > sizeof(reqBuf))
        return false;
    memcpy(reqBuf, name, nameLen);

    uint32_t resp = 0;
    uint16_t recvSize = 0;
    if (msp.requestWithResponse(MSP2_COMMON_SETTING, reqBuf, nameLen, &resp, sizeof(resp), &recvSize)) {
        value = resp;
        return true;
    }
    return false;
}

// Write a uint32_t setting by name via MSP2_COMMON_SET_SETTING (0x1004).
// Request payload: null-terminated setting name immediately followed by the raw value bytes.
bool msp_set_setting_u32(const char* name, uint32_t value) {
    uint8_t buf[64 + sizeof(uint32_t)];
    uint8_t nameLen = strlen(name) + 1;
    if (nameLen > 64)
        return false;
    memcpy(buf, name, nameLen);
    memcpy(buf + nameLen, &value, sizeof(uint32_t));
    return msp.command(MSP2_COMMON_SET_SETTING, buf, nameLen + sizeof(uint32_t));
}

// Helper: update cmdAvailable* flags from the current mspOverrideChannelsMask and print a summary.
static void updateCommandAvailability() {
    for (int i = 0; i < CMD_MODE_COUNT; i++) {
        FlightMode* m = cmdModes[i].mode;
        if (!m->range.found) {
            SerialMon.printf("  %-8s UNAVAILABLE (no mode range)\n", cmdModes[i].cmdName);
            m->available = false;
        } else {
            bool enabled = (mspOverrideChannelsMask & (1UL << m->range.rcChannelIndex)) != 0;
            m->available = enabled;
            SerialMon.printf("  %-8s RC_CH%-2d %s\n", cmdModes[i].cmdName,
                             m->range.rcChannelIndex + 1, enabled ? "OK" : "BLOCKED");
        }
    }
}

void msp_get_override_channels() {
    // Only needs to run once, and only after mode ranges are known
    if (mspOverrideFetched || !modeRangesFetched)
        return;

    // Step 1: read the current override mask from the FC
    if (!msp_get_setting_u32("msp_override_channels", mspOverrideChannelsMask)) {
        SerialMon.println("msp_override_channels read failed!");
        return;
    }
    SerialMon.printf("msp_override_channels (current): 0x%08X\n", mspOverrideChannelsMask);

    // Step 2: build a mask of every channel we need to control
    uint32_t neededMask = 0;
    for (int i = 0; i < CMD_MODE_COUNT; i++) {
        if (cmdModes[i].mode->range.found)
            neededMask |= (1UL << cmdModes[i].mode->range.rcChannelIndex);
    }

    // Step 3: if any needed channels are missing, OR them in and write back
    uint32_t missingMask = neededMask & ~mspOverrideChannelsMask;
    if (missingMask != 0) {
        uint32_t newMask = mspOverrideChannelsMask | missingMask;
        SerialMon.printf("Adding channels to msp_override_channels: 0x%08X -> 0x%08X\n",
                         mspOverrideChannelsMask, newMask);

        if (!msp_set_setting_u32("msp_override_channels", newMask)) {
            SerialMon.println("msp_override_channels write failed!");
            return;
        }

        // Step 4: read back to confirm the FC accepted the new value
        uint32_t confirmedMask = 0;
        if (!msp_get_setting_u32("msp_override_channels", confirmedMask)) {
            SerialMon.println("msp_override_channels confirm-read failed!");
            return;
        }
        mspOverrideChannelsMask = confirmedMask;
        SerialMon.printf("msp_override_channels (confirmed): 0x%08X\n", mspOverrideChannelsMask);
    }

    // Step 5: update per-command availability from the final confirmed mask
    SerialMon.println("Command availability:");
    updateCommandAvailability();

    mspOverrideFetched = true;
    lastMspCommunicationTs = millis();
}

void msp_get_rc() {
    uint16_t buf[RC_CHANNEL_MAX];
    uint16_t dataLen = 0;

    if (msp.request(MSP_RC, buf, sizeof(buf), &dataLen)) {
        uint8_t count = dataLen / sizeof(uint16_t);
        if (count > RC_CHANNEL_MAX)
            count = RC_CHANNEL_MAX;

        bool firstRead = (rcChannelCount == 0);
        rcChannelCount = count;
        for (uint8_t i = 0; i < count; i++)
            rcChannels[i] = buf[i];

        if (firstRead) {
            SerialMon.printf("MSP_RC: %d channels\n", count);
            for (uint8_t i = 0; i < count; i++)
                SerialMon.printf("  CH%d: %d\n", i + 1, rcChannels[i]);
        }

        lastMspCommunicationTs = millis();
    } else {
        SerialMon.println("MSP_RC request failed!");
    }
}

// Clear all per-command active states. Called when MSP RC Override goes inactive
// mid-flight, or when the FC disconnects — prevents stale commands from firing
// when the pilot regains control or the FC reconnects.
void clearAllCommandStates() {
    xSemaphoreTake(cmdMutex, portMAX_DELAY);
    for (int i = 0; i < CMD_MODE_COUNT; i++)
        cmdModes[i].mode->active = false;
    xSemaphoreGive(cmdMutex);
    uavstatus.cmdRth = uavstatus.cmdAltHold = uavstatus.cmdCruise =
        uavstatus.cmdBeeper = uavstatus.cmdWp = uavstatus.cmdPosHold = 0;
}

// Clear any active commands that share the same RC channel as a new incoming
// command. Called before setting a new command state to ON so that the latest
// command always wins on a shared channel.
void resolveChannelConflicts(uint8_t channelIndex) {
    for (int i = 0; i < CMD_MODE_COUNT; i++) {
        FlightMode* m = cmdModes[i].mode;
        if (m->active && m->range.found && m->range.rcChannelIndex == channelIndex)
            m->active = false;
    }
}

// Return a safe neutral PWM value for a channel that has known mode ranges but
// no active mode. Strategy: scan [900, 2100] for the largest contiguous gap not
// covered by any mode on this channel, then return the midpoint of that gap.
// Falls back to 1000 if no gap is found (shouldn't happen in practice).
static uint16_t findSafeOffValue(uint8_t ch) {
    // Collect all [startPWM, endPWM] intervals for modes on this channel.
    const uint16_t PWM_MIN = 900;
    const uint16_t PWM_MAX = 2100;

    // Max entries: CMD_MODE_COUNT intervals.
    uint16_t starts[CMD_MODE_COUNT];
    uint16_t ends[CMD_MODE_COUNT];
    int count = 0;
    for (int i = 0; i < CMD_MODE_COUNT; i++) {
        modeRangeInfo_t& r = cmdModes[i].mode->range;
        if (!r.found || r.rcChannelIndex != ch) continue;
        starts[count] = r.startPWM;
        ends[count]   = r.endPWM;
        count++;
    }

    if (count == 0) return 1000; // no ranges — shouldn't reach here

    // Insertion sort by startPWM.
    for (int i = 1; i < count; i++) {
        uint16_t ks = starts[i], ke = ends[i];
        int j = i - 1;
        while (j >= 0 && starts[j] > ks) {
            starts[j+1] = starts[j];
            ends[j+1]   = ends[j];
            j--;
        }
        starts[j+1] = ks;
        ends[j+1]   = ke;
    }

    // Scan for the largest gap: check [PWM_MIN, first start], each [end_i, start_{i+1}], [last end, PWM_MAX].
    uint16_t bestMid = 1000;
    uint16_t bestGap = 0;

    auto checkGap = [&](uint16_t lo, uint16_t hi) {
        if (hi <= lo) return;
        uint16_t gap = hi - lo;
        if (gap > bestGap) {
            bestGap = gap;
            bestMid = lo + gap / 2;
        }
    };

    checkGap(PWM_MIN, starts[0]);
    for (int i = 0; i < count - 1; i++)
        checkGap(ends[i], starts[i+1]);
    checkGap(ends[count-1], PWM_MAX);

    return bestMid;
}

// Send MSP_SET_RAW_RC to the FC when any sustained RC channel command is active.
// Called every telemetry cycle (~200 ms) after msp_get_rc() refreshes rcChannels[].
// Starts from the real radio values so untouched channels pass through unmodified.
void msp_send_rc_override() {
    if (rcChannelCount == 0) return;
    if (!uavstatus.mspRcOverride) return;

    // Snapshot active states under cmdMutex so we don't race with mqttCommandCallback.
    bool activeSnapshot[CMD_MODE_COUNT];
    bool anyActive = false;
    xSemaphoreTake(cmdMutex, portMAX_DELAY);
    for (int i = 0; i < CMD_MODE_COUNT; i++) {
        activeSnapshot[i] = cmdModes[i].mode->active;
        if (activeSnapshot[i]) anyActive = true;
    }
    xSemaphoreGive(cmdMutex);

    // Update telemetry command state fields from the snapshot (always, even when nothing active).
    // cmdModes[] order: 0=rth, 1=althold, 2=cruise, 3=beeper, 4=wp, 5=poshold
    uavstatus.cmdRth     = activeSnapshot[0] ? 1 : 0;
    uavstatus.cmdAltHold = activeSnapshot[1] ? 1 : 0;
    uavstatus.cmdCruise  = activeSnapshot[2] ? 1 : 0;
    uavstatus.cmdBeeper  = activeSnapshot[3] ? 1 : 0;
    uavstatus.cmdWp      = activeSnapshot[4] ? 1 : 0;
    uavstatus.cmdPosHold = activeSnapshot[5] ? 1 : 0;

    if (!anyActive) return;

    // Start from the current radio values read by msp_get_rc().
    // NOTE: when MSP RC Override is active, MSP_RC returns our own previously
    // sent values, not the real radio values. We must therefore explicitly
    // neutralise any channel that has known mode ranges but no active mode,
    // otherwise deactivated channels keep the last overridden value.
    uint16_t channels[RC_CHANNEL_MAX];
    memcpy(channels, rcChannels, rcChannelCount * sizeof(uint16_t));

    // For each RC channel that has at least one known range but no active mode,
    // find the largest gap in [900, 2100] not covered by any mode on that channel
    // and place the channel at the midpoint of that gap.
    for (uint8_t ch = 0; ch < rcChannelCount; ch++) {
        bool chHasMode       = false;
        bool chHasActiveMode = false;

        for (int i = 0; i < CMD_MODE_COUNT; i++) {
            modeRangeInfo_t& r = cmdModes[i].mode->range;
            if (!r.found || r.rcChannelIndex != ch) continue;
            chHasMode = true;
            if (activeSnapshot[i]) chHasActiveMode = true;
        }

        if (chHasMode && !chHasActiveMode)
            channels[ch] = findSafeOffValue(ch);
    }

    // Apply active mode overrides (always wins over the neutral value above).
    //static uint32_t lastRcOverrideTs = 0;
    //uint32_t now = millis();
    //if (lastRcOverrideTs != 0)
    //    SerialMon.printf("MSP_SET_RAW_RC: %lu ms since last call\n", now - lastRcOverrideTs);
    //lastRcOverrideTs = now;
    //SerialMon.printf("MSP_SET_RAW_RC: sending %d channels\n", rcChannelCount);
    for (int i = 0; i < CMD_MODE_COUNT; i++) {
        if (activeSnapshot[i] && cmdModes[i].mode->range.found) {
            uint8_t ch = cmdModes[i].mode->range.rcChannelIndex;
            uint16_t val = cmdModes[i].mode->range.onValue;
            //SerialMon.printf("  %s: CH%d %d -> %d\n", cmdModes[i].cmdName, ch + 1, channels[ch], val);
            channels[ch] = val;
        }
    }
    //SerialMon.printf("  Full channel dump:");
    //for (int i = 0; i < rcChannelCount; i++)
    //    SerialMon.printf(" CH%d=%d", i + 1, channels[i]);
    //SerialMon.println();

    msp.send(MSP_SET_RAW_RC, channels, rcChannelCount * sizeof(uint16_t));
    //SerialMon.println("  MSP_SET_RAW_RC sent.");
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
      uint8_t callsignLen = 0;

      for(uint16_t k = 0; k < dataLen; k++)
      {
        for(uint8_t i = 0; i < sizeof(allowedChars); i++)
        {
          if(allowedChars[i] == inavdata[k])
          {
            // Char is allowed
            uavstatus.callsign[callsignLen] = inavdata[k];
            callsignLen++;
          }
        }
      }
      
      // Set it to 1 so it doesn't run next time
      callsignFetched = 1;

      lastMspCommunicationTs = millis();
    }
    else
    {
      SerialMon.println("MSP CALLSIGN returned false!");
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
      bool fmArm        = (boxes64 & (1LL << boxIdArm))           != 0;
      bool fmFailsafe   = (boxes64 & (1LL << boxIdFailsafe))      != 0;
      bool fmManual     = (boxes64 & (1LL << boxIdManual))        != 0;
      bool fmAngle      = (boxes64 & (1LL << boxIdAngle))          != 0;
      bool fmRth        = (boxes64 & (1LL << modeRth.boxId))      != 0;
      bool fmPosHold    = (boxes64 & (1LL << modePosHold.boxId))  != 0;
      bool fmCruise     = (boxes64 & (1LL << modeCruise.boxId))   != 0;
      bool fmAltHold    = (boxes64 & (1LL << modeAltHold.boxId))  != 0;
      bool fmWaypoint   = (boxes64 & (1LL << modeWp.boxId))       != 0;
      bool fmHorizon    = (boxes64 & (1LL << boxIdHorizon))       != 0;
      bool fmMspOverride = (boxes64 & (1LL << boxIdMspOverride))  != 0;

      /*
      MANU = 1
      RTH  = 2
      P+AH = 3
      P H  = 4
      3CRS = 5
      CRS  = 6
      WP   = 7
      A H  = 8
      ANGL = 9
      HOR  = 10
      ACRO = 11
      */
      
      if(fmManual)
        uavstatus.flightModeId = 1;
      else if(fmRth)
        uavstatus.flightModeId = 2;
      else if(fmPosHold && fmAltHold)
        uavstatus.flightModeId = 3;
      else if(fmPosHold)
        uavstatus.flightModeId = 4;
      else if(fmCruise && fmAltHold)
        uavstatus.flightModeId = 5;
      else if(fmCruise)
        uavstatus.flightModeId = 6;
      else if(fmWaypoint)
        uavstatus.flightModeId = 7;
      else if(fmAltHold && fmAngle)
        uavstatus.flightModeId = 8;
      else if(fmAngle)
        uavstatus.flightModeId = 9;
      else if(fmHorizon)
        uavstatus.flightModeId = 10;
      else
        uavstatus.flightModeId = 11;

      uavstatus.uavIsArmed = fmArm;
      uavstatus.isFailsafeActive = fmFailsafe;
      uavstatus.fmCruise  = fmCruise;
      uavstatus.fmAltHold = fmAltHold;
      uavstatus.fmWp      = fmWaypoint;
      uavstatus.fmPosHold = fmPosHold;

      // Detect MSP RC Override going inactive — clear all command states so
      // stale commands don't fire when the pilot switches the mode back on.
      if (uavstatus.mspRcOverride && !fmMspOverride) {
          SerialMon.println("MSP RC Override deactivated — clearing all command states");
          clearAllCommandStates();
      }
      uavstatus.mspRcOverride = fmMspOverride;

      lastMspCommunicationTs = millis();
    }
    else
    {
      SerialMon.println("MSP ACTIVEBOXES returned false!");
    }
}

void sendMessageTask() {
  uint32_t timer = millis();
  
  if(timer - lastMessageTimer >= MESSAGE_SEND_INTERVAL)
  {
    //SerialMon.printf("Message: %d\n", timer);
    lastMessageTimer = timer;

    connectToTheInternet();
    connectToTheBroker();

    if(msgCounter==0)
    {
      SerialMon.print("Sending first message: id:0");
      char sessionStartMsg[] = "id:0,";
      sendMessage(sessionStartMsg);
    }

    char message[512];
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    buildTelemetryMessage(message);
    xSemaphoreGive(dataMutex);

    SerialMon.print("Sending message: ");
    SerialMon.print(message);
    sendMessage(message);

    if(timer - lastLowPriorityMessageTimer >= (LOW_PRIORITY_MESSAGE_INTERVAL * 1000))
    {
      lastLowPriorityMessageTimer = timer;
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      buildLowPriorityMessage(message);
      xSemaphoreGive(dataMutex);

      SerialMon.print("Sending low priority message: ");
      SerialMon.print(message);
      sendMessage(message);
    }

    // Check if there's a Waypoint mission, and send the message
    if(publishedStatus.waypointCount > 0)
      sendWaypointsMessage();
      
  }
}

void sendWaypointsMessage() {
  if(waypointMessageCounter < WP_MISSION_MESSAGE_INTERVAL)
  {
    waypointMessageCounter++;
    return;
  }

  // Snapshot mission data under dataMutex before iterating.
  // The mutex is held only for the fast memcpy; MQTT publish happens after release.
  uint8_t wpCount;
  msp_set_wp_t wpSnapshot[256];
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  wpCount = publishedStatus.waypointCount;
  memcpy(wpSnapshot, publishedMission, (wpCount + 1) * sizeof(msp_set_wp_t));
  xSemaphoreGive(dataMutex);

  char wpsg[256];

  for(uint16_t i = 0; i <= wpCount; i++)
  {
    sprintf(wpsg, "wpno:%d,", wpSnapshot[i].waypointNumber);
    sprintf(wpsg, "%sla:%d,", wpsg, wpSnapshot[i].lat);
    sprintf(wpsg, "%slo:%d,", wpsg, wpSnapshot[i].lon);
    sprintf(wpsg, "%sal:%d,", wpsg, wpSnapshot[i].alt);
    sprintf(wpsg, "%sac:%d,", wpsg, wpSnapshot[i].action);
    if(wpSnapshot[i].p1 != 0)
      sprintf(wpsg, "%sp1:%d,", wpsg, wpSnapshot[i].p1);
    if(wpSnapshot[i].p2 != 0)
      sprintf(wpsg, "%sp2:%d,", wpsg, wpSnapshot[i].p2);
    if(wpSnapshot[i].p3 != 0)
      sprintf(wpsg, "%sp3:%d,", wpsg, wpSnapshot[i].p3);
    if(wpSnapshot[i].flag != 0)
      sprintf(wpsg, "%sf:%d,", wpsg, wpSnapshot[i].flag);

    SerialMon.print("Sending message: ");
    SerialMon.print(wpsg);
    sendMessage(wpsg);
  }

  waypointMessageCounter = 0;
}

void buildTelemetryMessage(char* message) {
  msgCounter++;
  sprintf(message, ""); // messageCounter

  // Make 10 groups of messages, to force the values of this groups to be sent even if they didn't change.
  uint8_t msgGroup = (msgCounter % 10);

  if(lastStatus.rollAngle != publishedStatus.rollAngle || msgGroup == 0)
    sprintf(message, "%sran:%d,", message, publishedStatus.rollAngle); // rollAngle

  if(lastStatus.pitchAngle != publishedStatus.pitchAngle || msgGroup == 0)
    sprintf(message, "%span:%d,", message, publishedStatus.pitchAngle); // pitchAngle

  if(lastStatus.heading != publishedStatus.heading || msgGroup == 0)
    sprintf(message, "%shea:%d,", message, publishedStatus.heading); // heading

  if(lastStatus.altitudeSeaLevel != publishedStatus.altitudeSeaLevel || msgGroup == 1)
    sprintf(message, "%sasl:%d,", message, publishedStatus.altitudeSeaLevel); // altitudeSeaLevel

  if(lastStatus.altitude != publishedStatus.altitude || msgGroup == 1)
    sprintf(message, "%salt:%d,", message, publishedStatus.altitude); // relative altitude

  if(lastStatus.groundSpeed != publishedStatus.groundSpeed || msgGroup == 1)
    sprintf(message, "%sgsp:%d,", message, publishedStatus.groundSpeed); // groundSpeed

  if(lastStatus.verticalSpeed != publishedStatus.verticalSpeed || msgGroup == 2)
    sprintf(message, "%svsp:%d,", message, publishedStatus.verticalSpeed); // verticalSpeed

  if(lastStatus.homeDirection != publishedStatus.homeDirection || msgGroup == 2)
    sprintf(message, "%shdr:%d,", message, publishedStatus.homeDirection); // homeDirection

  if(lastStatus.homeDistance != publishedStatus.homeDistance || msgGroup == 2)
    sprintf(message, "%shds:%d,", message, publishedStatus.homeDistance); // homeDistance

  if(lastStatus.averageCellVoltage != publishedStatus.averageCellVoltage || msgGroup == 3)
    sprintf(message, "%sacv:%d,", message, publishedStatus.averageCellVoltage); // batteryVoltage

  if(lastStatus.batteryVoltage != publishedStatus.batteryVoltage || msgGroup == 3)
    sprintf(message, "%sbpv:%d,", message, publishedStatus.batteryVoltage); // batteryVoltage

  if(lastStatus.fuelPercent != publishedStatus.fuelPercent || msgGroup == 3)
    sprintf(message, "%sbfp:%d,", message, publishedStatus.fuelPercent); // fuelPercent

  if(lastStatus.currentDraw != publishedStatus.currentDraw || msgGroup == 4)
    sprintf(message, "%scud:%d,", message, publishedStatus.currentDraw); // currentDraw

  if(lastStatus.capacityDraw != publishedStatus.capacityDraw || msgGroup == 4)
    sprintf(message, "%scad:%d,", message, publishedStatus.capacityDraw); // capacityDraw

  if(lastStatus.rssiPercent != publishedStatus.rssiPercent || msgGroup == 4)
    sprintf(message, "%srsi:%d,", message, publishedStatus.rssiPercent); // rssiPercent

  if(lastStatus.gpsLatitude != publishedStatus.gpsLatitude || msgGroup == 5)
    sprintf(message, "%sgla:%d,", message, publishedStatus.gpsLatitude); // gpsLatitude

  if(lastStatus.gpsLongitude != publishedStatus.gpsLongitude || msgGroup == 5)
    sprintf(message, "%sglo:%d,", message, publishedStatus.gpsLongitude); // gpsLongitude

  if(lastStatus.gpsSatCount != publishedStatus.gpsSatCount || msgGroup == 5)
    sprintf(message, "%sgsc:%d,", message, publishedStatus.gpsSatCount); // gpsSatCount

  if(lastStatus.gpsHDOP != publishedStatus.gpsHDOP || msgGroup == 6)
    sprintf(message, "%sghp:%d,", message, publishedStatus.gpsHDOP); // gpsHDOP

  if(lastStatus.cellSignalStrength != publishedStatus.cellSignalStrength || msgGroup == 6)
    sprintf(message, "%scss:%d,", message, publishedStatus.cellSignalStrength); // cellSignalStrength

  if(lastStatus.gps3Dfix != publishedStatus.gps3Dfix || msgGroup == 6)
    sprintf(message, "%s3df:%d,", message, publishedStatus.gps3Dfix); // gps3Dfix

  if(lastStatus.isHardwareHealthy != publishedStatus.isHardwareHealthy || msgGroup == 7)
    sprintf(message, "%shwh:%d,", message, publishedStatus.isHardwareHealthy); // isHardwareHealthy

  if(lastStatus.uavIsArmed != publishedStatus.uavIsArmed || msgGroup == 7)
    sprintf(message, "%sarm:%d,", message, publishedStatus.uavIsArmed); // uavIsArmed

  if(lastStatus.downlinkStatus != publishedStatus.downlinkStatus || msgGroup == 7)
    sprintf(message, "%sdls:%d,", message, publishedStatus.downlinkStatus); // downlinkStatus

  if(lastStatus.mspRcOverride != publishedStatus.mspRcOverride || msgGroup == 7)
    sprintf(message, "%smro:%d,", message, publishedStatus.mspRcOverride); // mspRcOverride

  if(lastStatus.cmdRth != publishedStatus.cmdRth || msgGroup == 7)
    sprintf(message, "%scmdrth:%d,", message, publishedStatus.cmdRth);
  if(lastStatus.cmdAltHold != publishedStatus.cmdAltHold || msgGroup == 7)
    sprintf(message, "%scmdalt:%d,", message, publishedStatus.cmdAltHold);
  if(lastStatus.cmdCruise != publishedStatus.cmdCruise || msgGroup == 7)
    sprintf(message, "%scmdcrs:%d,", message, publishedStatus.cmdCruise);
  if(lastStatus.cmdBeeper != publishedStatus.cmdBeeper || msgGroup == 7)
    sprintf(message, "%scmdbep:%d,", message, publishedStatus.cmdBeeper);
  if(lastStatus.cmdWp != publishedStatus.cmdWp || msgGroup == 7)
    sprintf(message, "%scmdwp:%d,", message, publishedStatus.cmdWp);
  if(lastStatus.cmdPosHold != publishedStatus.cmdPosHold || msgGroup == 7)
    sprintf(message, "%scmdph:%d,", message, publishedStatus.cmdPosHold);
  if(lastStatus.fmCruise != publishedStatus.fmCruise || msgGroup == 7)
    sprintf(message, "%sfmcrs:%d,", message, publishedStatus.fmCruise);
  if(lastStatus.fmAltHold != publishedStatus.fmAltHold || msgGroup == 7)
    sprintf(message, "%sfmalt:%d,", message, publishedStatus.fmAltHold);
  if(lastStatus.fmWp != publishedStatus.fmWp || msgGroup == 7)
    sprintf(message, "%sfmwp:%d,", message, publishedStatus.fmWp);
  if(lastStatus.fmPosHold != publishedStatus.fmPosHold || msgGroup == 7)
    sprintf(message, "%sfmph:%d,", message, publishedStatus.fmPosHold);

  if(lastStatus.waypointCount != publishedStatus.waypointCount || msgGroup == 8)
    sprintf(message, "%swpc:%d,", message, publishedStatus.waypointCount); // waypointCount

  if(lastStatus.currentWaypointNumber != publishedStatus.currentWaypointNumber || msgGroup == 8)
    sprintf(message, "%scwn:%d,", message, publishedStatus.currentWaypointNumber); // currentWaypointNumber

  if(lastStatus.isWpMissionValid != publishedStatus.isWpMissionValid || msgGroup == 8)
    sprintf(message, "%swpv:%d,", message, publishedStatus.isWpMissionValid); // isWpMissionValid

  if(lastStatus.isFailsafeActive != publishedStatus.isFailsafeActive || msgGroup == 9)
    sprintf(message, "%sfs:%d,", message, publishedStatus.isFailsafeActive); // isFailsafeActive

  if(lastStatus.throttlePercent != publishedStatus.throttlePercent || msgGroup == 9)
    sprintf(message, "%strp:%d,", message, publishedStatus.throttlePercent); // throttlePercent

  if(lastStatus.autoThrottle != publishedStatus.autoThrottle || msgGroup == 9)
    sprintf(message, "%satt:%d,", message, publishedStatus.autoThrottle); // autoThrottle

  if(lastStatus.gpsGroundCourse != publishedStatus.gpsGroundCourse || msgGroup == 0)
    sprintf(message, "%sggc:%d,", message, publishedStatus.gpsGroundCourse); // gpsGroundCourse

  if(lastStatus.navState != publishedStatus.navState || msgGroup == 0)
    sprintf(message, "%snvs:%d,", message, publishedStatus.navState); // navState

  if(lastStatus.mWhDraw != publishedStatus.mWhDraw || msgGroup == 0)
    sprintf(message, "%swhd:%d,", message, publishedStatus.mWhDraw); // mWhDraw



  // This values will only be sent if changed... Otherwise they'll be sent by the Low priority message
  if(lastStatus.homeLatitude != publishedStatus.homeLatitude)
    sprintf(message, "%shla:%d,", message, publishedStatus.homeLatitude); // homeLatitude

  if(lastStatus.homeLongitude != publishedStatus.homeLongitude)
    sprintf(message, "%shlo:%d,", message, publishedStatus.homeLongitude); // homeLongitude

  if(lastStatus.homeAltitudeSL != publishedStatus.homeAltitudeSL)
    sprintf(message, "%shal:%d,", message, publishedStatus.homeAltitudeSL); // homeAltitudeSL

  if(lastStatus.flightModeId != publishedStatus.flightModeId)
    sprintf(message, "%sftm:%d,", message, publishedStatus.flightModeId); // flightModeId

  lastStatus = publishedStatus;
}

void buildLowPriorityMessage(char* message) {
  sprintf(message, ""); // messageCounter

  sprintf(message, "%spv:%d,", message, PROTOCOL_VERSION); // protocolVersion

  sprintf(message, "%sbcc:%d,", message, publishedStatus.batteryCellCount); // batteryCellCount

  sprintf(message, "%scs:%s,", message, publishedStatus.callsign); // callsign

  sprintf(message, "%shla:%d,", message, publishedStatus.homeLatitude); // homeLatitude

  sprintf(message, "%shlo:%d,", message, publishedStatus.homeLongitude); // homeLongitude

  sprintf(message, "%shal:%d,", message, publishedStatus.homeAltitudeSL); // homeAltitudeSL

  sprintf(message, "%sont:%d,", message, publishedStatus.onTime); // onTime

  sprintf(message, "%sflt:%d,", message, publishedStatus.flightTime); // flightTime

  sprintf(message, "%sftm:%d,", message, publishedStatus.flightModeId); // flightModeId

  sprintf(message, "%smfr:%d,", message, MESSAGE_SEND_INTERVAL); // mfr (message frequency)

  sprintf(message, "%sfcver:%d.%d.%d,", message, publishedStatus.fcVersionMajor, publishedStatus.fcVersionMinor, publishedStatus.fcVersionPatch); // FC firmware version

  sprintf(message, "%swpmax:%d,", message, publishedStatus.maxWaypoints); // max WPs supported by FC

  char pkBase64[45];
  base64Encode32(commandPublicKey, pkBase64);
  sprintf(message, "%spk:%s,", message, pkBase64); // public key (Ed25519, base64)
}

// Encode exactly 32 bytes as base64 (output must be at least 45 bytes: 44 chars + null).
// 32 bytes = 10 complete 3-byte groups (30 bytes) + 1 partial group of 2 bytes.
void base64Encode32(const uint8_t* input, char* output) {
  static const char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  int out = 0;
  for (int i = 0; i < 30; i += 3) {
    output[out++] = b64[input[i] >> 2];
    output[out++] = b64[((input[i] & 0x03) << 4) | (input[i+1] >> 4)];
    output[out++] = b64[((input[i+1] & 0x0f) << 2) | (input[i+2] >> 6)];
    output[out++] = b64[input[i+2] & 0x3f];
  }
  // Remaining 2 bytes (indices 30, 31)
  output[out++] = b64[input[30] >> 2];
  output[out++] = b64[((input[30] & 0x03) << 4) | (input[31] >> 4)];
  output[out++] = b64[(input[31] & 0x0f) << 2];
  output[out++] = '=';
  output[out]   = '\0';
}

void sendMessage(char* message) {
  if(client.publish(mqttUplinkTopic, message))
  {
      SerialMon.println(" Success!");
  }
  else
  {
    failureCounter++;
    SerialMon.println(" Error.");
  }
}

void connectToTheBroker()
{
  while (!client.connected())
  {
    SerialMon.println("Connecting to the MQTT broker...");
    char mqttClientId[32];
    snprintf(mqttClientId, sizeof(mqttClientId), "ESP32_%llX", ESP.getEfuseMac());
    if (client.connect(mqttClientId, mqttUser, mqttPassword))
    {
      SerialMon.println("Connected to the broker!");
      if (client.subscribe(mqttDownlinkTopic))
      {
        downlinkActive = true;
        SerialMon.print("Subscribed to command topic: ");
        SerialMon.println(mqttDownlinkTopic);
      }
      else
      {
        downlinkActive = false;
        SerialMon.println("Failed to subscribe to command topic.");
      }
    }
    else
    {
      SerialMon.print("Error connecting to the broker - State: ");
      SerialMon.println(client.state());
      failureCounter++;
      if(failureCounter>=10)
      {
        SerialMon.println("Too many fails connecting to Broker. Restarting...");
        ESP.restart();
      }
      delay(5000);
    }
  }
}
