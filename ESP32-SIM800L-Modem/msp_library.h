/*
  msp_library.h

  This is a MSPv2 Library, made for Bullet Ground Control Station System
  https://github.com/danarrib/BulletGCSS
  
  Written by Daniel Ribeiro - https://github.com/danarrib
  
  With the valuable help of:
    Michel Pastor - https://github.com/shellixyz
    Paweł Spychalski - https://github.com/DzikuVx
    Olivier C. - https://github.com/OlivierC-FR

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

#pragma once

#ifndef MSP_LIBRARY_H
#define MSP_LIBRARY_H

#include <Arduino.h>
#include <Stream.h>

#define MSP_NAME          10
#define MSP_WP_GETINFO    20
#define MSP_RAW_GPS       106
#define MSP_COMP_GPS      107
#define MSP_ATTITUDE      108
#define MSP_ALTITUDE      109
#define MSP_ACTIVEBOXES   113
#define MSP_BOXNAMES      116
#define MSP_NAV_STATUS    121
#define MSP_SENSOR_STATUS 151
#define MSP_SET_WP        209
#define MSP2_INAV_ANALOG  0x2002

// This enum is a copy from INAV one in "src/main/fc/rc_modes.h"
typedef enum {
    BOXARM           = 0,
    BOXANGLE         = 1,
    BOXHORIZON       = 2,
    BOXNAVALTHOLD    = 3,    // old BOXBARO
    BOXHEADINGHOLD   = 4,    // old MAG
    BOXHEADFREE      = 5,
    BOXHEADADJ       = 6,
    BOXCAMSTAB       = 7,
    BOXNAVRTH        = 8,    // old GPSHOME
    BOXNAVPOSHOLD    = 9,    // old GPSHOLD
    BOXMANUAL        = 10,
    BOXBEEPERON      = 11,
    BOXLEDLOW        = 12,
    BOXLIGHTS        = 13,
    BOXNAVLAUNCH     = 14,
    BOXOSD           = 15,
    BOXTELEMETRY     = 16,
    BOXBLACKBOX      = 17,
    BOXFAILSAFE      = 18,
    BOXNAVWP         = 19,
    BOXAIRMODE       = 20,
    BOXHOMERESET     = 21,
    BOXGCSNAV        = 22,
    BOXKILLSWITCH    = 23,   // old HEADING LOCK
    BOXSURFACE       = 24,
    BOXFLAPERON      = 25,
    BOXTURNASSIST    = 26,
    BOXAUTOTRIM      = 27,
    BOXAUTOTUNE      = 28,
    BOXCAMERA1       = 29,
    BOXCAMERA2       = 30,
    BOXCAMERA3       = 31,
    BOXOSDALT1       = 32,
    BOXOSDALT2       = 33,
    BOXOSDALT3       = 34,
    BOXNAVCRUISE     = 35,
    BOXBRAKING       = 36,
    BOXUSER1         = 37,
    BOXUSER2         = 38,
    BOXFPVANGLEMIX   = 39,
    BOXLOITERDIRCHN  = 40,
    BOXMSPRCOVERRIDE = 41,
    CHECKBOX_ITEM_COUNT
} boxId_e;

typedef uint32_t bitarrayElement_t;
#define BITARRAY_DECLARE(name, bits) bitarrayElement_t name[(bits + 31) / 32]
typedef struct boxBitmask_s { BITARRAY_DECLARE(bits, CHECKBOX_ITEM_COUNT); } boxBitmask_t;

struct MSP_WP_GETINFO_t
{
    uint8_t dummy;
    uint8_t maxWaypoints;
    uint8_t isWaypointListValid;
    uint8_t waypointCount;
} __attribute__((packed));

struct MSP_NAV_STATUS_t
{
    uint8_t mode;   // 0 = None, 1 = Hold, 2 = RTH, 3 = NAV, 15 = EMERG
    uint8_t state;  // 0 = None, 1 = RTH Start, 2 = RTH ENROUTE, 3 = HOLD INF, 4 = HOLD TIMED, 5 = WP ENROUTE
                    // 6 = Process Next, 7 = Jump, 8 = Land Start, 9 = Land In Progress, 10 = Landed,
                    // 11 = Land Settle, 12 = Land Start Descent, 13 = Hover Above Home, 14 = Emerg Landing
    uint8_t activeWpAction; 
    uint8_t activeWpNumber; 
    uint8_t error;  // 0 = None, 1 = Too Far, 2 = Spoiled GPS, 3 = WP CRC, 4 = Finished, 5 = Time wait, 6 = Invalid Jump, 
                    // 7 = Invalid Data, 8 = Wait RTH altitude, 9 = GPS Fix Loss, 10 = Disarm, 11 = Landing
    int16_t HeadingHoldTarget; 
} __attribute__((packed));

struct MSP_SENSOR_STATUS_t
{
    uint8_t isHardwareHealthy; // 0 or 1
    uint8_t HwGyroStatus; // 0 = Not selected, 1 = OK, 2 = Unavailable, 3 = Unhealthy
    uint8_t HwAccelerometerStatus; 
    uint8_t HwCompassStatus; 
    uint8_t HwBarometerStatus; 
    uint8_t HwGPSStatus; 
    uint8_t HwRangefinderStatus; 
    uint8_t HwPitotmeterStatus; 
    uint8_t HwOpticalFlowStatus; 
} __attribute__((packed));

struct MSP2_INAV_ANALOG_t
{
    uint8_t batteryStats; 
    uint16_t batteryVoltage;
    int16_t currentDraw;
    int32_t powerDraw;
    int32_t mAhDraw;
    int32_t mWhDraw;
    uint32_t remainingCapacity;
    uint8_t batteryPercentage; 
    uint16_t rssi; 
} __attribute__((packed));

struct MSP_COMP_GPS_t
{
    int16_t distanceToHome;  // distance to home in meters
    int16_t directionToHome; // direction to home in degrees
    uint8_t heartbeat;       // toggles 0 and 1 for each change
} __attribute__((packed));

struct MSP_RAW_GPS_t
{
    uint8_t fixType; // MSP_GPS_NO_FIX, MSP_GPS_FIX_2D, MSP_GPS_FIX_3D
    uint8_t numSat;
    int32_t lat;          // 1 / 10000000 deg
    int32_t lon;          // 1 / 10000000 deg
    int16_t alt;          // meters
    int16_t groundSpeed;  // cm/s
    int16_t groundCourse; // decidegrees
    int16_t hdop;         // 1 / 100
} __attribute__((packed));

struct MSP_ATTITUDE_t {
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
} __attribute__ ((packed));

struct MSP_ALTITUDE_t
{
    int32_t estimatedAltitude;
    int16_t estimatedVelocity;
    int32_t baroAltitude;
} __attribute__((packed));

// MSP_SET_WP command
// Special waypoints are 0 and 255. 0 is the RTH position, 255 is the POSHOLD position (lat, lon, alt).
struct msp_set_wp_t
{
    uint8_t waypointNumber;
    uint8_t action; // one of MSP_NAV_STATUS_WAYPOINT_ACTION_XXX
    int32_t lat;    // decimal degrees latitude * 10000000
    int32_t lon;    // decimal degrees longitude * 10000000
    int32_t alt;    // altitude (cm)
    int16_t p1;     // speed (cm/s) when action is MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT, or "land" (value 1) when action is MSP_NAV_STATUS_WAYPOINT_ACTION_RTH
    int16_t p2;     // not used
    int16_t p3;     // not used
    uint8_t flag;   // 0xa5 = last, otherwise set to 0
} __attribute__((packed));

class MSPLibrary
{
public:
    void begin(Stream &stream, uint32_t timeout);
    void reset();

    uint8_t crc8_dvb_s2(uint8_t crc, byte a);
    void send(uint16_t messageID, void * payload, uint16_t size);
    bool recv(uint16_t * messageID, void * payload, uint16_t maxSize, uint16_t * recvSize);
    bool waitFor(uint16_t messageID, void * payload, uint16_t maxSize, uint16_t * recvSize = NULL);
    bool command(uint16_t messageID, void * payload, uint16_t size, bool waitACK = true);
    bool request(uint16_t messageID, void *payload, uint16_t maxSize, uint16_t *recvSize = NULL);

    bool requestText(uint16_t messageID, void * payload, uint16_t *recvSize);
    bool waitForText(uint16_t messageID, void * payload, uint16_t *recvSize);
    bool recvText(uint16_t * messageID, void * dst, uint16_t *recvSize);

private:
    Stream *_stream;
    uint32_t _timeout;
};

#endif
