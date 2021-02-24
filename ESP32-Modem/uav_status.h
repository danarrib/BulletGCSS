/*
  uav_status.h

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

#pragma once

#ifndef UAV_STATUS_H
#define UAV_STATUS_H

#include <Arduino.h>

struct uav_status
{
    // High priority information
    int32_t gpsLatitude;
    int32_t gpsLongitude;
    int16_t altitudeSeaLevel;
    int32_t altitude;
    int16_t groundSpeed;
    int16_t heading;
    int16_t gpsGroundCourse;
    int16_t rollAngle;
    int16_t pitchAngle;
    int16_t verticalSpeed;
    uint16_t batteryVoltage;
    int16_t currentDraw;
    uint8_t gps3Dfix;
    uint8_t isHardwareHealthy;
    uint8_t uavIsArmed;
    uint8_t isFailsafeActive;
    uint16_t averageCellVoltage;

    // Normal priority information
    uint8_t gpsSatCount;
    int16_t gpsHDOP;
    int16_t homeDistance;
    int16_t homeDirection;
    int32_t capacityDraw;
    int32_t mWhDraw;
    uint8_t rssiPercent;
    uint8_t fuelPercent;
    uint8_t batteryCellCount;
    uint8_t cellSignalStrength;
    uint8_t flightModeId;
    uint8_t waypointCount;
    uint8_t currentWaypointNumber;
    uint8_t throttlePercent;
    uint8_t autoThrottle; 
    uint8_t navState;
    uint8_t isWpMissionValid; 

    // Low priority information
    uint32_t onTime; 
    uint32_t flightTime; 
    char callsign[17];
    int32_t homeLatitude;
    int32_t homeLongitude;
    int32_t homeAltitudeSL;
    
} __attribute__((packed));




#endif
