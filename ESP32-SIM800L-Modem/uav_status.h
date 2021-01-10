/*
  uav_status.h

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

#pragma once

#ifndef UAV_STATUS_H
#define UAV_STATUS_H

#include <Arduino.h>

struct uav_status
{
    uint8_t gpsSatCount;
    double gpsHDOP;
    double gpsLatitude;
    double gpsLongitude;
    int16_t altitudeSeaLevel;
    int16_t altitude;
    int16_t groundSpeed;
    int16_t heading;
    int16_t homeDistance;
    int16_t homeDirection;
    int16_t rollAngle;
    int16_t pitchAngle;
    int16_t verticalSpeed;
    double batteryVoltage;
    double currentDraw;
    int16_t capacityDraw;
    uint8_t rssiPercent;
    uint8_t fuelPercent;
    double averageCellVoltage;
    uint8_t batteryCellCount;
    uint8_t cellSignalStrength;
    uint8_t gps3Dfix;
    uint8_t isHardwareHealthy;
    uint8_t uavIsArmed;
    uint8_t isFailsafeActive;
    String flightMode;
    uint8_t isWpMissionValid; 
    uint8_t waypointCount;
    uint8_t currentWaypointNumber;
    String callsign;
    double homeLatitude;
    double homeLongitude;
    int16_t homeAltitudeSL;
    
} __attribute__((packed));

#endif
