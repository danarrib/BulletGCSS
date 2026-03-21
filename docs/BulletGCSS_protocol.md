# BulletGCSS Communication Protocol

This document describes the full communication protocol between the ESP32 modem (aircraft side) and the Web UI (ground side).

---

## Overview

Communication is **one-way**: the ESP32 publishes telemetry; the UI only subscribes and displays. No commands are ever sent from the UI to the aircraft.

```
Flight Controller  ──MSPv2 (UART)──►  ESP32 Modem  ──MQTT──►  MQTT Broker  ──MQTT──►  Web UI
```

---

## Transport Layer

| Parameter | ESP32 → Broker | Browser → Broker |
|---|---|---|
| Protocol | MQTT 3.1.1 | MQTT 3.1.1 over WebSocket |
| Default port | 1883 (plaintext) | 8084 (WSS/TLS) |
| Default broker | `broker.emqx.io` | `broker.emqx.io` |
| QoS | 0 (fire and forget) | 0 (subscribe) |
| Client ID | `ESP32_<MAC>` (chip MAC address) | `web_<random>` |

**Topic format:** `bulletgcss/uavs/<callsign>`

The callsign is read from the flight controller at startup via MSP and used as the topic suffix. The UI subscribes to the configured topic stored in browser `localStorage`.

---

## Message Format

All messages are plain ASCII text. Each message is a comma-separated list of `key:value` pairs, always ending with a trailing comma:

```
key1:value1,key2:value2,key3:value3,
```

Values are always integers. Floating-point quantities are transmitted as scaled integers (see the field reference below for each field's scale factor).

There is no message envelope, no length prefix, and no message-level checksum. Each MQTT publish is one complete message.

---

## Message Types

There are four distinct message types, distinguished by their content.

### 1. Session Start Message

Sent **once**, as the very first message after the ESP32 connects to the broker.

```
id:0,
```

This signals to any subscriber that a new session has begun.

---

### 2. Standard Telemetry Message

Sent **every 1000 ms** (configurable via `MESSAGE_SEND_INTERVAL` in `Config.h`).

To reduce bandwidth, a field is only included in a message if its value has changed since the last message. However, to prevent stale data in the UI (e.g. after the UI reconnects mid-flight), all fields are **forced into the message periodically**, regardless of whether they changed.

Fields are divided into 10 groups (0–9). On each cycle, `msgCounter % 10` determines which group is force-refreshed. This means every field is guaranteed to be re-sent at least once every 10 seconds.

| Group | Fields force-refreshed |
|---|---|
| 0 | `ran`, `pan`, `hea`, `ggc`, `nvs`, `whd` |
| 1 | `asl`, `alt`, `gsp` |
| 2 | `vsp`, `hdr`, `hds` |
| 3 | `acv`, `bpv`, `bfp` |
| 4 | `cud`, `cad`, `rsi` |
| 5 | `gla`, `glo`, `gsc` |
| 6 | `ghp`, `css`, `3df` |
| 7 | `hwh`, `arm` |
| 8 | `wpc`, `cwn`, `wpv` |
| 9 | `fs`, `trp`, `att` |

The following fields are **only sent when changed** and are never force-refreshed here (they appear in the Low Priority Message instead):

`hla`, `hlo`, `hal`, `ftm`

---

### 3. Low Priority Message

Sent **every 60 seconds** (`LOW_PRIORITY_MESSAGE_INTERVAL` in `Config.h`), always including all fields regardless of change.

Contains slow-changing or static data: home coordinates, cell count, callsign, flight times, and message frequency.

```
bcc:4,cs:MyCallsign,hla:123456789,hlo:-456789012,hal:80000,ont:3600,flt:1200,ftm:9,mfr:1000,
```

---

### 4. Waypoint Message

Sent **every 30 cycles** (every ~30 seconds) when the aircraft has a waypoint mission loaded (`waypointCount > 0`).

One MQTT message is published **per waypoint**, including waypoint 0 (the home point). Waypoint messages are identified by starting with `wpno:`.

```
wpno:1,la:123456789,lo:-456789012,al:5000,ac:1,p1:100,
```

Optional fields (`p1`, `p2`, `p3`, `f`) are omitted when their value is 0.

---

## Standard Telemetry Field Reference

### Attitude

| Key | Description | Unit / Scale | JS field |
|---|---|---|---|
| `ran` | Roll angle | Decidegrees (÷10 → degrees) | `data.rollAngle` |
| `pan` | Pitch angle | Decidegrees (÷10 → degrees) | `data.pitchAngle` |
| `hea` | Heading (yaw) | Degrees | `data.heading` |
| `ggc` | GPS ground course | Degrees | `data.gpsGroundCourse` |

### Altitude & Speed

| Key | Description | Unit / Scale | JS field |
|---|---|---|---|
| `alt` | Relative altitude (barometric/estimated) | Centimeters | `data.altitude` |
| `asl` | Altitude above sea level (GPS) | Meters | `data.altitudeSeaLevel` |
| `gsp` | Ground speed | cm/s | `data.groundSpeed` |
| `vsp` | Vertical speed | cm/s | `data.verticalSpeed` |

### GPS

| Key | Description | Unit / Scale | JS field |
|---|---|---|---|
| `gla` | GPS latitude | Degrees × 10,000,000 | `data.gpsLatitude` |
| `glo` | GPS longitude | Degrees × 10,000,000 | `data.gpsLongitude` |
| `gsc` | GPS satellite count | Count | `data.gpsSatCount` |
| `ghp` | GPS HDOP | HDOP × 100 (÷100 → HDOP) | `data.gpsHDOP` |
| `3df` | GPS 3D fix | `0` = no fix, `1` = 3D fix | `data.gps3DFix` |

### Navigation

| Key | Description | Unit / Scale | JS field |
|---|---|---|---|
| `hdr` | Direction to home | Degrees | `data.homeDirection` |
| `hds` | Distance to home | Meters | `data.homeDistance` |
| `nvs` | Navigation state (INAV nav_state enum) | Integer | `data.navState` |
| `cwn` | Current active waypoint number | Count | `data.currentWaypointNumber` |
| `wpc` | Total waypoint count in mission | Count | `data.waypointCount` |
| `wpv` | Waypoint mission valid flag | `0` / `1` | `data.isWaypointMissionValid` |

### Battery & Power

| Key | Description | Unit / Scale | JS field |
|---|---|---|---|
| `bpv` | Total battery voltage | Centivolts (÷100 → V) | `data.batteryVoltage` |
| `acv` | Average cell voltage | Centivolts (÷100 → V) | `data.battCellVoltage` |
| `bfp` | Battery fuel (charge) percent | Percent | `data.fuelPercent` |
| `cud` | Current draw | Centiamps (÷100 → A) | `data.currentDraw` |
| `cad` | Capacity drawn | mAh | `data.capacityDraw` |
| `whd` | Energy drawn | mWh | `data.mWhDraw` |
| `trp` | Throttle percent | Percent | `data.throttlePercent` |
| `att` | Auto-throttle active | `0` / `1` | `data.isAutoThrottleActive` |

### Status Flags

| Key | Description | Values | JS field |
|---|---|---|---|
| `arm` | Aircraft armed | `0` / `1` | `data.uavIsArmed` |
| `fs` | Failsafe active | `0` / `1` | `data.isFailsafeActive` |
| `hwh` | Hardware healthy | `0` / `1` | `data.isHardwareHealthy` |
| `css` | Cellular/WiFi signal strength | `0`–`3` | `data.cellSignalStrength` |
| `rsi` | RC link RSSI | Percent | `data.rssiPercent` |

### Flight Mode

| Key | Description | JS field |
|---|---|---|
| `ftm` | Flight mode ID (see table below) | `data.flightMode` (resolved to name) |

Flight mode ID values:

| ID | Name | Description |
|---|---|---|
| 1 | `MANUAL` | Manual / passthrough |
| 2 | `RTH` | Return to home |
| 3 | `A+PH` | Altitude hold + position hold |
| 4 | `POS H` | Position hold only |
| 5 | `3CRS` | 3D cruise with altitude hold |
| 6 | `CRS` | Cruise |
| 7 | `WP` | Waypoint mission active |
| 8 | `ALT H` | Altitude hold + angle |
| 9 | `ANGLE` | Angle (self-leveling) |
| 10 | `HORIZON` | Horizon mode |
| 11 | `ACRO` | Acro / rate mode |

---

## Low Priority Message Field Reference

| Key | Description | Unit / Scale | JS field |
|---|---|---|---|
| `bcc` | Battery cell count | Count | `data.batteryCellCount` |
| `cs` | Aircraft callsign | String (alphanumeric, `_`, `-`) | `data.callsign` |
| `hla` | Home latitude | Degrees × 10,000,000 | `data.homeLatitude` |
| `hlo` | Home longitude | Degrees × 10,000,000 | `data.homeLongitude` |
| `hal` | Home altitude above sea level | Centimeters (÷100 → m) | `data.homeAltitudeSL` |
| `ont` | On-time (time since power on) | Seconds | `data.powerTime` |
| `flt` | Flight time (time since arm) | Seconds | `data.flightTime` |
| `ftm` | Flight mode ID | See flight mode table | `data.flightMode` |
| `mfr` | Message frequency (send interval) | Milliseconds | `pageSettings.messageInterval` |

---

## Waypoint Message Field Reference

Waypoint messages start with `wpno:` and are parsed separately from telemetry messages.

| Key | Description | Unit / Scale |
|---|---|---|
| `wpno` | Waypoint number (`0` = home point) | Count |
| `la` | Waypoint latitude | Degrees × 10,000,000 |
| `lo` | Waypoint longitude | Degrees × 10,000,000 |
| `al` | Waypoint altitude | Centimeters |
| `ac` | Waypoint action (INAV action enum) | Integer |
| `p1` | Parameter 1 (omitted if 0) | Integer |
| `p2` | Parameter 2 (omitted if 0) | Integer |
| `p3` | Parameter 3 (omitted if 0) | Integer |
| `f` | Flag (omitted if 0) | Integer |

---

## Timing Summary

| Message type | Interval |
|---|---|
| Standard telemetry | 1000 ms (default) |
| Low priority | 60 s |
| Waypoint mission | Every 30 telemetry cycles (~30 s) |
| Telemetry fetch from FC | 200 ms (5× per send cycle) |

---

## MSP Layer (ESP32 ↔ Flight Controller)

The ESP32 communicates with the flight controller using **MSPv2** (MultiWii Serial Protocol v2) over UART at 115200 baud (Serial2: RX=GPIO19, TX=GPIO18).

Each telemetry cycle the ESP32 requests the following MSP messages from the FC:

| MSP Message | Content |
|---|---|
| `MSP_RAW_GPS` | GPS coordinates, speed, fix type, satellite count, HDOP |
| `MSP_COMP_GPS` | Distance and direction to home |
| `MSP_ATTITUDE` | Roll, pitch, yaw (heading) |
| `MSP_ALTITUDE` | Estimated altitude, vertical speed |
| `MSP_SENSOR_STATUS` | Hardware health flag |
| `MSP_ACTIVEBOXES` | Active flight mode bitmask |
| `MSP_WP_GETINFO` | Waypoint count and mission validity |
| `MSP_NAV_STATUS` | Nav state, active waypoint number |
| `MSP2_INAV_MISC2` | On-time, flight time, throttle, auto-throttle |
| `MSP2_INAV_ANALOG` | Battery voltage, current, RSSI, fuel percent |
| `MSP_BOXNAMES` | Flight mode names (fetched once at startup) |
| `MSP_NAME` | Aircraft callsign (fetched once at startup) |
| `MSP_WP` | Individual waypoint data (polled every 10 cycles) |

MSPv2 frame format: `$X<` header, 1-byte flags, 2-byte message ID, 2-byte payload length, payload, 1-byte CRC8-DVB-S2 checksum.
