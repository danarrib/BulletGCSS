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

**QoS note:** All messages use QoS 0 (fire-and-forget) by design. Cellular coverage is inherently intermittent — aircraft routinely fly beyond antenna range — and operators already expect gaps in telemetry. A dropped packet means a one-second stale display at most; the 10-second force-refresh cycle (see Standard Telemetry Message below) re-syncs all fields automatically when connectivity returns. The UI's stale-data indicator signals loss of connection; this is normal operating behaviour, not an error condition.

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

| Key | Description | Unit / Scale | JS field | Valid Range |
|---|---|---|---|---|
| `ran` | Roll angle | Decidegrees (÷10 → degrees) | `data.rollAngle` | -1800 to 1800 |
| `pan` | Pitch angle | Decidegrees (÷10 → degrees) | `data.pitchAngle` | -900 to 900 |
| `hea` | Heading (yaw) | Degrees | `data.heading` | 0 to 359 |
| `ggc` | GPS ground course | Degrees | `data.gpsGroundCourse` | 0 to 359 |

### Altitude & Speed

| Key | Description | Unit / Scale | JS field | Valid Range |
|---|---|---|---|---|
| `alt` | Relative altitude (barometric/estimated) | Centimeters | `data.altitude` | -1000000 to 10000000 |
| `asl` | Altitude above sea level (GPS) | Meters | `data.altitudeSeaLevel` | -500 to 9000 |
| `gsp` | Ground speed | cm/s | `data.groundSpeed` | 0 to 15000 |
| `vsp` | Vertical speed | cm/s | `data.verticalSpeed` | -60000 to 60000 |

> `alt` range covers 10 km below home to 100 km relative altitude.
> `asl` range covers below-sea-level terrain to above the highest mountain.
> `gsp` ceiling of 15000 cm/s = 540 km/h, well above any fixed-wing UAV speed.
> `vsp` allows ±2160 km/h vertical.

### GPS

| Key | Description | Unit / Scale | JS field | Valid Range |
|---|---|---|---|---|
| `gla` | GPS latitude | Degrees × 10,000,000 | `data.gpsLatitude` | -900000000 to 900000000 |
| `glo` | GPS longitude | Degrees × 10,000,000 | `data.gpsLongitude` | -1800000000 to 1800000000 |
| `gsc` | GPS satellite count | Count | `data.gpsSatCount` | 0 to 50 |
| `ghp` | GPS HDOP | HDOP × 100 (÷100 → HDOP) | `data.gpsHDOP` | 0 to 9999 |
| `3df` | GPS 3D fix | `0` = no fix, `1` = 3D fix | `data.gps3DFix` | 0 or 1 |

### Navigation

| Key | Description | Unit / Scale | JS field | Valid Range |
|---|---|---|---|---|
| `hdr` | Direction to home | Degrees | `data.homeDirection` | 0 to 359 |
| `hds` | Distance to home | Meters | `data.homeDistance` | 0 to 20000000 |
| `nvs` | Navigation state (INAV nav_state enum) | Integer | `data.navState` | 0 to 30 |
| `cwn` | Current active waypoint number | Count | `data.currentWaypointNumber` | 0 to 255 |
| `wpc` | Total waypoint count in mission | Count | `data.waypointCount` | 0 to 256 |
| `wpv` | Waypoint mission valid flag | `0` / `1` | `data.isWaypointMissionValid` | 0 or 1 |

> `hds` ceiling of 20000 km ≈ half the Earth's circumference.
> `nvs` ceiling of 30 is intentionally generous — INAV's nav_state enum currently tops out around 20.

### Battery & Power

| Key | Description | Unit / Scale | JS field | Valid Range |
|---|---|---|---|---|
| `bpv` | Total battery voltage | Centivolts (÷100 → V) | `data.batteryVoltage` | 0 to 6000 |
| `acv` | Average cell voltage | Centivolts (÷100 → V) | `data.battCellVoltage` | 0 to 500 |
| `bfp` | Battery fuel (charge) percent | Percent | `data.fuelPercent` | 0 to 100 |
| `cud` | Current draw | Centiamps (÷100 → A) | `data.currentDraw` | 0 to 50000 |
| `cad` | Capacity drawn | mAh | `data.capacityDraw` | 0 to 100000 |
| `whd` | Energy drawn | mWh | `data.mWhDraw` | 0 to 1000000 |
| `trp` | Throttle percent | Percent | `data.throttlePercent` | 0 to 100 |
| `att` | Auto-throttle active | `0` / `1` | `data.isAutoThrottleActive` | 0 or 1 |

> `bpv` ceiling of 6000 cV = 60 V covers up to 14S LiPo configurations.
> `acv` ceiling of 500 cV = 5.0 V per cell covers HV LiPo (4.35 V) with margin.
> `cud` ceiling of 50000 cA = 500 A accommodates very high-current setups.

### Status Flags

| Key | Description | Values | JS field | Valid Range |
|---|---|---|---|---|
| `arm` | Aircraft armed | `0` / `1` | `data.uavIsArmed` | 0 or 1 |
| `fs` | Failsafe active | `0` / `1` | `data.isFailsafeActive` | 0 or 1 |
| `hwh` | Hardware healthy | `0` / `1` | `data.isHardwareHealthy` | 0 or 1 |
| `css` | Cellular/WiFi signal strength | `0`–`3` | `data.cellSignalStrength` | 0 to 3 |
| `rsi` | RC link RSSI | Percent | `data.rssiPercent` | 0 to 100 |

### Flight Mode

| Key | Description | JS field | Valid Range |
|---|---|---|---|
| `ftm` | Flight mode ID (see table below) | `data.flightMode` (resolved to name) | 1 to 11 |

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

| Key | Description | Unit / Scale | JS field | Valid Range |
|---|---|---|---|---|
| `bcc` | Battery cell count | Count | `data.batteryCellCount` | 1 to 12 |
| `cs` | Aircraft callsign | String (alphanumeric, `_`, `-`) | `data.callsign` | 1–16 chars; pattern `^[A-Za-z0-9_-]+$` |
| `hla` | Home latitude | Degrees × 10,000,000 | `data.homeLatitude` | -900000000 to 900000000 |
| `hlo` | Home longitude | Degrees × 10,000,000 | `data.homeLongitude` | -1800000000 to 1800000000 |
| `hal` | Home altitude above sea level | Centimeters (÷100 → m) | `data.homeAltitudeSL` | -50000 to 900000 |
| `ont` | On-time (time since power on) | Seconds | `data.powerTime` | 0 to 172800 |
| `flt` | Flight time (time since arm) | Seconds | `data.flightTime` | 0 to 86400 |
| `ftm` | Flight mode ID | See flight mode table | `data.flightMode` | 1 to 11 |
| `mfr` | Message frequency (send interval) | Milliseconds | `pageSettings.messageInterval` | 100 to 10000 |

> `hal` range covers Dead Sea (-430 m = -43000 cm) to above Everest (8849 m = 884900 cm), rounded to safe integers.
> `ont` ceiling of 172800 s = 48 h. `flt` ceiling of 86400 s = 24 h.
> `mfr` clamped to 100–10000 ms to prevent the UI from interpreting implausibly fast or slow rates.

---

## Waypoint Message Field Reference

Waypoint messages start with `wpno:` and are parsed separately from telemetry messages.

| Key | Description | Unit / Scale | Valid Range |
|---|---|---|---|
| `wpno` | Waypoint number (`0` = home point) | Count | 0 to 255 |
| `la` | Waypoint latitude | Degrees × 10,000,000 | -900000000 to 900000000 |
| `lo` | Waypoint longitude | Degrees × 10,000,000 | -1800000000 to 1800000000 |
| `al` | Waypoint altitude | Centimeters | 0 to 60000 |
| `ac` | Waypoint action (INAV action enum) | Integer | 1 to 8 |
| `p1` | Parameter 1 (omitted if 0) | Integer | -32768 to 32767 |
| `p2` | Parameter 2 (omitted if 0) | Integer | -32768 to 32767 |
| `p3` | Parameter 3 (omitted if 0) | Integer | -32768 to 32767 |
| `f` | Flag (omitted if 0) | Integer | 0 to 255 |

> `al` ceiling of 60000 cm = 600 m is the typical INAV waypoint altitude limit.
> `ac` values: 1=WAYPOINT, 2=POSHOLD_UNLIM, 3=POSHOLD_TIME, 4=RTH, 5=SET_POI, 6=JUMP, 7=SET_HEAD, 8=LAND.
> `p1`/`p2`/`p3` are signed 16-bit integers as defined by the MSPv2 waypoint payload.

---

## Validation Strategy

When implementing input validation in the UI parser (`CommScripts.js`), apply the following rules consistently:

- **Out-of-range numeric fields:** silently discard the field (leave the previous value unchanged). Do not clamp — a clamped value could be misleading (e.g. showing 100 m altitude when the message contained 999999 m).
- **Boolean / flag fields (0 or 1):** discard the message field if the parsed value is anything other than 0 or 1.
- **Enum fields (`nvs`, `ac`, `ftm`):** discard values outside the documented range.
- **String fields (`cs`):** reject the callsign if it does not match `^[A-Za-z0-9_-]+$` or exceeds 16 characters. Fall back to the previously displayed callsign.
- **GPS coordinates:** both latitude and longitude must be within range simultaneously. If either is invalid, discard both (a position with only one valid coordinate is unusable and could plot to the wrong location).
- **`parseInt` / `parseFloat` failures:** `NaN` results must be treated as invalid and discarded — never passed to the display logic.

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
