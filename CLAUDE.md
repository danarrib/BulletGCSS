# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Bullet GCSS** (Ground Control Station System) is a web-based UAV ground control station that operates over cellular networks with no range limit. It is a PWA (no app installation required) that works cross-platform.

## Two-Component Architecture

### 1. ESP32-Modem (Embedded Firmware — [ESP32-Modem/](ESP32-Modem/))
Runs on an ESP32 board aboard the UAV. Communicates with the flight controller via the **MSP v2 protocol** (UART, 115200 baud on Serial2: TX18/RX19), then publishes telemetry as JSON to an **MQTT broker** via WiFi or a cellular modem (SIM800/SIM7600).

- [ESP32-Modem.ino](ESP32-Modem/ESP32-Modem.ino): Main sketch — two FreeRTOS tasks: `getTelemetryDataTask` (polls MSP at 200ms) and `sendMessageTask` (publishes MQTT at 1000ms).
- [Config.h](ESP32-Modem/Config.h): All user-configurable settings: modem type (`USE_WIFI` vs `TINY_GSM_MODEM_SIM7600`/`SIM800`), WiFi/GPRS credentials, MQTT broker/topic/credentials, serial pin assignments, polling intervals.
- [msp_library.h](ESP32-Modem/msp_library.h) / [msp_library.cpp](ESP32-Modem/msp_library.cpp): MSPv2 protocol implementation (header `$X<`, flags, messageID, payload, CRC8-DVB-S2). Defines all telemetry message types.
- [uav_status.h](ESP32-Modem/uav_status.h): Central `uav_status` struct holding all parsed telemetry (GPS, altitude, speed, battery, heading, flight mode, waypoints, callsign, etc.).

**Build:** Arduino IDE (board: DOIT ESP32 DEVKIT V1, Flash: 80MHz, Upload: 921600). Required libraries: `PubSubClient`, `TinyGSM`.

**Debug:** Monitor Serial at 115200 baud. Use `mosquitto_sub` or similar to inspect MQTT messages.

### 2. Web UI (Browser PWA — [UI/](UI/))
Static HTML/CSS/JS — no build step. Deployed directly to a web server.

- [basicui.html](UI/basicui.html): Single-page app entry point. Status icon bar (connection, signal, battery, GPS), three views: Data, Map, Sidebar/Settings.
- [js/CommScripts.js](UI/js/CommScripts.js): MQTT over WebSocket (Paho MQTT). Handles connect/subscribe/publish, message replay/logging, localStorage settings persistence.
- [js/MapScripts.js](UI/js/MapScripts.js): OpenLayers-based map. Renders aircraft icon, real-time position updates, waypoint visualization, track/manual pan modes.
- [js/EfisScripts.js](UI/js/EfisScripts.js): Artificial horizon, heading indicator, altitude/speed gauges — canvas-based EFIS instruments.
- [js/InfoPanelScripts.js](UI/js/InfoPanelScripts.js): Telemetry data panels (battery, GPS, navigation, flight times).
- [js/PageScripts.js](UI/js/PageScripts.js): Viewport, NoSleep, sidebar menu, unit conversion preferences (speed/altitude/distance).

**Key libraries bundled in repo:** OpenLayers ([UI/ol/](UI/ol/)), Paho MQTT JS, NoSleep, Open Location Code.

**Deployment:** GitHub Actions ([.github/workflows/](.github/workflows/)) — `rsync` to production server on push to `master`. No compilation, just file copy.

## Data Flow

```
Flight Controller (INAV/Betaflight)
  → MSP v2 (Serial2, 115200 baud)
  → ESP32: parse telemetry → uav_status struct → JSON
  → MQTT Publish (WiFi or cellular)
  → MQTT Broker (default: broker.emqx.io:8883, TLS encrypted)
  → Web UI: MQTT Subscribe over WebSocket
  → Update map, EFIS, info panels
```

## Configuration

All ESP32 settings live in [Config.h](ESP32-Modem/Config.h). To switch between WiFi and GPRS, toggle `#define USE_WIFI`. The MQTT topic format is `bulletgcss/uavs/<callsign>`.

Web UI settings (MQTT broker, topic, credentials, units) are persisted in browser `localStorage` — no server-side config file.

## Important Notes

- The MQTT broker is public by default (`broker.emqx.io`). Production deployments should use a private broker with authentication.
- The UI is designed mobile-first for outdoor use (dark theme, large touch targets).
- Multiple browser clients can simultaneously monitor the same aircraft by subscribing to the same MQTT topic.
- The PWA caches assets for offline use after first load.

## Git Rules

- **Never commit or push changes unless explicitly asked to do so.**

## Pending Work

Known issues, security concerns, and improvement opportunities are tracked in [docs/TODO.md](docs/TODO.md). Consult this file before starting new features to avoid duplicating effort or making changes that conflict with planned improvements.
