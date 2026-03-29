# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Bullet GCSS** (Ground Control Station System) is a web-based UAV ground control station that operates over cellular networks with no range limit. It is a PWA (no app installation required) that works cross-platform.

## Two-Component Architecture

### 1. ESP32-Modem (Embedded Firmware — [ESP32-Modem/](ESP32-Modem/))
Runs on an ESP32 board aboard the UAV. Communicates with the flight controller via the **MSP v2 protocol** (UART, 115200 baud on Serial2: TX18/RX19), then publishes telemetry as key:value messages to an **MQTT broker** via WiFi or a cellular modem (SIM800/SIM7600).

- [ESP32-Modem.cpp](ESP32-Modem/ESP32-Modem.cpp): Main source file. Runs two tasks: `fcTask` (FreeRTOS, priority 2, Core 1 — all MSP, ticks every 160 ms via `TASK_MSP_READ_MS`) and the Arduino loop task (MQTT publish, every 1000 ms). Key startup sequence on FC connection: `msp_get_boxnames()` → `msp_get_mode_ranges()` → `msp_get_override_channels()`. Per-cycle (every 160 ms): `msp_get_rc()` + `msp_send_rc_override()`. Telemetry split into 6 round-robin groups (one per cycle, ~960 ms full refresh). Slow-poll every 10 s: `msp_get_callsign()` + `get_all_waypoints()`.
- [Config.h](ESP32-Modem/Config.h): All user-configurable settings: modem type (`USE_WIFI` vs `TINY_GSM_MODEM_SIM7600`/`SIM800`), WiFi/GPRS credentials, MQTT broker/topic/credentials, serial pin assignments, polling intervals.
- [msp_library.h](ESP32-Modem/msp_library.h) / [msp_library.cpp](ESP32-Modem/msp_library.cpp): MSPv2 protocol implementation (header `$X<`, flags, messageID, payload, CRC8-DVB-S2). Defines all telemetry message types, structs (`modeRangeEntry_t`, `modeRangeInfo_t`, `FlightMode`, `msp_set_wp_t`, etc.), and permanent-ID constants (`MSP_PERM_ID_*`). Key MSP codes: `MSP_BOXNAMES` (116), `MSP_MODE_RANGES` (34), `MSP_RC` (105), `MSP_ACTIVEBOXES` (113), `MSP2_COMMON_SETTING` (0x1003), `MSP2_COMMON_SET_SETTING` (0x1004). The `requestWithResponse()` method supports requests with a separate send and receive buffer (needed for settings get/set). `modeRangeInfo_t` now includes `startPWM` and `endPWM` fields used by the gap-finder. `FlightMode` struct consolidates `range`, `available`, `active`, and `boxId` per commandable mode.
- [uav_status.h](ESP32-Modem/uav_status.h): Central `uav_status` struct holding all parsed telemetry (GPS, altitude, speed, battery, heading, flight mode, waypoints, callsign, downlink status, MSP RC override state, etc.).

**Build:** PlatformIO only (see [docs/Development-setup.md](docs/Development-setup.md)). Two environments: `esp32-sim800` (SIM800 2G) and `esp32-sim7600` (SIM7600 4G). Required libraries — managed automatically via `platformio.ini`: `PubSubClient`, `TinyGSM`, `SSLClient`, `Crypto` (Rhys Weatherley's arduinolibs — Ed25519 signature verification).

**Debug/Testing workflow:** New firmware features are validated by adding `SerialMon.printf()` calls, flashing, and reading the output. Because `pio device monitor` requires an interactive terminal, use Python to read the port directly:

```bash
# Kill any process holding the port, flash, then read output
fuser /dev/ttyUSB1 | xargs -r kill
pio run -e esp32-sim7600 --target upload --upload-port /dev/ttyUSB1

python3 -c "
import serial, time
s = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
start = time.time()
while time.time() - start < 25:
    line = s.readline()
    if line:
        print(line.decode('utf-8', errors='replace'), end='', flush=True)
s.close()
"
```

Use `grep -E "pattern"` to filter output for specific messages during verification.

### 2. Web UI (Browser PWA — [UI/](UI/))
Static HTML/CSS/JS — no build step. Deployed directly to a web server.

- [basicui.html](UI/basicui.html): Single-page app entry point. Status icon bar (connection, signal, battery, GPS), three views: Data, Map, Sidebar/Settings.
- [js/CommScripts.js](UI/js/CommScripts.js): MQTT over WebSocket (Paho MQTT). Handles connect/subscribe/publish, message replay/logging, localStorage settings persistence. Exposes session recording hooks (`setOnMessageCallback`, `setOnReplayStop`) and replay/restore functions for IndexedDB sessions.
- [js/SessionScripts.js](UI/js/SessionScripts.js): IndexedDB persistence layer for flight sessions. Two-store schema: `sessions` (metadata) and `session_messages` (log lines). Level 0 — no imports from other app modules.
- [js/MapScripts.js](UI/js/MapScripts.js): MapLibre GL JS-based vector map. Renders aircraft icon, real-time position updates, waypoint visualization, flight path, home point, user location. Supports runtime style switching between CARTO Dark Matter and OpenFreeMap Liberty styles (`setMapStyle`). Track/manual pan modes, compass control, center-on-aircraft button. All sizes scaled by `window.devicePixelRatio` for HiDPI displays.
- [js/EfisScripts.js](UI/js/EfisScripts.js): Artificial horizon, heading indicator, altitude/speed gauges — canvas-based EFIS instruments.
- [js/InfoPanelScripts.js](UI/js/InfoPanelScripts.js): Telemetry data panels (battery, GPS, navigation, flight times).
- [js/PageScripts.js](UI/js/PageScripts.js): Viewport, NoSleep, sidebar menu (with Settings submenu), unit conversion preferences (speed/altitude/distance). Async `DOMContentLoaded` initialises IndexedDB, restores open session state, wires session recording and replay-stop callbacks. Also owns the Security panel and the Commands panel — Ed25519 key pair generation (Web Crypto API), key storage in `localStorage`, key status display (5 states), and RC command sending with `rcCommands[]` state tracking. FAB button (`#fabCommands`) provides quick access to the commands panel.

**Key libraries bundled in repo:** Paho MQTT JS, NoSleep, Open Location Code. MapLibre GL JS is loaded from CDN (referenced in `basicui.html`). The old `UI/ol/` OpenLayers directory has been removed.

**Deployment:** GitHub Actions ([.github/workflows/](.github/workflows/)) — `rsync` to production server on push to `master`. No compilation, just file copy.

## Data Flow

```
Flight Controller (INAV/Betaflight)
  → MSP v2 (Serial2, 115200 baud)
  → ESP32: parse telemetry → uav_status struct → key:value message
  → MQTT Publish (WiFi or cellular)
  → MQTT Broker (default: broker.emqx.io:8883, TLS encrypted)
  → Web UI: MQTT Subscribe over WebSocket
  → Update map, EFIS, info panels
```

## Configuration

All ESP32 settings live in [Config.h](ESP32-Modem/Config.h). To switch between WiFi and GPRS, toggle `#define USE_WIFI`. Two MQTT topics are configured: uplink telemetry (`mqttUplinkTopic`, format `bulletgcss/telem/<callsign>`) and downlink commands (`mqttDownlinkTopic`, format `bulletgcss/cmd/<callsign>`).

Web UI settings (MQTT broker, topic, credentials, units) are persisted in browser `localStorage` — no server-side config file. Flight session data (telemetry log lines and session metadata) are persisted in browser `IndexedDB` via `SessionScripts.js`.

## Important Notes

- The MQTT broker is public by default (`broker.emqx.io`). Production deployments should use a private broker with authentication.
- All downlink commands are signed with Ed25519 (Web Crypto API in the UI, `arduinolibs` `Ed25519::verify()` in firmware). The operator generates a key pair in the UI Security panel, pastes the public key into `Config.h`, and re-flashes. The firmware rejects any command with a missing, invalid, or replayed signature. The last accepted sequence number is persisted to ESP32 NVS so replay protection survives reboots.
- **MSP RC Override:** Flight mode commands (RTH, altitude hold, cruise, etc.) work by overriding RC channel values via `MSP_SET_RAW_RC`. INAV requires the `MSP RC OVERRIDE` flight mode to be active on the FC for this to take effect. The firmware auto-discovers channel-to-mode mappings at startup (`MSP_MODE_RANGES`) and auto-configures `msp_override_channels` in INAV RAM — no manual INAV Configurator steps needed. The `mro` telemetry field is reported in every telemetry cycle. The UI merges the downlink status and MRO state into a single **Command Channel** icon with three states (error / warning / ok).
- **RC Channel Deactivation (gap-finder):** When deactivating a mode, the firmware cannot simply set the channel to 900 µs if another mode shares that channel. `findSafeOffValue(ch)` collects all PWM activation ranges mapped to the channel, sorts them, scans [900, 2100] for the largest uncovered gap, and uses the gap midpoint as the safe "off" value. This prevents inadvertently activating another mode.
- **Table-driven commands:** `cmdModes[]` is a 6-entry table (RTH, AltHold, Cruise, WP, Angle, Beeper), each a `FlightMode` struct. `msp_send_rc_override()` iterates this table every cycle — it sends the `onValue` for active commands or `findSafeOffValue()` for inactive ones.
- **FC cold-boot:** The firmware waits for the flight controller to become available, probing every 2 seconds. If MSP contact is lost mid-flight (1-second timeout), all startup flags reset and the full init sequence reruns automatically on reconnect.
- The UI is designed mobile-first for outdoor use (dark theme, large touch targets).
- Multiple browser clients can simultaneously monitor the same aircraft by subscribing to the same MQTT topic.
- The PWA caches assets for offline use after first load.

## Git Rules

- **Never commit or push changes unless explicitly asked to do so.**

## Pending Work

Known issues, security concerns, and improvement opportunities are tracked in [docs/TODO.md](docs/TODO.md). Consult this file before starting new features to avoid duplicating effort or making changes that conflict with planned improvements.
