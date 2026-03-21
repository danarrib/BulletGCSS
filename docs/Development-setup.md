# Development Setup

This guide explains how to set up a local development environment for the Bullet GCSS firmware using **PlatformIO**. It works on Windows, macOS, and Linux.

---

## Why PlatformIO?

PlatformIO replaces Arduino IDE for development. Key advantages:

- **Cross-platform** — same workflow on Windows, macOS, and Linux.
- **Dependency management** — libraries (`PubSubClient`, `TinyGSM`) are declared in `platformio.ini` and downloaded automatically. No manual library installation.
- **Multiple build targets** — separate environments for SIM7600 (4G) and SIM800 (2G) in a single config file.
- **CLI support** — works headlessly for CI/CD.

---

## Prerequisites

Install **Visual Studio Code** and the **PlatformIO IDE** extension.

### Step 1 — Install VS Code

Download and install VS Code from the [official website](https://code.visualstudio.com/).

### Step 2 — Install the PlatformIO extension

1. Open VS Code.
2. Click the **Extensions** icon in the left sidebar (or press `Ctrl+Shift+X` / `Cmd+Shift+X`).
3. Search for **PlatformIO IDE**.
4. Click **Install**.

PlatformIO will download the necessary toolchains automatically the first time you build.

---

## Opening the project

1. Open VS Code.
2. Go to **File → Open Folder…**
3. Navigate to the `ESP32-Modem/` folder inside the repository and open it.

PlatformIO will detect `platformio.ini` and configure the project automatically.

---

## Configuring credentials

Before building, edit `ESP32-Modem/Config.h` with your settings:

```c
// WiFi credentials (only needed if using USE_WIFI mode)
const char* ssid = "YourNetworkName";
const char* password = "YourPassword";

// MQTT broker
const char* mqttServer = "your.broker.address";
const int   mqttPort   = 1883;
const char* mqttUser   = "your_username";
const char* mqttPassword = "your_password";
const char* mqttTopic  = "bulletgcss/uavs/your_callsign";

// GPRS credentials (only needed for cellular modem)
const char apn[]      = "your.apn";
const char gprsUser[] = "";
const char gprsPass[] = "";
```

> **Do not commit `Config.h` with real credentials.** Keep your personal copy local.

---

## Build environments

`platformio.ini` defines two build environments:

| Environment | Modem | Board |
|---|---|---|
| `esp32-sim7600` | SIM7600 (4G LTE) — TTGO T-PCIE | ESP32 Dev Module |
| `esp32-sim800` | SIM800 (2G GPRS) — TTGO T-Call | ESP32 Dev Module |

The modem type is set automatically by the build environment via `build_flags`. You do **not** need to uncomment anything in `Config.h` when using PlatformIO.

---

## Building

### In VS Code

Click the **PlatformIO toolbar** at the bottom of the screen:

- **Build** (✓ checkmark) — compiles the firmware.
- **Upload** (→ arrow) — compiles and flashes to a connected ESP32.
- **Monitor** (plug icon) — opens the serial monitor at 115200 baud.

To select which environment to build, click the environment name in the status bar (bottom left).

### From the command line

```bash
# Build both environments
pio run

# Build only SIM7600
pio run -e esp32-sim7600

# Build only SIM800
pio run -e esp32-sim800

# Build and flash (replace esp32-sim7600 with your target)
pio run -e esp32-sim7600 --target upload

# Open serial monitor
pio device monitor
```

---

## Flashing the ESP32

1. Connect the ESP32 board to your computer via USB.
2. PlatformIO will detect the port automatically in most cases.
3. Click **Upload** in VS Code, or run `pio run -e esp32-sim7600 --target upload`.

If the port is not detected automatically, add it to `platformio.ini`:

```ini
[env:esp32-sim7600]
upload_port = /dev/ttyUSB0   ; Linux
; upload_port = /dev/cu.usbserial-0001  ; macOS
; upload_port = COM3                    ; Windows
```

---

## Serial monitor

After flashing, open the serial monitor to see debug output:

- **VS Code:** click the plug icon in the PlatformIO toolbar.
- **CLI:** `pio device monitor`
- **Baud rate:** 115200

Useful things to look for:
- `WiFi connected!` or `GPRS connected!` — network is up.
- `Connected to the broker!` — MQTT connection established.
- `MSP connected` — flight controller communication working.

---

## Arduino IDE (alternative)

If you prefer Arduino IDE:

1. Install Arduino IDE 2.x from [arduino.cc](https://www.arduino.cc/en/software).
2. Add ESP32 board support: go to **File → Preferences**, add this URL to "Additional boards manager URLs":
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. Go to **Tools → Board → Boards Manager**, search for `esp32`, install **esp32 by Espressif Systems**.
4. Install libraries via **Sketch → Include Library → Manage Libraries**:
   - `PubSubClient` by Nick O'Leary
   - `TinyGSM` by Volodymyr Shymanskyy
5. In `Config.h`, **uncomment** the line for your modem type:
   ```c
   // #define TINY_GSM_MODEM_SIM800   // T-Call board
   #define TINY_GSM_MODEM_SIM7600     // T-PCIE board ← uncomment yours
   ```
6. Board settings: **Tools → Board → ESP32 Arduino → DOIT ESP32 DEVKIT V1**, Upload Speed: 921600.

---

## Project structure

```
ESP32-Modem/
├── platformio.ini       # PlatformIO build config (environments, libraries)
├── Config.h             # User settings (credentials, modem type, intervals)
├── ESP32-Modem.ino      # Main sketch (FreeRTOS tasks, MQTT, network)
├── msp_library.h        # MSPv2 protocol interface
├── msp_library.cpp      # MSPv2 protocol implementation
└── uav_status.h         # UAV telemetry data struct
```
