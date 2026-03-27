# Development Setup

This guide explains how to set up a local development environment for the Bullet GCSS firmware using **PlatformIO**. It works on Windows, macOS, and Linux.

---

## Why PlatformIO?

PlatformIO replaces Arduino IDE for development. Key advantages:

- **Cross-platform** — same workflow on Windows, macOS, and Linux.
- **Dependency management** — libraries (`PubSubClient`, `TinyGSM`, `SSLClient`, `Crypto`) are declared in `platformio.ini` and downloaded automatically. No manual library installation.
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

`Config.h` contains your personal credentials and is not committed to the repository. You need to create it from the provided example before your first build:

```bash
cp ESP32-Modem/Config.h.example ESP32-Modem/Config.h
```

Then edit `ESP32-Modem/Config.h` with your settings:

```c
// WiFi credentials (only needed if using USE_WIFI mode)
const char* ssid = "YourNetworkName";
const char* password = "YourPassword";

// MQTT broker
const char* mqttServer = "your.broker.address";
const int   mqttPort   = 8883; // TLS port — use 1883 if you comment out USE_TLS
const char* mqttUser         = "your_username";
const char* mqttPassword     = "your_password";
const char* mqttUplinkTopic  = "bulletgcss/telem/your_callsign";
const char* mqttDownlinkTopic = "bulletgcss/cmd/your_callsign";

// Command signing public key (Ed25519, 32 bytes)
// Generate in the UI Security panel, paste here, then re-flash.
const uint8_t commandPublicKey[32] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// GPRS credentials (only needed for cellular modem)
const char apn[]      = "your.apn";
const char gprsUser[] = "";
const char gprsPass[] = "";
```

> **Do not commit `Config.h` with real credentials.** Keep your personal copy local.

### TLS encryption

TLS is enabled by default via `#define USE_TLS` in `Config.h`. This encrypts the MQTT connection between the firmware and the broker (port 8883). To disable it — for example, if your broker does not support TLS — comment out that line and change the port to 1883:

```c
// #define USE_TLS   ← commented out = no encryption
const int mqttPort = 1883;
```

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
- `Waiting for flight controller...` — FC probe in progress (normal on cold boot).
- `MSP connected` — flight controller communication working.
- `Mode ranges fetched` — `MSP_MODE_RANGES` startup step complete.
- `Override channels fetched` — `msp_override_channels` auto-configuration complete.

### Headless serial monitoring (no interactive terminal)

`pio device monitor` requires an interactive TTY and fails in non-TTY environments (e.g. scripts, CI, remote shells). Use Python instead:

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

To filter specific messages during verification:

```bash
python3 -c "..." | grep -E "Mode ranges|Override|MSP connected|cmd"
```

> **Note:** If the upload fails with "port is busy", another process (a previous monitor session or IDE) is holding the serial port. The `fuser ... | xargs -r kill` command above resolves this by killing those processes before flashing.

---

## Arduino IDE

Arduino IDE support was **discontinued**. The project originally shipped as an `.ino` sketch, but the main source file was renamed to `ESP32-Modem.cpp` when the build system was migrated to PlatformIO exclusively. Arduino IDE does not process `.cpp` files as sketches and will not open the project correctly.

Use PlatformIO as described above.

---

## Project structure

```
ESP32-Modem/
├── platformio.ini       # PlatformIO build config (environments, libraries)
├── Config.h             # User settings (credentials, modem type, intervals)
├── ESP32-Modem.cpp      # Main source file (FreeRTOS tasks, MQTT, network)
├── msp_library.h        # MSPv2 protocol interface
├── msp_library.cpp      # MSPv2 protocol implementation
└── uav_status.h         # UAV telemetry data struct
```
