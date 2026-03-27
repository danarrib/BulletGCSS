You'll need a computer to configure and flash the TTGO T-Call or the TTGO T-PCIE boards. Bullet GCSS uses **PlatformIO** as its build system — see [Development Setup](Development-setup.md) for instructions on installing PlatformIO and opening the project.

## Setting up the environment

Follow the [Development Setup](Development-setup.md) guide to install PlatformIO and open the project. Return here once PlatformIO is ready.

## Configuring the firmware

Download Bullet GCSS from the [official repository](https://github.com/danarrib/BulletGCSS), then copy the example configuration file:

```bash
cp ESP32-Modem/Config.h.example ESP32-Modem/Config.h
```

Open `Config.h` in your editor and fill in your settings. Every setting is described below.

#### Use WiFi or GPRS

```c
#define USE_WIFI
```

When uncommented, the firmware connects via WiFi instead of the cellular modem. This is useful for bench tests — you don't need a SIM card or a data plan. For actual flight, comment this line out:

```c
// #define USE_WIFI
```

WiFi credentials are set in the `ssid` and `password` variables. Note: the ESP32 only supports 2.4 GHz networks.

#### Modem type (GPRS mode only)

When using a cellular modem, the modem type is set automatically by the PlatformIO **build environment** — you do **not** need to change anything in `Config.h`:

- Build `esp32-sim800` → selects the SIM800 module (TTGO T-Call board)
- Build `esp32-sim7600` → selects the SIM7600 module (TTGO T-PCIE board)

See [Development Setup — Build environments](Development-setup.md#build-environments) for how to select the right environment.

#### TLS encryption

```c
#define USE_TLS
```

When uncommented (the default), the MQTT connection is encrypted with TLS on port 8883. To disable encryption — for example, if your broker does not support TLS — comment the line out and change the port to 1883:

```c
// #define USE_TLS
const int mqttPort = 1883;
```

TLS is strongly recommended when using public brokers.

#### MQTT settings

```c
const char* mqttServer        = "broker.emqx.io";
const int   mqttPort          = 8883; // 8883 = TLS, 1883 = plaintext
const char* mqttUser          = "your_username";
const char* mqttPassword      = "your_password";
const char* mqttUplinkTopic   = "bulletgcss/telem/your_callsign";
const char* mqttDownlinkTopic = "bulletgcss/cmd/your_callsign";
```

Both topics must share the same callsign suffix. Choose a callsign that uniquely identifies your aircraft. See [Find a MQTT Broker](Find-a-MQTT-Broker.md) for broker options.

#### GSM / GPRS credentials (cellular mode only)

```c
const char apn[]      = "your.apn.address";
const char gprsUser[] = "";
const char gprsPass[] = "";
```

These depend on your cellular carrier — check your carrier's documentation for the correct APN string.

If your SIM card has a PIN lock, set it here (leave the string empty if not):

```c
#define GSM_PIN "1234"
```

#### Command signing public key

```c
const uint8_t commandPublicKey[32] = {
    0x00, 0x00, ...
};
```

This is the Ed25519 public key used by the firmware to verify that commands sent from the UI are authentic. Without it, the firmware will reject all commands (including Ping).

**Setup steps:**
1. Open the Bullet GCSS UI and go to **Settings → Security…**
2. Click **Generate key pair** (requires Chrome 113+, Firefox 130+, or Safari 17+).
3. Copy the `const uint8_t commandPublicKey[32] = { ... };` declaration shown in the panel.
4. Paste it into `Config.h`, replacing the all-zeros placeholder.
5. Re-flash the firmware.

Once the firmware is running with the new key, the Security panel will show **✓ Keys match** confirming that signed commands will be accepted.

> **Note:** If you regenerate the key pair in the UI, you must re-flash the firmware with the new public key. The old key will no longer work.

#### Polling settings

These control how often telemetry is fetched from the flight controller and published to the broker. The defaults work well for most use cases:

```c
#define MESSAGE_SEND_INTERVAL         1000  // ms between MQTT publishes (1 s)
#define TELEMETRY_FETCH_DUTY_CYCLE     200  // ms to fetch telemetry from FC
#define LOW_PRIORITY_MESSAGE_INTERVAL   60  // seconds between low-priority messages
```

See [How much data will Bullet GCSS use?](How-much-will-Bullet-GCSS-use-from-my-GPRS-data-plan?.md) if you need to reduce data consumption.

## Flashing the ESP32

Once `Config.h` is saved, flash the firmware using PlatformIO. See [Development Setup — Flashing the ESP32](Development-setup.md#flashing-the-esp32) for full instructions.

In brief:

1. Connect the ESP32 board to your computer via USB.
2. In VS Code, click the **Upload** button (→ arrow) in the PlatformIO toolbar, after selecting the right environment (`esp32-sim800` or `esp32-sim7600`).
3. Or from the command line:
   ```bash
   pio run -e esp32-sim800 --target upload
   ```

## Verifying it works

After flashing, open the serial monitor at 115200 baud:

- **VS Code:** click the plug icon in the PlatformIO toolbar.
- **CLI:** `pio device monitor`

Look for these messages in order:

| Message | Meaning |
|---|---|
| `WiFi connected!` or `GPRS connected!` | Network connection established |
| `Connected to the broker!` | MQTT connection working |
| `Waiting for flight controller...` | Normal — ESP32 probes FC every 2 s until INAV is ready |
| `MSP connected` | Flight controller communication working |
| `Mode ranges fetched` | RC channel-to-mode mapping discovered from INAV |
| `Override channels fetched` | `msp_override_channels` auto-configured in INAV RAM |

> **About `msp_override_channels`:** The firmware automatically reads the current `msp_override_channels` setting from INAV at every boot, enables any RC channels needed for the discovered flight modes, and writes the updated bitmask back. This is applied in RAM only — it is not saved to INAV's EEPROM, so it is re-applied on every boot without any manual INAV Configurator steps. You do not need to configure `msp_override_channels` manually.

> **MSP RC Override flight mode:** For RC channel commands (RTH, altitude hold, cruise, etc.) to take effect on the flight controller, the `MSP RC OVERRIDE` flight mode must be assigned to an RC channel switch in INAV Configurator (Modes tab) and the switch must be in the active position. The firmware auto-configures the channel bitmask, but the mode itself must be assigned and enabled by the pilot. The Command Channel status icon in the UI will show **warning** (amber) when subscribed but the mode is off, and **OK** (green) when both conditions are met.

If you see errors, refer to the [Troubleshooting guide](Troubleshooting.md).
