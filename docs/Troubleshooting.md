# Troubleshooting

This guide covers the most common problems and how to diagnose them.

---

## Firmware (ESP32) issues

### How to read the Serial Monitor

The firmware prints diagnostic messages to the serial port at **115200 baud**. This is the first place to look when something isn't working.

- **PlatformIO:** click the plug icon in the VS Code toolbar, or run `pio device monitor`
- **Arduino IDE:** not supported — use PlatformIO.

**Messages to look for:**

| Message | Meaning |
|---|---|
| `WiFi connected!` | WiFi connection established |
| `GPRS connected!` | Cellular data connection established |
| `Connected to the broker!` | MQTT connection to the broker succeeded |
| `Waiting for flight controller...` | FC probe in progress — normal during cold boot; ESP32 waits for INAV to finish booting |
| `MSP connected` | Flight controller communication working; startup sequence will now begin |
| `Mode ranges fetched` | `MSP_MODE_RANGES` startup step complete; channel/PWM assignments discovered |
| `Override channels fetched` | `msp_override_channels` auto-configuration complete; `cmdAvailable*` flags set |
| `MSP port recovery triggered` | Lost MSP contact for >1 s; all startup flags reset; re-probing FC |
| `Loaded lastSeq from NVS: N` | Replay-protection sequence number loaded from flash (shown on every boot) |
| `Command received: ...` | A command message arrived on the downlink topic |
| `Command verified and accepted. Sending ack: ...` | Signature valid — command executed and acknowledged |
| `Command rejected: no public key configured in Config.h` | `commandPublicKey` is all zeros — paste the key from the Security panel and re-flash |
| `Command rejected: missing or malformed security fields` | Command is missing `seq` or `sig`, or `sig` is not 88 characters — likely an old UI version |
| `Command rejected: seq N <= lastSeq M` | Replayed or out-of-order command — normal if an old signed message is re-sent |
| `Command rejected: signature base64 decode failed` | `sig` field contains invalid base64 characters |
| `Command rejected: invalid signature` | Signature did not verify — key mismatch between UI and firmware |
| `Failed to connect to broker, rc=...` | MQTT connection failed — check broker address, port, and credentials |
| `Modem not responding` | SIM card or modem hardware issue |

---

### ESP32 won't connect to WiFi

1. Check `ssid` and `password` in `Config.h` — they are case-sensitive.
2. Make sure `#define USE_WIFI` is uncommented in `Config.h`.
3. Confirm the WiFi network is 2.4 GHz — the ESP32 does not support 5 GHz networks.

---

### ESP32 won't connect to the cellular network

1. Check the serial monitor for modem initialization messages.
2. Verify the SIM card is correctly seated in the modem board.
3. Check that `apn`, `gprsUser`, and `gprsPass` in `Config.h` match your carrier's settings.
4. If the modem prints `SIM not ready`, the SIM card may be PIN-locked — set `GSM_PIN` in `Config.h`.
5. Some modem boards require a power boost circuit to initialize. The SIM800-based T-Call board has a `setPowerBoostKeepOn()` call for this — check the serial output for power-related errors.

---

### ESP32 won't connect to the MQTT broker

1. Confirm `mqttServer` and `mqttPort` in `Config.h` are correct.
2. If using TLS (`#define USE_TLS`), the port must be the broker's TLS port (typically 8883).
3. If the broker requires authentication, check `mqttUser` and `mqttPassword`.
4. The broker address must be reachable from the network the ESP32 is on. Test from a phone on the same network if possible.
5. Some public brokers have rate limits or require registration — check the broker's documentation.

---

### ESP32 connects to the broker but sends no data

1. Check the serial monitor — the firmware should print confirmation each time it fetches telemetry from the flight controller.
2. Verify the flight controller is powered on and connected to the ESP32 via UART (Serial2: RX=GPIO19, TX=GPIO18).
3. Check the MSP baud rate — the flight controller must be configured for 115200 baud on the UART port connected to the ESP32.
4. In INAV, confirm that the serial port connected to the ESP32 has **MSP** enabled (not GPS, telemetry, or another protocol).

---

### Verifying the ESP32 is publishing with `mosquitto_sub`

If you have `mosquitto-clients` installed, you can subscribe to the same topic from a computer and watch messages arrive in real time:

```bash
# Plain (no TLS)
mosquitto_sub -h broker.emqx.io -p 1883 -t "bulletgcss/telem/your_callsign" -v

# With TLS (skip certificate verification)
mosquitto_sub -h broker.emqx.io -p 8883 --insecure -t "bulletgcss/telem/your_callsign" -v
```

Replace `broker.emqx.io` and `your_callsign` with your actual broker and topic. If messages appear here but not in the UI, the problem is in the UI configuration.

---

---

### Commands are sent but never acknowledged

1. Check the serial monitor — look for `Command received:` followed by a rejection reason.
2. **`Command rejected: no public key configured`** — you have not pasted the public key into `Config.h` and re-flashed. Go to the UI Security panel, copy the declaration, paste it into `Config.h`, and reflash.
3. **`Command rejected: invalid signature`** — the UI's key pair does not match the key in the firmware. Either the key was regenerated in the UI without re-flashing, or the wrong key was pasted into `Config.h`. Regenerate the key pair in the Security panel and re-flash.
4. **`Command rejected: missing or malformed security fields`** — the command is missing the `seq` or `sig` fields. This can happen if an old version of the UI is being used.
5. Check the **Downlink Status** icon in the UI — if it is grey, the firmware has not confirmed it is subscribed to the command topic yet. Wait for the next low-priority telemetry message (sent every 60 s).
6. Check the **Security panel** status line — it must show **✓ Keys match** before commands can be accepted.

---

### Command Channel icon shows warning (not green)

The **Command Channel** status icon in the status bar has three states: error (not subscribed), warning (subscribed but MSP RC Override not active), and OK (subscribed and MSP RC Override active).

If the icon shows **warning** (attention state), the firmware is subscribed to the command topic (Ping will work) but the `MSP RC OVERRIDE` flight mode is not active on the flight controller. RC channel commands (RTH, altitude hold, etc.) will be accepted by the firmware but ignored by INAV.

To resolve:

1. Open INAV Configurator → **Modes** tab. Confirm that `MSP RC OVERRIDE` is assigned to an RC channel switch.
2. Flip the switch to the active position on your radio. The `mro` field in telemetry will change to `1` and the icon will turn green.
3. In the serial monitor, look for `Override channels fetched` — if this message does not appear, the startup sequence has not completed. Wait a few seconds after `MSP connected` appears.
4. Look for `Mode ranges fetched` — if the flight modes you care about show `not found`, they have no RC switch assigned in INAV. Assign them in the Modes tab and reconnect.

If the icon shows **error** (red/broken), the UI is not connected to MQTT or the firmware has not yet confirmed it is subscribed to the command topic. Check the main connection icon first.

---

### ESP32 displays "Waiting for flight controller..." repeatedly

The firmware probes the FC every 2 seconds on boot. If it never connects:

1. Confirm the flight controller is powered on.
2. Check the UART wiring between the ESP32 and FC (Serial2: RX=GPIO19, TX=GPIO18 on the ESP32). Swap TX/RX if in doubt.
3. In INAV Configurator → **Ports** tab, confirm the serial port connected to the ESP32 has **MSP** enabled (not GPS, not telemetry, not another protocol).
4. Confirm the FC UART is configured at **115200 baud**.

---

## UI issues

### The UI shows no data

Work through these checks in order:

1. **Is the broker address correct?** Open the sidebar → **Broker settings** and confirm the host, port, **Telemetry Topic**, and **Command Topic** match what is configured in `Config.h`.
2. **Is TLS enabled consistently?** If the ESP32 uses port 8883 (TLS), the UI must also use a TLS WebSocket port (typically 8084). If one side uses TLS and the other does not, messages will not flow.
3. **Is the topic exactly right?** Topic names are case-sensitive. `bulletgcss/telem/MyPlane` and `bulletgcss/telem/myplane` are different topics.
4. **Is the UI connected to the broker?** Watch the connection icon in the status bar — if it shows disconnected, the UI cannot reach the broker. Check the browser console for WebSocket errors.
5. **Is the ESP32 publishing?** Use `mosquitto_sub` (see above) to confirm messages are arriving at the broker independently of the UI.
6. **Is the page running over HTTPS?** Browsers block WebSocket connections from HTTPS pages to non-TLS brokers. If you see the protocol warning on load, that is the issue — use a TLS-enabled broker port.

---

### The UI shows stale data

Cellular coverage is intermittent by design — the aircraft may fly beyond antenna range temporarily. The connection icon in the status bar will indicate when no messages are being received. Data will resume automatically when the link recovers.

If data is permanently stale:
- Check that the ESP32 is still powered and connected to the broker (use `mosquitto_sub` to verify).
- The UI force-refreshes all fields every 10 seconds — if connectivity has been lost for longer than that, some fields may not update immediately on reconnect.

---

### UI settings are lost after refreshing

UI settings (broker address, topic, units) are stored in the browser's `localStorage`. Clearing browser data or using a private/incognito window will reset them to defaults.

---

## Still stuck?

- Check the [GitHub issues](https://github.com/danarrib/BulletGCSS/issues) to see if others have reported the same problem.
- Open the browser developer console (F12) and look for error messages in the **Console** and **Network** tabs.
- Enable verbose logging in the Serial Monitor and look for any repeated error patterns.
