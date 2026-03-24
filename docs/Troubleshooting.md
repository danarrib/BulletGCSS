# Troubleshooting

This guide covers the most common problems and how to diagnose them.

---

## Firmware (ESP32) issues

### How to read the Serial Monitor

The firmware prints diagnostic messages to the serial port at **115200 baud**. This is the first place to look when something isn't working.

- **PlatformIO:** click the plug icon in the VS Code toolbar, or run `pio device monitor`
- **Arduino IDE:** go to **Tools → Serial Monitor** and set baud rate to 115200

**Messages to look for:**

| Message | Meaning |
|---|---|
| `WiFi connected!` | WiFi connection established |
| `GPRS connected!` | Cellular data connection established |
| `Connected to the broker!` | MQTT connection to the broker succeeded |
| `MSP connected` | Flight controller communication working |
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
4. If the modem prints `SIM not ready`, the SIM card may be PIN-locked — set `simPIN` in `Config.h`.
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

## UI issues

### The UI shows no data

Work through these checks in order:

1. **Is the broker address correct?** Open the sidebar → **Broker settings** and confirm the host, port, and topic match what is configured in `Config.h`.
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
