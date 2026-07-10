# Future Plan: Bullet GCSS over LoRa

This document captures the design research and open questions for a potential LoRa-based variant of Bullet GCSS. No implementation work has been started. The notes are preserved here for reference if this is ever revisited.

---

## Motivation

The cellular version of Bullet GCSS requires a SIM card with a data plan and depends on cellular network coverage. A LoRa-based variant would allow operators to use the system in areas without cellular coverage, and without a recurring subscription cost.

---

## Proposed Architecture

### Aircraft side

Replace the TTGO T-PCIE + SIM7600 with an **ESP32-LoRa32** board (an ESP32 with an integrated LoRa radio, such as the TTGO LoRa32 or Heltec WiFi LoRa 32). The firmware reads MSP telemetry from the flight controller over UART exactly as today, but instead of publishing to an MQTT broker over cellular, it serialises the telemetry into a compact binary frame and transmits it over LoRa.

### Ground side

A second **ESP32-LoRa32** board on the ground receives the LoRa packets, decodes them, and exposes the telemetry to the operator's phone. The phone connects to the ground ESP32 over WiFi and the Bullet GCSS UI runs in the browser as normal.

### Data flow

```
Flight Controller (INAV)
  → MSP v2 (UART)
  → ESP32-LoRa32 (aircraft)
  → LoRa radio link
  → ESP32-LoRa32 (ground)
  → WiFi (local)
  → Phone browser running Bullet GCSS UI
```

Commands travel the same path in reverse. Ed25519 signing and replay protection carry over unchanged from the cellular version.

---

## Why a Binary Protocol is Required

The current Bullet GCSS protocol uses plain ASCII `key:value` pairs. A typical full telemetry message is 150–200 bytes. This is fine for MQTT over cellular but is too large for long-range LoRa operation.

In binary encoding, the same fields would fit in approximately **30–40 bytes**:

| Field group | ASCII size | Binary size |
|---|---|---|
| GPS lat/lon (two I32) | ~30 bytes | 8 bytes |
| Altitude, speed (four I16/I32) | ~36 bytes | 8 bytes |
| Attitude roll/pitch/heading (three I16) | ~24 bytes | 6 bytes |
| Battery voltage, current, fuel, drawn (four I16) | ~36 bytes | 8 bytes |
| Status flags (arm, dls, mro, hwh, fs, css, rsi — bitmask + two bytes) | ~24 bytes | 3 bytes |
| Header + checksum | — | ~4 bytes |
| **Total** | **~150–200 bytes** | **~37–40 bytes** |

A binary frame would need a small fixed header (frame type, sequence number, length) and a CRC checksum. The field layout would be fixed and versioned — not self-describing like the current ASCII format — which reduces flexibility but is necessary for bandwidth reasons.

The existing ASCII protocol can be kept for cellular mode. The binary protocol would be a LoRa-specific addition.

---

## LoRa Link Budget and Range

### Key LoRa parameters

LoRa range and data rate are controlled by three settings:

- **Spreading Factor (SF):** SF7 (fastest, shortest range) to SF12 (slowest, longest range). Each step up roughly doubles range and halves data rate.
- **Bandwidth (BW):** 125 kHz, 250 kHz, or 500 kHz. Wider = faster but shorter range.
- **Coding Rate (CR):** 4/5 to 4/8. Affects error correction overhead.

### Estimated performance

| Setting | Data rate | 40-byte airtime | Est. open-field range |
|---|---|---|---|
| SF12, BW125, CR4/5 | ~250 bps | ~1.3 s | 15–40 km |
| SF11, BW125, CR4/5 | ~440 bps | ~0.8 s | 10–20 km |
| SF10, BW125, CR4/5 | ~980 bps | ~0.3 s | 6–12 km |
| SF7, BW500, CR4/5 | ~21 kbps | ~15 ms | 2–5 km |

For a 10 km open-field target, **SF10 or SF11** is the right operating point. SF12 provides extra margin for non-ideal conditions (light vegetation, moderate elevation changes).

### Telemetry update rate

At SF10 with a 40-byte binary frame (~0.3s airtime) and a 5-second cycle:

```
t = 0.0s   Aircraft transmits telemetry (~0.3s)
t = 0.5s   Ground station may transmit a command if queued (~0.2s)
t = 0.7s   Channel idle
t = 5.0s   Next cycle
```

One telemetry update every 5 seconds is acceptable for most monitoring use cases. It is coarser than the 1-second cellular update but sufficient for situational awareness at range.

Command round-trip time (send → ACK) is at most one full cycle = 5 seconds.

### EU duty cycle restriction

In Europe, most LoRa sub-bands on 868 MHz are limited to **1% duty cycle**. A 1.3-second airtime packet (SF12) requires a minimum 130-second wait before retransmitting on the same sub-band. This effectively prevents 5-second update rates at SF12 in Europe.

Options:
- Use the 869.525 MHz sub-band (10% duty cycle, EU) — allows SF12 at ~13-second intervals
- Use SF10 or SF11 (shorter airtime) — stays within 1% duty cycle at 5-second intervals with some margin
- In North America (915 MHz, no duty cycle limit) this is not a constraint

**EU operators should target SF10 or SF11 for 5-second update rates.**

---

## Half-Duplex Scheduling

LoRa is half-duplex — the aircraft and ground station cannot transmit simultaneously. A simple time-division scheme is needed:

- The aircraft transmits telemetry on a fixed schedule (every N seconds).
- The ground station listens during aircraft transmit windows.
- The ground station transmits commands only in the idle gap after the aircraft transmission ends.
- If no command is queued, the ground station stays silent.

No acknowledgement is needed for telemetry (same as the current QoS 0 MQTT approach — a missed packet means a 5-second stale display at most). Commands do need acknowledgement; the ACK is carried in the next aircraft telemetry frame or as a short dedicated uplink packet.

---

## Ground Station WiFi Architecture

### The hotspot problem

The natural approach — have the ground ESP32 create a WiFi hotspot that the phone connects to — works but has a significant drawback: **the phone loses internet while connected to the ESP32 hotspot**, which also breaks terrain elevation data in the Mission Planner.

### Recommended approach: phone creates the hotspot

A better architecture reverses the direction:

1. The operator enables their phone's personal hotspot.
2. The ground ESP32 connects to the phone's hotspot as a WiFi client.
3. The phone retains full internet access (cellular data continues flowing).
4. The browser on the phone reaches the ESP32 at its hotspot IP (typically `172.20.10.x`).

```
Internet ──► Phone (cellular data + hotspot active)
                  └── WiFi client: ESP32-LoRa32 (ground)
                        ← browser connects to 172.20.10.x
```

This preserves internet access, terrain elevation, and all existing UI functionality.

The downside: the operator must manually enable their hotspot before flying, and the ESP32 must be configured with the phone's hotspot SSID and password (or use a fixed SSID/password set in firmware). This is a minor friction point but acceptable.

### WebSocket server on the ESP32

The ground ESP32 would run a WebSocket server using the **[AsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer)** + `AsyncWebSocket` library. This is well-supported on ESP32 and handles multiple simultaneous connections (up to ~4–5 clients on a hotspot).

The UI's transport layer would need to be adapted to speak WebSocket directly rather than MQTT-over-WebSocket. The MQTT framing adds overhead that only makes sense for a remote broker. All other UI logic (telemetry parsing, map, EFIS, mission planner, command panel) stays unchanged.

Alternatively, a minimal MQTT broker could run on the ESP32 (see [esp_mqtt](https://github.com/martin-ger/esp_mqtt)), which would allow the UI to connect without any changes. This is more complex to implement and maintain.

---

## TLS and Security

### Mixed-content problem

If the Bullet GCSS UI is loaded from an HTTPS origin (e.g. `https://bulletgcss.outros.net`), the browser will block unencrypted WebSocket connections (`ws://`) to the local ESP32 as mixed content. The UI must either be served from HTTP, or the WebSocket connection must use TLS.

### TLS on ESP32

TLS is technically supported on ESP32 via mbedTLS (bundled in ESP-IDF). `AsyncWebServer` has a TLS variant. However:

- A TLS handshake requires ~40–60 KB of heap. With WiFi, LoRa receive buffers, and WebSocket overhead, RAM is tight (ESP32 has ~200 KB free in a typical sketch).
- Self-signed certificates cause browser warnings unless the CA is installed on the phone — poor user experience.
- Let's Encrypt certificates for a local IP are not straightforward to automate.

### Recommendation

For a local private network (phone hotspot), TLS provides minimal security benefit — there is no third party who can intercept traffic on the phone's own hotspot. **TLS is not recommended for the local LoRa ground station connection.**

The practical solutions to the mixed-content problem:

1. **Serve the UI from the ESP32 itself** over plain HTTP. The ESP32 can serve the static HTML/JS/CSS files from SPIFFS/LittleFS. The browser loads the UI from `http://172.20.10.x` and connects to the WebSocket on the same origin — no mixed content. Terrain elevation still works because the browser itself has internet via the phone's cellular connection.

2. **Use a special DNS entry** pointing a subdomain (e.g. `local.bulletgcss.outros.net`) to the ESP32's hotspot IP, with a certificate baked into firmware. More complex but avoids serving the UI from the ESP32.

Option 1 is significantly simpler and is the recommended approach.

### Command security

Ed25519 signing and replay protection apply unchanged. The LoRa channel is broadcast by nature (anyone with a LoRa receiver on the same frequency can hear telemetry), but command forgery is still prevented by the signature scheme. Telemetry privacy on LoRa is no worse than on the public MQTT broker — both are essentially public channels.

---

## BLE — Why It Was Considered and Rejected

Bluetooth Low Energy was evaluated as an alternative to WiFi for the ground station connection (it would allow the phone to stay on its normal WiFi/cellular network while communicating with the ESP32 over BLE).

**The dealbreaker: Web Bluetooth is not supported on iPhone.**

Apple has not implemented Web Bluetooth in WebKit, and all browsers on iOS are required to use WebKit. This means no browser on iPhone — including Chrome — can access BLE. Since a significant portion of UAV operators use iPhones, BLE is not a viable transport for a PWA.

Web Bluetooth support matrix for reference:

| Platform | Browser | Web Bluetooth |
|---|---|---|
| Android | Chrome | ✅ Supported |
| Android | Firefox / Samsung | ❌ Not supported |
| iPhone / iPad | Any browser | ❌ Not supported (WebKit restriction) |
| macOS | Chrome | ✅ Supported |
| Windows | Chrome | ✅ Supported |

---

## Open Questions

1. **Binary protocol design** — What is the exact field layout, frame header format, and versioning scheme? Which fields can be dropped or reduced in precision for LoRa mode? Should the binary frame be a strict fixed layout or length-prefixed variable fields?

2. **Frequency and regional variants** — The LoRa frequency band varies by region (868 MHz EU, 915 MHz US, 433 MHz some regions). The hardware module must match the operating region. Should the firmware support runtime frequency configuration?

3. **LoRa library** — The most common Arduino LoRa library is [arduino-LoRa](https://github.com/sandeepmistry/arduino-LoRa). RadioLib is a more capable alternative with better support for advanced features. Which to use?

4. **Frequency hopping** — For US operation (no duty cycle limit), frequency hopping across the 915 MHz band would improve robustness against interference and is standard practice (used by ExpressLRS, LoRaWAN). Worth implementing from the start or add later?

5. **Ground station IP discovery** — The phone needs to know the ESP32's IP on the hotspot network. Options: fixed IP (ESP32 requests a specific IP), mDNS (`bulletgcss.local`), or a QR code shown on the ESP32 display (the LoRa32 boards have an integrated OLED screen). mDNS is the most seamless.

6. **UI transport layer** — Should the LoRa ground station expose a raw WebSocket, a minimal MQTT broker, or something else? If raw WebSocket, how much of `CommScripts.js` needs to change?

7. **Simultaneous cellular + LoRa** — Could the same UI session receive from both a cellular aircraft and a LoRa aircraft? Probably out of scope for v1.

8. **ESP32-LoRa32 board selection** — Several boards exist (TTGO LoRa32 V2.1, Heltec WiFi LoRa 32 V3, LilyGo T3-S3). They differ in LoRa chip (SX1276 vs SX1262), display, battery connector, and pin layout. The SX1262-based boards (newer) are preferred for better sensitivity and lower current draw.

9. **Power consumption** — LoRa transmission draws ~120 mA at 20 dBm. At one 40-byte packet every 5 seconds, average current is very low. A 1S LiPo (same as the TTGO T-Call) would give many hours of operation. Worth profiling.

10. **Encryption of the LoRa payload** — LoRa telemetry is broadcast and readable by anyone. If telemetry privacy is desired, AES-128 encryption of the binary frame is feasible on ESP32. This is separate from command signing (Ed25519) which already provides command authentication.

---

## Meshtastic as an Alternative to Raw LoRa

[Meshtastic](https://meshtastic.org/) is an open-source protocol that runs on the same ESP32+LoRa hardware already considered here (Heltec WiFi LoRa 32, LilyGo T3-S3, etc.). Rather than raw LoRa frames, it adds a full mesh networking stack on top:

- **Mesh routing** — any node in range relays packets for other nodes, extending effective range beyond a single LoRa hop
- **AES-256 encryption** — per-channel, built in, solves the telemetry privacy open question entirely
- **Protobuf serialisation** — compact binary framing with versioning
- **Multiple host APIs** — Serial (UART), TCP (WiFi), BLE for talking to a Meshtastic node from external firmware
- **Native MQTT gateway** — a Meshtastic node with internet access can bridge the mesh to/from an MQTT broker

### The MQTT bridging advantage

Meshtastic's MQTT gateway feature is particularly relevant to Bullet GCSS. A node with WiFi or cellular can relay all mesh traffic to a standard MQTT broker. This means the existing Bullet GCSS UI and MQTT transport layer could be reused without modification:

```
FC → ESP32 (BulletGCSS firmware) → UART Serial API → Meshtastic node (aircraft)
  → LoRa mesh → internet-connected Meshtastic node (anywhere in range)
    → MQTT broker → Bullet GCSS UI (unchanged)
```

The operator's phone stays on cellular. No local WebSocket server, no mixed-content TLS problem, no phone hotspot required. If existing Meshtastic infrastructure is present in the area (community nodes, relay stations), the aircraft is reachable even beyond direct LoRa range.

### Practical integration model

A dedicated private two-node setup on a private Meshtastic channel (separate PSK, invisible to the public mesh):

- **Aircraft side:** The BulletGCSS ESP32 firmware talks to a Meshtastic node over UART using the Serial API, sending telemetry as custom payload packets (private portnum 512–1023). The BulletGCSS firmware does not need to run Meshtastic itself — it treats the node as a radio pipe.
- **Ground side:** A second Meshtastic node with WiFi bridges received packets to the MQTT broker. This node can be fixed at the launch site (mains powered) or battery-powered.

### Limitations

**Update rate** is the main constraint. Meshtastic enforces rate limiting to be a good mesh citizen. Telemetry every 5–10 seconds is achievable; 1-second updates are not realistic and would be antisocial on shared infrastructure. A private two-node setup relaxes the social constraint but the radio duty cycle limits still apply.

**Command latency** in a multi-hop mesh scenario could reach 10–30 seconds round-trip. Acceptable for situational awareness and non-urgent commands; not suitable for anything time-critical.

### Comparison with raw LoRa

| | Raw LoRa (custom firmware) | Meshtastic |
|---|---|---|
| Range | 10–40 km direct | Same + mesh relay extends further |
| Encryption | Manual (AES-128, extra work) | AES-256, built in |
| Update rate | ~5 s at SF10 | ~5–10 s (rate limited) |
| Command latency | ~5 s | 5–30 s (mesh dependent) |
| UI changes | Transport layer adaptation needed | None if MQTT bridging used |
| Infrastructure dependency | None | Optional (works standalone, better with nodes nearby) |
| Implementation complexity | Higher (custom binary protocol) | Lower (Serial API to existing firmware) |

Meshtastic is the better starting point for a monitoring-focused LoRa variant. The lower implementation complexity and MQTT compatibility are significant advantages. The update rate and command latency trade-offs are acceptable for most beyond-cellular-range monitoring use cases.

---

## Summary

A LoRa variant of Bullet GCSS is technically feasible and would offer compelling range (10–40 km open field) without cellular infrastructure. The main design decisions are:

- Binary protocol redesign (required for bandwidth) — or use Meshtastic's Serial API to avoid this entirely
- Phone-as-hotspot architecture (preserves internet on the phone) — or use Meshtastic MQTT bridging via a fixed ground node
- UI served from ESP32 over plain HTTP (avoids TLS complexity) — not needed if MQTT bridging is used
- SF10/SF11 for 10 km range at 5-second update rates (EU duty cycle compliant)

**Recommended starting point:** a Meshtastic-based implementation using a private two-node channel with MQTT bridging. This reuses the existing UI and MQTT architecture unchanged, avoids the binary protocol design work, and provides built-in encryption. Raw LoRa with custom firmware remains an option if sub-5-second update rates or lower latency commands are required.
