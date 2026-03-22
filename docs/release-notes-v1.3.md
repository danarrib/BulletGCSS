# BulletGCSS v1.3 Release Notes

It has been a while since the last release, but BulletGCSS is very much alive — and this update is a big one. Version 1.3 brings security improvements, firmware reliability fixes, better tooling for developers, and a lot of new documentation. And there is plenty more coming.

---

## What's new

### 🔒 Security improvements

**Optional TLS encryption for the MQTT connection**
The firmware can now connect to your MQTT broker over an encrypted TLS connection. Enable it with `#define USE_TLS` in `Config.h` and point your broker to port 8883. This protects your telemetry stream from passive eavesdropping on the cellular or WiFi network. TLS is optional — plain connections still work exactly as before.

**Telemetry input validation**
The UI now validates every field received from the MQTT broker before displaying it. Out-of-range or malformed values are silently discarded rather than displayed. GPS coordinates are validated as a pair — a single bad coordinate no longer corrupts the map position.

**Elevation proxy hardened**
The `proxy.php` used for terrain elevation data now enforces a strict domain allowlist. Previously the allowlist was disabled entirely, making it an open proxy.

---

### 🛠 Firmware fixes

- **MQTT client ID collision** — Multiple aircraft connecting to the same broker could knock each other offline by sharing the hardcoded client ID `"ESP32Client"`. Each device now identifies itself with a unique ID based on its hardware MAC address (`ESP32_<MAC>`).
- **MQTT connection dropping after every message** — The MQTT keepalive loop (`client.loop()`) was never called, so the broker would close the connection after the keepalive timeout — typically 15 seconds. On brokers that require a PINGREQ/PINGRESP exchange (rather than counting any packet as activity), this caused the connection to drop on every message cycle. Fixed by calling `client.loop()` on every iteration of the main loop.
- **Timer overflow bug** — A subtle `uint32_t` overflow in the telemetry loop timing would cause the firmware to stall after approximately 49 days of continuous uptime. Fixed.
- **Waypoint array** — The waypoint mission buffer was one entry short (255 instead of 256), which could cause a buffer overrun on a full mission. Fixed.
- **MSP error handling** — An accidentally commented-out error return in `msp_library.cpp` was re-enabled, making communication failures properly reported instead of silently ignored.

---

### 🧰 Developer experience

- **PlatformIO support** — The firmware can now be built with PlatformIO in addition to Arduino IDE. Two ready-to-use environments are included: `esp32-sim7600` (T-PCIE board) and `esp32-sim800` (T-Call board). Dependencies are managed automatically — no manual library installation needed. See the new [Development Setup guide](docs/Development-setup.md).
- **GitHub Actions CI** — Every push to the firmware code now automatically compiles both board variants to catch build errors early.
- **Credentials no longer in the repository** — `Config.h` is now gitignored. A `Config.h.example` with placeholder values is provided as a starting point.
- **Third-party library cleanup** — Bootstrap, jQuery, and Popper were never actually used by the UI and have been removed from the repository. The remaining libraries (OpenLayers, Paho MQTT, NoSleep, Open Location Code) are now documented with pinned versions in `UI/package.json`, and a `download-libs.sh` script makes updating them straightforward.

---

### 📖 Documentation

A lot of effort went into documentation this release:

- **[Troubleshooting guide](docs/Troubleshooting.md)** — Step-by-step help for the most common problems: firmware not connecting, no data in the UI, how to verify the ESP32 is publishing using `mosquitto_sub`, and more.
- **[Development setup guide](docs/Development-setup.md)** — Full instructions for setting up PlatformIO or Arduino IDE on Windows, macOS, and Linux.
- **[Communication protocol reference](docs/BulletGCSS_protocol.md)** — Updated with field validation bounds and a note on the QoS design decision.
- **[Multi-aircraft monitoring](docs/Multi-aircraft-monitoring.md)** — Documents current behaviour and the planned multi-aircraft feature.
- All documentation has been moved into the repository itself (previously it lived on the GitHub Wiki), making it easier to keep in sync with the code.

---

## What's coming

v1.3 is laying the groundwork for some bigger features planned for upcoming releases:

- **Uplink / command capability** — Sending waypoint missions from the UI to the aircraft
- **Multi-aircraft monitoring** — Multiple aircraft on a single map with a quick telemetry selector
- **Binary telemetry protocol** — A compact binary format to replace the current text protocol, reducing data usage and enabling protocol versioning
- **ES Modules refactor** — Modernising the UI codebase to use native JavaScript modules

---

## Upgrading

**Firmware:** re-flash your ESP32 with the new firmware. If you want TLS, add `#define USE_TLS` to your `Config.h` and update your broker port to 8883.

**UI:** if you are self-hosting, re-deploy the `UI/` folder. If you use the hosted version at [bulletgcss.outros.net](https://bulletgcss.outros.net), it is already updated.

The new UI is fully backwards compatible with v1.0 firmware — you can update the UI without updating your firmware and everything will continue to work.
