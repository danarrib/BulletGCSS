# Bullet GCSS v1.8 Release Notes

## Firmware — Reliability & Stability

### Modem Watchdog and Auto-Recovery
The ESP32 firmware now recovers from a frozen cellular modem without rebooting the entire board. If the modem stops responding (for example, due to a half-open TCP connection after a cellular NAT timeout), the firmware detects the freeze and performs a targeted recovery: it kills the modem task, performs a hardware reset of the SIM7600/SIM800 module, and restarts only the networking task. Total recovery time is typically 15–30 seconds, compared to a full ESP32 reboot in previous versions.

- Freeze detection triggers a warning after 15 seconds of modem silence, and a full recovery after 30 seconds.
- For SIM7600 modems, AT+CTCPKA (TCP keepalive) is now enabled to detect dead connections earlier.
- The entire TinyGSM/MQTT networking stack was moved to a dedicated FreeRTOS task (`modemTask`) on Core 0, isolating it from the flight controller communication task (`fcTask`) on Core 1.

### Thread Safety Fix
Signal strength polling (AT+CSQ) was being called from the wrong FreeRTOS task, creating a data race between the two CPU cores. This has been fixed — the modem is now accessed exclusively from `modemTask`, with the signal strength value shared safely via a volatile variable.

### Publish Failure Counter
The MQTT publish failure counter now decays on each successful publish, preventing transient WiFi or cellular hiccups from accumulating toward an unnecessary restart threshold.

### Firmware Log Improvements
All firmware serial log lines now include a seconds-since-boot timestamp, making it much easier to correlate events when debugging.

### MSP Box ID Discovery
Switched from `MSP_BOXNAMES` to `MSP_BOXIDS` for flight mode discovery at startup. This is more reliable and avoids issues with long box name strings overflowing the receive buffer. The `MSP_BOXNAMES` buffer was also increased from 512 to 1024 bytes as a safety measure.

---

## Multi-Aircraft Monitoring

This release adds full support for monitoring multiple aircraft simultaneously from a single browser session.

- A secondary aircraft panel appears when a second aircraft is detected on the same MQTT broker.
- The panel shows key telemetry: GPS coordinates, altitude, speed, battery, flight mode, and flight time.
- Flight time increments locally in real time even between telemetry updates.
- A "Stop Tracking" button removes the secondary aircraft from the session.
- A confirmation dialog prevents accidental removal.

---

## UI — Security & Commands

### Private Key Export and Import
Operators can now export their Ed25519 signing key as a password-protected PEM file and import it on another device. The key is encrypted with AES-256-GCM using a PBKDF2-derived key (100,000 iterations). This makes it practical to command the same aircraft from multiple devices without regenerating the key pair and re-flashing the firmware.

### Command Sequence Synchronisation
The firmware now broadcasts its last accepted command sequence number (`lseq`) in regular telemetry messages and in all command acknowledgements. When a second UI connects (after importing the key), it automatically synchronises its local sequence counter from the firmware, preventing replay-protection rejections.

### Improved Command Modal Warnings
The "Send Command" panel now shows specific, actionable warnings for each blocking condition:

- **INAV too old** — Commands require INAV 9.0.0 or newer.
- **No signing key** — No key pair has been configured in the Security panel.
- **Firmware has no public key** — The public key hasn't been flashed into Config.h yet.
- **Key mismatch** — The UI and firmware are using different keys.
- **No command channel** — The downlink MQTT topic is not active.
- **MSP RC Override not active** — The flight controller is not in MSP RC Override mode.

### INAV Version-Gated Commands
The "Set Altitude", "Set Course", and "Jump to WP" commands require INAV 9.0.2 or newer (extended MSP command support). These rows are now hidden entirely when connected to an older firmware version, rather than shown as permanently disabled.

---

## UI — Mission Planner

### Home Point on Planner Map
If the aircraft has reported a home point, it is now shown on the Mission Planner map as well. This gives a visual reference when placing waypoints relative to the launch/home position.

### Mission Download Fix
The `getmission` command was dropping the last waypoint when downloading a mission from the aircraft. The off-by-one error in the firmware's buffer allocation and loop bounds has been fixed.

---

## UI — Status Icons

### Icon Info Modal
Tapping any status icon in the top bar (connection, signal, battery, GPS, command channel) now opens a small modal explaining what the icon means and what its current state indicates. This is especially useful for new users unfamiliar with the icon states.

---

## UI — General

### Tap Feedback on Buttons
Buttons and navigation menu items now show a visual highlight when tapped, providing clearer touch feedback on mobile devices.

### Mission Planner Layout
The mission planner stats bar and navigation row share a single line in landscape orientation, recovering vertical screen space on phones held sideways.
