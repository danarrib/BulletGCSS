# Bullet GCSS — Technical Backlog

This document tracks known issues, security concerns, and improvement opportunities.

---

## Completed Items

- **U1** — Fix main menu in landscape orientation ✅ — resolved by Bootstrap 5 migration (offcanvas sidebar scrolls naturally).
- **U2** — Migrate UI to Bootstrap 5 ✅ — completed in v1.7; `basicui.html` now runs on Bootstrap 5.3.8.
- **F1** — Multi-aircraft monitoring ✅ — completed in v1.6.
- **F2** — Waypoint mission planner ✅ — completed in v1.6.
- **C1** — Set Course (Cruise Mode heading) ✅ — uses `MSP2_INAV_SET_CRUISE_HEADING` (0x2223). Firmware and UI implemented.
- **C2** — Jump to Waypoint ✅ — uses `MSP2_INAV_SET_WP_INDEX` (0x2221). Firmware and UI implemented.
- **C3** — Set Altitude ✅ — uses `MSP2_INAV_SET_ALT_TARGET` (0x2215, INAV 10+). Firmware sends 5-byte payload (U8 datum + I32 altCm). UI gates the row behind INAV >= 10.0.0.
