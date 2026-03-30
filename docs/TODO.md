# Bullet GCSS — Technical Backlog

This document tracks known issues, security concerns, and improvement opportunities. Items are grouped by category and re-enumerated after each cleanup pass.

---

## New Commands

### C1. Set Course command (Cruise Mode heading)
Send a target heading to the aircraft while in Cruise Mode, so the operator can redirect the flight without using the RC transmitter.

**INAV MSP command:** `MSP_SET_HEAD` (code **211**) — payload: one `U16` value in centidegrees (0–36000). Already implemented in INAV; no firmware changes needed on the INAV side.

**What needs to be done:**
- **Firmware (`ESP32-Modem.cpp`):** add a new `cmd:setheading` handler that reads the `heading` field from the downlink message and sends `MSP_SET_HEAD` to the FC.
- **UI (`PageScripts.js` / commands panel):** add a heading input field (0–359°) and a Send button to the Commands panel.
- **Protocol (`BulletGCSS_protocol.md`):** document the new `cmd:setheading,heading:<centidegrees>` downlink message.

---

### C2. Jump to Waypoint (skip to WP N during active mission)

No dedicated MSP command exists in INAV for this. The controlling variable is `posControl.activeWaypointIndex` (`int8_t` inside the global `posControl` struct, defined in `navigation_private.h:494`). It is managed entirely by the navigation state machine and is not directly writable via MSP.

**Approach: contribute `MSP_SET_WP_INDEX` upstream to INAV**

A mid-flight waypoint manipulation workaround (overwriting the current WP slot with a temporary JUMP action, then restoring) was investigated and is technically feasible — but it goes against a core premise of Bullet GCSS: **INAV should be the one flying the aircraft**. Manipulating mission data mid-flight from the ESP32 is fragile and puts mission-management logic in the wrong place.

The right solution is a new `MSP_SET_WP_INDEX` command contributed to INAV that directly writes `posControl.activeWaypointIndex` and triggers the navigation state machine to activate the target waypoint cleanly. The INAV navigation code already has all the logic needed — it just needs an MSP entry point.

**INAV contribution scope:**
- Add a new MSP command (e.g. `MSP_SET_WP_INDEX`, code TBD) to `fc_msp.c` that accepts a `uint8_t` waypoint index, validates it against `posControl.waypointCount`, sets `posControl.activeWaypointIndex`, and fires the appropriate FSM event to activate the new waypoint.
- Open a PR to the INAV repository.

**Bullet GCSS scope (after INAV merges):**
- **Firmware (`ESP32-Modem.cpp`):** add `cmd:jumpwp` handler that sends `MSP_SET_WP_INDEX` to the FC.
- **UI:** add waypoint selector to the Commands panel.
- **Protocol (`BulletGCSS_protocol.md`):** document `cmd:jumpwp,wp:<n>`.

---

### C3. Set Altitude (target altitude for Altitude Hold / Cruise mode)

Allow the operator to command the aircraft to climb or descend to a specific altitude while Altitude Hold or Cruise mode is active.

INAV does not expose the target altitude via MSP. Internally, the target is stored in `posControl.desiredState.pos.z` (float, centimetres, local frame relative to home). It is set exclusively by `updateClimbRateToAltitudeController(desiredClimbRate, targetAltitude, mode)` in `navigation.c`. The relevant mode is `ROC_TO_ALT_TARGET` — this is the same call already used by waypoint navigation at `navigation.c:3496`:

```c
updateClimbRateToAltitudeController(0, pos->z, ROC_TO_ALT_TARGET);
```

This pattern works identically for both multicopters (throttle stick manages climb) and fixed-wing aircraft (pitch stick manages climb) — both share the same variable and function, just with different actuator outputs downstream.

**Approach: contribute `MSP_SET_ALTITUDE` upstream to INAV**

A new MSP command that writes `posControl.desiredState.pos.z` and calls `updateClimbRateToAltitudeController(0, targetAlt, ROC_TO_ALT_TARGET)` would let the ground station command a target altitude cleanly. INAV then handles the climb or descent exactly as it would for a waypoint target — no mid-flight RC channel manipulation needed.

**INAV contribution scope:**
- Add a new MSP command (e.g. `MSP_SET_ALTITUDE`) to `fc_msp.c` that accepts a signed 32-bit integer payload (target altitude in centimetres, relative to home), validates that Altitude Hold or a dependent mode (Cruise, Waypoint) is active, and calls `updateClimbRateToAltitudeController(0, targetAlt, ROC_TO_ALT_TARGET)`.
- Open a PR to the INAV repository.

**Bullet GCSS scope (after INAV merges):**
- **Firmware (`ESP32-Modem.cpp`):** add `cmd:setalt` handler that reads the `alt` field from the downlink message and sends the new MSP command to the FC.
- **UI:** add an altitude input field and Send button to the Commands panel (visible when Altitude Hold or Cruise mode is active).
- **Protocol (`BulletGCSS_protocol.md`):** document the new `cmd:setalt,alt:<centimetres>` downlink message.

---

## UI Improvements

### U1. Migrate UI to Bootstrap 5 ⏳ Deferred
Eventually migrate the UI to Bootstrap 5 for consistent, mobile-correct components. Priority: replace the sidebar menus with Bootstrap's **Offcanvas** component (directly fixes the touch handling issues). Bootstrap 5.3 dark mode (`data-bs-theme="dark"`) reduces the conflict with the existing dark theme. Integration approach: adopt Bootstrap JS + utility classes first, keep existing custom CSS for layout and theme, audit conflicts incrementally.

---

## Future Features

### F1. Multi-aircraft monitoring

Allow the UI to subscribe to additional MQTT telemetry topics and display those aircraft on the same map alongside the primary aircraft. Secondary aircraft are read-only — no commands, no EFIS, no detail panels — just situational awareness. The primary aircraft configuration and all existing functionality remain unchanged.

No firmware changes are required.

---

#### F1a — UI: "Monitor other UAVs" panel

A new **"Monitor other UAVs"** entry in the sidebar navigation menu opens a dedicated panel. The panel displays the list of currently monitored secondary topics and provides controls to manage them.

**Panel contents:**
- A scrollable list of monitored topics, each showing:
  - The MQTT topic string
  - The callsign received from that topic (or "—" if no message received yet)
  - A coloured dot matching the aircraft's map colour
  - A **Remove** button
- An **Add topic** input field + **Add** button at the bottom

**Topic validation on Add:**
- Must not be empty.
- Must not equal the primary uplink topic (`mqttUplinkTopic`) or the primary downlink topic (`mqttDownlinkTopic`).
- Must not already be in the secondary list.
- If any check fails, show an inline error message; do not add.

**Persistence:** The secondary topic list is saved to `localStorage` under the key `gcssMonitoredTopics` (JSON array of topic strings). It is restored and re-subscribed on page load / MQTT reconnect.

**Maximum:** No hard limit in the UI, but practically limited by broker and map readability. A warning (non-blocking) is shown if more than 5 topics are added.

---

#### F1b — MQTT subscription management

All secondary subscriptions are made on the same Paho MQTT connection as the primary aircraft — no second connection is needed.

**On Add:** call `mqttClient.subscribe(topic)`.
**On Remove:** call `mqttClient.unsubscribe(topic)`; delete the corresponding entry from `otherAircraft` and remove its marker, course line, flight path, and label from the map.
**On reconnect** (`onConnect` in `CommScripts.js`): re-subscribe to all topics in `gcssMonitoredTopics` from `localStorage`.

Incoming messages are dispatched by topic in the existing `onMessageArrived` handler:
- If the topic matches the primary uplink topic → existing parser path (unchanged).
- If the topic matches a key in `otherAircraft` → secondary parser (see F1c).

---

#### F1c — Simplified telemetry parser for secondary aircraft

Secondary aircraft state is stored in a module-level object in `CommScripts.js`:

```js
// keyed by MQTT topic string
const otherAircraft = {};
// per-aircraft entry shape:
// {
//   topic:     "bulletgcss/telem/GOLF1",
//   callsign:  "",
//   lat:       0,     // integer, degrees × 10,000,000  (÷1e7 → decimal degrees)
//   lon:       0,     // integer, degrees × 10,000,000
//   alt:       0,     // integer, centimetres            (÷100 → metres)
//   gsp:       0,     // integer, cm/s                   (÷27.78 → km/h)
//   vsp:       0,     // integer, cm/s                   (positive = climbing)
//   course:    0,     // integer, degrees true (0–359)   — field `ggc`
//   lastSeen:  0,     // Date.now() ms
//   colour:    "#rrggbb",  // assigned at subscribe time, see F1d
//   flightPath: [],   // array of [lon, lat] pairs in decimal degrees (for GeoJSON)
// }
```

All protocol values are integers — floating-point quantities are scaled integers, consistent with the BulletGCSS protocol. Fields parsed from the MQTT payload:

| Protocol key | Entry field | Scale / notes |
|---|---|---|
| `cs` | `callsign` | String; validated same as primary (`^[A-Za-z0-9_-]+$`, ≤ 16 chars) |
| `gla` | `lat` | Integer, degrees × 10,000,000 |
| `glo` | `lon` | Integer, degrees × 10,000,000 |
| `alt` | `alt` | Integer, centimetres |
| `gsp` | `gsp` | Integer, cm/s |
| `vsp` | `vsp` | Integer, cm/s (positive = climbing, negative = descending) |
| `ggc` | `course` | Integer, degrees true (0–359) — GPS ground course |

> Note: `gsc` in the protocol is the GPS **satellite count**, not the course. The GPS ground course is `ggc`.

All other keys in the payload are ignored. `lastSeen` is updated to `Date.now()` on every received message that contains a valid `gla`/`glo` pair. The `flightPath` array is appended with each new valid position fix (see F1d).

`otherAircraft` is exported from `CommScripts.js` for use by `MapScripts.js`.

---

#### F1d — Map rendering

**Colour assignment:** When a topic is added, a colour is assigned from a fixed palette of distinct hues (e.g. orange, cyan, magenta, yellow, lime, red). The palette cycles if more than 6 topics are added. The colour is stored in the `otherAircraft` entry and used for the marker, course line, flight path, and label.

**Aircraft marker:** Each secondary aircraft is represented by an HTML `<div>` element overlay on the MapLibre GL map (same technique as the primary aircraft icon), containing the existing aircraft SVG. A CSS `hue-rotate()` filter is applied to shift the icon colour to the assigned hue. The marker rotates to show the current `course`.

**Course line:** Rendered as a MapLibre GL GeoJSON source + line layer (one per secondary aircraft), matching the behaviour of the primary aircraft's course line. The line extends from the aircraft's current position in the direction of `course` (field `ggc`), with a length equal to **one minute of flight at current ground speed** — i.e. `gsp` (cm/s) × 60 s = distance in cm, converted to metres and projected as a geodesic offset. Updated on every message. A stationary aircraft (`gsp == 0`) shows no line.

**Flight path:** A MapLibre GL GeoJSON line layer (one per secondary aircraft) connecting all recorded positions in chronological order, styled in the aircraft's assigned colour (same style as the primary aircraft's flight path). Each new valid `gla`/`glo` pair received is appended to `entry.flightPath`. The path is cleared if the aircraft's topic is removed and re-added.

**Callsign / telemetry label:** A small text box rendered as an HTML overlay div, positioned slightly to the right of the aircraft marker. Contents (single compact line):

```
CALLSIGN  ALT m  SPD km/h  ↑  (or ↓ for descending, — for near-zero)
```

- Altitude: `alt ÷ 100` metres.
- Speed: `gsp ÷ 27.78` km/h (rounded to one decimal).
- Climb/dive indicator: `↑` if `vsp > 20 cm/s`, `↓` if `vsp < -20 cm/s`, `—` otherwise.
- Label styled to match the aircraft colour (coloured left border or tinted background) for quick visual association.

**Stale data dimming:** A `setInterval` running every 5 seconds checks `Date.now() - entry.lastSeen` for each secondary aircraft. If more than **10 seconds** have elapsed with no message, the aircraft marker, course line, flight path, and label are set to **50% opacity** to signal a stale condition. Opacity is restored to 100% as soon as a new message arrives. Secondary aircraft are **never removed from the map** by a timeout — they remain (dimmed) until the user explicitly removes them from the monitored topic list.

---

#### F1e — Tap interaction

Tapping or clicking a secondary aircraft marker opens a small popup overlaid on the map, anchored near the marker. The popup contains:

- **Last seen:** human-readable time since last message (e.g. "3 s ago", "2 min ago").
- **Location:** the aircraft's current position expressed as an **Open Location Code** (Plus Code), computed from `lat`/`lon` (÷1e7 for decimal degrees). The code is displayed as a tappable/clickable element — tapping copies the code to the clipboard (using the Clipboard API; show a brief "Copied!" confirmation). The Open Location Code library is already bundled in the repo at `UI/js/olc.min.js`.
- **Stop tracking** button: unsubscribes the MQTT topic, removes the marker, course line, flight path, and label from the map, removes the entry from `otherAircraft`, removes the topic from `gcssMonitoredTopics` in `localStorage`, and closes the popup.

Tapping outside the popup closes it without action.

### F2. Mission planner
Allow the operator to plan a waypoint mission directly in the UI and upload it to the aircraft over the command channel.

---

#### INAV waypoint upload protocol (researched from INAV source)

**MSP commands used:**
- `MSP_WP_GETINFO` (20) — returns `maxWaypoints` (default 15), `missionValid` flag, `waypointCount`
- `MSP_SET_WP` (209) — upload one waypoint at a time; 21-byte payload
- `MSP_WP` (118) — read one waypoint from FC (already implemented for telemetry display)

**MSP_SET_WP payload (21 bytes, all little-endian):**

| Field | Type | Units | Notes |
|---|---|---|---|
| `waypointNumber` | uint8_t | — | 1-based index (1..N for mission) |
| `action` | uint8_t | — | See action codes below |
| `lat` | int32_t | deg × 10⁷ | e.g. 484123456 = 48.4123456° |
| `lon` | int32_t | deg × 10⁷ | |
| `alt` | int32_t | cm | Relative to home (p3 bit 0 = 0) or AMSL (p3 bit 0 = 1) |
| `p1` | int16_t | varies | Speed cm/s (WAYPOINT); loiter seconds (POSHOLD_TIME); land flag (RTH); heading (SET_HEAD); target WP index (JUMP) |
| `p2` | int16_t | varies | Speed cm/s (POSHOLD_TIME); loop count (JUMP); unused otherwise |
| `p3` | int16_t | bitfield | Bit 0 = alt mode (0=relative to home, 1=AMSL); bits 1-4 = user flags |
| `flag` | uint8_t | — | 0x00 = normal; **0xA5 = last waypoint** (required to validate mission) |

**Action codes:**

| Code | Name | p1 | p2 |
|---|---|---|---|
| 1 | WAYPOINT | speed (cm/s, 0=FC default) | unused |
| 3 | POSHOLD_TIME | loiter time (s) | speed (cm/s) |
| 4 | RTH | 1=land, 0=hover | unused |
| 5 | SET_POI | unused | unused |
| 6 | JUMP | target WP index (0-based) | loop count (-1=infinite) |
| 7 | SET_HEAD | heading 0–359 (-1=cancel) | unused |
| 8 | LAND | unused | unused |

**INAV upload rules:**
1. WP index 1 arriving at the FC triggers `resetWaypointList()` — this clears any existing mission.
2. WPs must be sent in strict order (1, 2, 3…N). INAV rejects WP N if it hasn't received N-1 yet.
3. The last WP must have `flag = 0xA5`. INAV sets `missionValid = true` only then.
4. INAV rejects all uploads if `NAV_WP_MODE` is currently active on the FC.

**Altitude:** always use p3 bit 0 = 0 (relative to home). AMSL mode requires knowing the home point's absolute elevation and is not worth the complexity for this use case.

**Maximum waypoints:** 15 by default (`NAV_MAX_WAYPOINTS`). Read at runtime from `MSP_WP_GETINFO`.

---

#### Transport: UI → Firmware → FC

The UI sends the entire planned mission to the firmware in a single burst of signed MQTT messages — one per waypoint — using a new command type:

```
cmd:setwp,wpno:<N>,la:<lat_e7>,lo:<lon_e7>,al:<alt_cm>,ac:<action>,p1:<p1>,p2:<p2>,p3:<p3>,f:<flag>
```

The last waypoint in the mission has `f:165` (0xA5). All messages use the existing Ed25519 signing and sequence-number mechanism.

**The firmware does NOT forward individual waypoints to the FC as they arrive.** Instead it buffers the entire incoming mission in a staging array and validates it before touching the FC:
- All WP numbers must be contiguous starting at 1 (no gaps, no duplicates).
- Exactly one WP must have `f:165`, and it must be the one with the highest WP number.
- No WP must have `wpno > maxWaypoints`.

Only after the WP with `f:165` is received and the above checks pass does the firmware forward the mission to the FC via sequential `MSP_SET_WP` calls. If validation fails, the staging buffer is discarded and a failure ACK is sent. The firmware also checks `uavstatus.fmWp == 1` before forwarding and refuses to upload if WP Mission mode is active.

This approach means the FC's mission is never left in a partial state from a dropped message mid-transfer.

---

#### F2a — UI: Mission Planner screen

The Mission Planner is a **full-screen view** accessed from a new **"Mission Planner"** entry in the sidebar navigation menu. It completely replaces the normal flight monitoring view — no EFIS, no information panel, no telemetry overlay. The user switches back to normal monitoring via the same menu or a close/back button.

**New file: `MissionPlannerScripts.js`** — owns all planner state and logic.
**New section in `basicui.html`** — a full-screen `<div>` containing the planner map and UI elements. Hidden by default; shown via CSS when planner mode is active.

**Planner map:**
- A separate MapLibre GL JS map instance, initialised when the planner view is first opened (lazy init to avoid loading map tiles until needed).
- Same tile styles available as the main map (CARTO Dark Matter / OpenFreeMap Liberty), controlled by the existing settings.
- Clicking/tapping an empty area of the map appends a new waypoint at that location and immediately opens the WP parameter modal (see below).
- Clicking/tapping an existing waypoint marker opens the parameter modal for that waypoint.
- WP markers are numbered and connected by a route line in order.
- WP markers are **draggable** — dragging repositions the waypoint; the route line updates live.

**Waypoint parameter modal:**
- Appears overlaid on the map, anchored near the tapped waypoint (repositioned if it would overflow the screen edge).
- Fields:
  - **WP #** (read-only label)
  - **Action** (dropdown): Waypoint / Loiter (POSHOLD_TIME) / RTH / Land
  - **Altitude** (number input, metres, relative to home) — shown for all action types
  - **Speed** (number input, m/s, 0 = FC default) — shown for WAYPOINT and POSHOLD_TIME
  - **Loiter time** (number input, seconds) — shown only when action = POSHOLD_TIME
- **Delete WP** button — removes this waypoint; remaining WPs are renumbered to keep the sequence contiguous (e.g. deleting WP 3 of 5 renumbers old 4→3, old 5→4; markers and line update immediately).
- **Close** button (or tap outside) — dismisses without changes.
- Changes take effect immediately on close (no separate "apply" step).

**Mission stats bar:**
- Persistent strip at the top or bottom of the planner screen (always visible).
- Shows: **N waypoints** · **total distance Xkm** · (optionally estimated flight time if a default speed is configured).
- Distance is computed as the sum of great-circle distances between consecutive WP coordinates.
- Updates live as WPs are added, moved, or deleted.

**Action buttons (toolbar or bottom bar):**
- **Upload to aircraft** — triggers the upload flow (see F2c). Disabled if no WPs or command channel unavailable.
- **Save to file** — exports the mission as INAV JSON.
- **Load from file** — imports an INAV JSON mission, populating the planner (replaces current planned mission after confirmation if one already exists).
- **Clear mission** — removes all WPs (with confirmation prompt if any exist).

**Constraint:** The "add waypoint" action is disabled (cursor change + toast message) once `plannedMission.length >= data.maxWaypoints`.

**Planned mission storage:** `plannedMission[]` is a module-level array in `MissionPlannerScripts.js`, not part of the shared `data` object. It persists for the duration of the browser session (not saved to IndexedDB — the operator saves to file if they want to keep it).

---

#### F2b — UI: Mission file format (INAV JSON)

Import/export uses the INAV Configurator's JSON format for interoperability with INAV Configurator and other tools:

```json
{
  "version": 2,
  "items": [
    { "type": "Waypoint", "lat": 48.1234, "lon": 11.5678, "alt": 50, "p1": 0, "p2": 0, "p3": 0, "flag": 0 },
    { "type": "Waypoint", "lat": 48.1240, "lon": 11.5690, "alt": 60, "p1": 0, "p2": 0, "p3": 0, "flag": 165 }
  ]
}
```

- `alt` is in metres in the JSON file; convert to cm when sending via MQTT.
- `flag: 165` marks the last waypoint.
- Action type string ↔ code mapping: `"Waypoint"↔1`, `"PosHoldTime"↔3`, `"RTH"↔4`, `"SetPOI"↔5`, `"Jump"↔6`, `"SetHead"↔7`, `"Land"↔8`.
- On import the `flag` field of the last item is forced to 165 regardless of file content (defensive).

---

#### F2c — UI: Upload flow

1. Pre-flight checks:
   - Command channel must be available (`downlinkOk`).
   - `data.fmWp !== 1` — refuse to upload if WP Mission mode is currently active; show error.
   - `plannedMission.length <= data.maxWaypoints` — refuse if over limit.
   - Mission must have at least 1 waypoint.
2. Show upload progress UI: "Uploading mission… WP 1 / N".
3. Send WP 1 (`f:0`) → wait for ACK.
4. Send WP 2 (`f:0`) → wait for ACK. Repeat.
5. Send last WP (`f:165`) → wait for ACK.
6. On success: show "Mission uploaded successfully". Update progress UI to complete.
7. On any ACK failure or timeout: abort remaining sends; show "Upload failed at WP N — mission not sent to FC".

The UI does not need to wait for `wpc`/`wpv` telemetry to confirm the FC accepted the mission; the firmware's validation and ACK response is the authoritative result.

---

#### F2d — Firmware: staging buffer and `cmd:setwp` handler

**Staging buffer:** `msp_set_wp_t stagedMission[NAV_MAX_WAYPOINTS_FIRMWARE]` — a file-scope array (separate from `currentWPMission[]` which holds the FC's currently loaded mission). `NAV_MAX_WAYPOINTS_FIRMWARE` can be defined as 15 (matching INAV's default); `uavstatus.maxWaypoints` holds the runtime value from `MSP_WP_GETINFO`.

**`cmd:setwp` handler in `mqttCommandCallback()`:**
1. Parse `wpno`, `la`, `lo`, `al`, `ac`, `p1`, `p2`, `p3`, `f`.
2. Validate `wpno` is in range 1..`uavstatus.maxWaypoints`; coordinates are within plausible bounds.
3. Store the parsed WP into `stagedMission[wpno - 1]`.
4. Track the highest received `wpno` and whether `f:165` has been seen and on which index.
5. If `f:165` is not set: send ACK success (more WPs expected). No MSP traffic to FC yet.
6. If `f:165` is set (last WP received): run full validation:
   - All indices 1..wpno must be filled (no gaps in the staged buffer).
   - Exactly one WP has `f:165` and it is the highest-numbered one.
   - `uavstatus.fmWp == 0` (WP Mission mode must not be active).
   - If any check fails: clear the staging buffer, send failure ACK with a short reason code.
   - If all checks pass: iterate `stagedMission[0..wpno-1]` and call `msp.command(MSP_SET_WP, ...)` for each. Send success ACK.
7. On a new `wpno:1` arriving, always reset the staging buffer (start of a new upload attempt).

---

#### F2e — Firmware: `wpmax` telemetry field

- Read `maxWaypoints` from `MSP_WP_GETINFO` once at startup (already called in the round-robin for `waypointCount`; just also capture `maxWaypoints` into `uavstatus.maxWaypoints`).
- Publish as `wpmax:<N>` in the low-priority telemetry message (it never changes at runtime).
- `uav_status.h`: add `uint8_t maxWaypoints`.
- UI stores in `data.maxWaypoints`; gates the "add WP" action in the planner.

---

#### F2f — Documentation

**`docs/User-Interface.md`:**

Replace the current "Mission Planner" stub (under Sidebar Menu) with a full section covering:

1. **Opening the planner** — tap "Mission Planner" in the sidebar menu; the full-screen planner view replaces the normal monitoring UI. Tap the ✕ or "Back" button to return to monitoring.

2. **Placing waypoints** — tap or click any point on the map to place a waypoint there. A parameter modal appears anchored near the tap point. Fields:
   - **Action** — Waypoint (fly-through), Loiter (hold position for N seconds), RTH (return to home), Land
   - **Altitude** — metres above the home/takeoff point
   - **Speed** — m/s cruise speed to this waypoint (0 = use FC default)
   - **Loiter time** — seconds to hold (shown only for Loiter action)
   Confirm to add, or dismiss to cancel.

3. **Editing a waypoint** — tap an existing waypoint marker on the map to reopen its parameter modal. Changes take effect on close.

4. **Moving a waypoint** — drag a waypoint marker to a new location; the route line updates in real time.

5. **Deleting a waypoint** — tap the waypoint marker to open the modal, then press Delete. Waypoints after the deleted one are renumbered automatically to keep the sequence contiguous.

6. **Mission stats bar** — the bar at the top of the planner always shows the current waypoint count and total route distance (sum of straight-line distances between consecutive waypoints).

7. **Uploading to the aircraft** — press **Upload to aircraft**. The planner checks that the command channel is available and that WP Mission mode is not active on the FC before sending. Progress is shown as each waypoint is acknowledged by the firmware. If any waypoint fails, the upload is aborted and the FC's mission is left unchanged.

8. **Saving and loading mission files** — press **Save to file** to export the mission as an INAV-compatible JSON file that can be opened in INAV Configurator. Press **Load from file** to import a previously saved file (or one created in INAV Configurator); the current planned mission is replaced after a confirmation prompt.

9. **Clearing the mission** — press **Clear mission** to remove all waypoints (confirmation required).

10. **Waypoint limit** — the planner prevents adding more waypoints than the connected FC supports (reported by the firmware at connection time, default 15). A message is shown if the limit is reached.

**Screenshots to add to `scripts/take-screenshots.js`** (implement after the feature is built):

- `ui_missionplanner_overview.png` — full-screen planner with a 5–7 waypoint mission placed, route line visible, stats bar showing count and distance. Clip to the planner `<div>`.
- `ui_missionplanner_modal.png` — planner with the waypoint parameter modal open over a WP marker. Clip to show both the map context and the modal.
- `mp_workflow.png` — composition (3 frames): (1) sidebar menu with "Mission Planner" highlighted, (2) empty planner map, (3) planner with a complete mission and the upload button visible.

### F3. ESP32-Cam integration — aerial image capture
Support an ESP32-Cam module connected to the UAV, allowing the operator to trigger image captures and view them in the ground station.

**Scope:**
- The ESP32-Cam would operate as a companion device alongside the telemetry ESP32, or as a combined board if hardware allows.
- The operator triggers a capture from the UI; the image is transferred to the ground station and displayed in a dedicated panel or overlay.

**Open design questions:**
- **Transport:** MQTT is not well-suited for binary image data. Options include: a separate direct HTTP endpoint on the ESP32-Cam, encoding images as base64 over MQTT (inefficient), or a dedicated WebSocket stream.
- **Trigger mechanism:** the command channel could carry the capture command, or a separate command channel could be defined for the camera.
- **Image size vs data usage:** on cellular, full-resolution JPEG frames are expensive. The firmware should compress aggressively and allow the operator to configure resolution/quality.
- **Live video:** out of scope for the initial implementation — single frame capture on demand is the target.

---

## Completed

| Item | Description |
|------|-------------|
| Step 1 | **ES Modules refactor** — All UI JS files migrated from global scripts to ES Modules (`import`/`export`). Global state eliminated; `CommScripts.js` owns the central `data` object. Cache-busting added via import map. |
| Step 2 | **Flight Sessions** — Every MQTT message is persisted to IndexedDB in real time (two-store schema: `sessions` metadata + `session_messages` log lines). On page load the open session state is restored by fast-forwarding all stored messages through the parser. Sessions can be renamed, replayed, exported, imported, or deleted from the Sessions panel. |
| Step 3 | **Protocol version detection** — `pv` field added to the low-priority message (every 60 s). UI reads `pv` and stores it in `data.protocolVersion`. Missing `pv` (old firmware) is treated as version 1. |
| Step 4 | **Bidirectional channel setup + ping** — Separate uplink/downlink MQTT topics. Firmware subscribes and acknowledges commands with a `cid`. UI tracks pending commands and marks them received or lost. |
| Step 5 | **Key pair setup and distribution** — Ed25519 key pair generated via Web Crypto API. Private key in `localStorage`; public key pasted into `Config.h`. Security panel displays ready-to-paste C declaration. |
| Step 6 | **Encrypted ping** — All commands signed with Ed25519. Monotonically increasing sequence number for replay protection, persisted to NVS across reboots. |
| Step 7 | **Flight controller commands** — RTH, Altitude Hold, Cruise, Mission (WP), Angle, and Beeper toggled via `MSP_SET_RAW_RC`. Channel deactivation uses a gap-finding algorithm. UI commands panel with ON/OFF buttons, visual state indication, and a FAB for quick access. Single command channel status icon with 3 states (error / warning / ok). |
| Bug 1 (SIM) | **SIM cards with PIN lock** — `connectToGprsNetwork()` now calls `modem.simUnlock()` when `GSM_PIN` is set. On failure, logs a clear error and restarts. |
| Bug 2 | **User location compass heading** — `DeviceOrientationEvent` integrated in `MapScripts.js`. iOS uses `webkitCompassHeading`; Android/Chrome uses `deviceorientationabsolute`. iOS 13+ button in sidebar triggers permission; all other platforms start automatically. |
| U1 | **CSS touch fixes** — `touch-action: manipulation`, `user-select: none`, and `cursor: pointer` added to all interactive elements. `:hover` styles restricted to pointer devices via `@media (hover: hover)`. |
| U2 | **Command state telemetry + visual feedback** — Per-command active state fields (`cmdrth`, `cmdalt`, `cmdcrs`, `cmdwp`, `cmdbep`) added to telemetry uplink. Commands panel buttons driven by firmware-reported state. |
| U3 | **Alpine.js** — Included via CDN to replace manual DOM manipulation for panel/menu state and button active states. |
| Doc 1 | **INAV 9 minimum version** — All relevant documentation updated to state that Bullet GCSS requires INAV 9 or newer. |
| Doc 2 | **README security notice** — Prominent ⚠ Security Notice section added explaining public broker risk (GPS location visible to anyone), linking to self-hosting guide, and noting commands are Ed25519-protected. |
| F (map) | **MapLibre GL JS migration** — OpenLayers replaced with MapLibre GL JS vector map. Two user-selectable styles (OpenFreeMap Liberty default, CARTO Dark Matter). All elements scaled by `devicePixelRatio`. Compass and center-on-aircraft controls added. |
| C4 | **Position Hold command** — ON/OFF toggle added to the Commands panel. `MSP_PERM_ID_POSHOLD` added to `cmdModes[]`; `cmdph`/`fmph` telemetry fields added to firmware, UI, and protocol docs. |
