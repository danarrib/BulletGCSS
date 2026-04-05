# Bullet GCSS — UI Architecture

## Overview

The UI is a single-page application (SPA) built as a PWA. There is no build step — all files are plain HTML, CSS, and JavaScript served directly. The entry point is `basicui.html`, which loads the JavaScript as a single ES module entry point (`bsPageScripts.js`), which in turn imports all other modules.

`basicui.html` is built on **Bootstrap 5.3.8** (loaded from CDN) with a fixed-top navbar, a slide-in offcanvas sidebar, and Bootstrap modals for settings panels. `bsPageScripts.js` replaces the older `PageScripts.js` (which backed the now-retired `oldui.html`).

The application has three visual areas rendered simultaneously:

- **Data panel** (`#panel-info`) — tabular telemetry values (GPS, battery, navigation, flight times)
- **Map** (`#panel-map`) — MapLibre GL JS vector map with aircraft icon, flight path, home point, mission waypoints
- **EFIS (HUD)** (`#panel-efis` / `#hudview`) — canvas-drawn artificial horizon, heading tape, speed/altitude/vertical speed gauges

All three areas are driven by a central `data` object that is populated by parsed MQTT telemetry messages.

---

## Module Dependency Graph

```
basicui.html
└── bsPageScripts.js        ← ES module entry point (type="module")
    ├── CommScripts.js      ← Level 0: data, MQTT, shared utilities, session hooks
    ├── SessionScripts.js   ← Level 0: IndexedDB flight session persistence
    ├── EfisScripts.js      ← Level 1: imports CommScripts
    │   └── CommScripts.js
    ├── MapScripts.js       ← Level 2: imports CommScripts + EfisScripts
    │   ├── CommScripts.js
    │   └── EfisScripts.js
    ├── InfoPanelScripts.js ← Level 2: imports CommScripts + EfisScripts + MapScripts
    │   ├── CommScripts.js
    │   ├── EfisScripts.js
    │   └── MapScripts.js
    └── MissionPlannerScripts.js ← Level 2: imports CommScripts + MapScripts
        ├── CommScripts.js
        └── MapScripts.js
```

The graph is a strict directed acyclic graph (DAG). No module imports from a module at the same or higher level. `bsPageScripts.js` is the only module that imports everything — it is the application's wiring layer.

---

## Module Responsibilities

### `CommScripts.js` — Data & Communication Layer (Level 0)

The lowest-level module. Has no imports from other app modules.

**Responsibilities:**
- MQTT connection lifecycle via Paho MQTT over WebSocket
- MQTT message log: recording, saving to file, replaying from file or from IndexedDB session
- Telemetry parsing: deserialises incoming messages into the `data` object
- Dead reckoning: `estimatePosition()` and `estimateEfis()` fill in values between telemetry updates
- localStorage: read/write of MQTT broker settings
- Session hooks: fires callbacks set by PageScripts for message recording and replay-stop events

**Key exports:**

| Export | Type | Purpose |
|--------|------|---------|
| `data` | `let` object | Central telemetry state. All other modules read from this. |
| `mqtt` | `let` | Paho MQTT client instance |
| `mqttConnected` | `let` | Boolean connection flag |
| `lastMessageDate` | `let` | Timestamp of last received message |
| `isPlayingLogFile` | `let` | True while replaying a recorded log |
| `playbackPercent` | `let` | 0–100 progress of log replay |
| `pageSettings` | `let` | Refresh interval constants for all timers |
| `updatingWpAltitudes` | `let` | Flag: altitude fetch in progress (written via setter) |
| `setUpdatingWpAltitudes(val)` | function | Setter for `updatingWpAltitudes` (ES module live binding rule) |
| `MQTTconnect()` | function | Initialise and connect the MQTT client |
| `MQTTSetDefaultSettings()` | function | Reset broker settings in localStorage |
| `resetDataObject()` | function | Re-initialise `data` to default values |
| `parseTelemetryData(payload)` | function | Parse a telemetry string into `data` |
| `estimatePosition()` | function | Dead-reckoning position update |
| `estimateEfis()` | function | Dead-reckoning EFIS attitude update |
| `savemqttlog()` | function | Download the current session log as a text file |
| `replaymqttlog()` | function | Open a file picker and replay a previously saved log |
| `stopreplaymqttlog()` | function | Stop an active replay, reconnect MQTT, fire onReplayStop callback |
| `replayFromSessionMessages(lines)` | function | Start replay from an array of `"timestamp\|payload"` strings (from IndexedDB) |
| `restoreFromSessionMessages(lines)` | function | Fast-forward all lines through the parser to restore last known state |
| `setOnMessageCallback(fn)` | function | Register a callback fired for each live MQTT message line (used by PageScripts for session recording) |
| `setOnReplayStop(fn)` | function | Register a callback fired when replay ends (used by PageScripts to restore live session state) |
| `getDistanceBetweenTwoPoints()` | function | Haversine distance between two GPS coordinates (metres) |
| `DestinationCoordinates()` | function | Calculate a destination point given bearing and distance |
| `secondsToNiceTime(s)` | function | Format a number of seconds as `"1h 23m 45s"` |
| `inRange(val, min, max)` | function | Validate a value falls within a range |
| `rangeNumbers(...)` | function | Linear interpolation between two values |
| `rangeNumbers360(...)` | function | Linear interpolation wrapping around 0/360 |
| `publishCommand(cmdType)` | async function | Sign with Ed25519 and publish a command to the downlink MQTT topic. Returns the `cid` or `null` (if no private key or connection). Requires `commandPrivateKey` in `localStorage`. |
| `commandHistory` | array | Array of `{ cid, type, timestamp, status }` objects (newest first). Statuses: `'sent'`, `'received'`, `'lost'`. |

**Internal timers:**
- `timerReplay` (1000 ms): advances the log replay frame-by-frame when `isPlayingLogFile` is true.

---

### `SessionScripts.js` — Flight Session Persistence (Level 0)

The IndexedDB layer. Has no imports from other app modules.

**Responsibilities:**
- Opens/creates the `BulletGCSS` IndexedDB database with two object stores:
  - `sessions` — metadata: `{ id, name, status ('open'|'closed'), startTime, lastUpdate }`
  - `session_messages` — log lines: `{ id, sessionId, line }` where `line` is `"timestamp|payload"`
- Provides async CRUD operations consumed by PageScripts

**Key exports:**

| Export | Type | Purpose |
|--------|------|---------|
| `openDB()` | async function | Open/create the database; must be called once before any other function |
| `createSession(name)` | async function | Create a new open session; returns the new session id |
| `closeSession(id)` | async function | Mark a session as closed and record its end time |
| `appendMessage(sessionId, line)` | function | Fire-and-forget: append a log line to a session |
| `getOpenSession()` | async function | Return the current open session object, or null |
| `listSessions()` | async function | Return all sessions sorted by start time (newest first) |
| `getSessionMessages(sessionId)` | async function | Return all message objects for a session |
| `deleteSession(sessionId)` | async function | Delete a session and all its messages |
| `renameSession(sessionId, name)` | async function | Update a session's display name |

---

### `EfisScripts.js` — EFIS Rendering (Level 1)

**Imports:** `data` from CommScripts.

**Responsibilities:**
- Holds the `efis` configuration object (FOV, unit factors, canvas references)
- Renders all EFIS instruments to the `<canvas id="cvsEFIS">` element:
  - Artificial horizon (pitch + roll)
  - Heading tape
  - Speed tape
  - Altitude tape
  - Vertical speed bar
  - Compass rose

**Key exports:**

| Export | Type | Purpose |
|--------|------|---------|
| `efis` | `var` object | EFIS config + canvas state. Read by InfoPanelScripts for unit factors. |
| `renderEFIS(data)` | function | Full redraw of the EFIS canvas. Called by PageScripts timers. |
| `AngleToRadians(deg)` | function | Unit conversion utility used by MapScripts |
| `RadiansToAngle(rad)` | function | Unit conversion utility used by InfoPanelScripts |

**Note on private copies:** `CommScripts.js` also defines private (non-exported) `AngleToRadians` / `RadiansToAngle` functions for use by `getDistanceBetweenTwoPoints` and `DestinationCoordinates`. This avoids a dependency of CommScripts on EfisScripts.

---

### `MapScripts.js` — Map Rendering (Level 2)

**Imports:** `CommScripts` (data, utilities, state) + `EfisScripts` (AngleToRadians).

**Responsibilities:**
- Initialises the MapLibre GL JS map at startup (module top-level code runs on import)
- Renders and updates map features: aircraft icon, home icon, user location icon, flight path, course line, mission waypoints
- Tracks whether the user has manually panned the map (`user_moved_map`)
- Fetches waypoint terrain elevation from OpenTopoData API
- Supports runtime map style switching (CARTO Dark Matter / OpenFreeMap Liberty); style preference stored in `localStorage`
- All marker sizes, line widths, and fonts scaled by `window.devicePixelRatio` for HiDPI displays
- Custom map controls: compass (north-up reset) and center-on-aircraft button

**Key exports:**

| Export | Type | Purpose |
|--------|------|---------|
| `user_moved_map` | `let` | True if the user has manually panned the map |
| `setUserMovedMap(val)` | function | Setter for `user_moved_map` (written by PageScripts timers) |
| `setMapStyle(styleKey)` | function | Switch map style at runtime; `styleKey` is `'dark'` or `'liberty'`. Saves to `localStorage` and calls `map.setStyle()`. |
| `drawAircraftOnMap(data)` | function | Update aircraft icon position and heading |
| `drawAircraftPathOnMap(data)` | function | Draw the recorded flight path polyline |
| `drawCourseLineOnMap(data)` | function | Draw the line from aircraft to next waypoint |
| `drawMissionOnMap(data)` | function | Draw all mission waypoints |
| `drawHomeOnMap(data)` | function | Draw the home point |
| `drawUserOnMap(data)` | function | Draw the user's GPS location |
| `centerMap(data)` | function | Re-centre the map on the aircraft (if not panned) |
| `hasUserLocation()` | function | Returns true if the browser has provided a GPS fix |
| `getUserLocation()` | function | Start watching browser GPS position |
| `getMissionWaypointsAltitude()` | function | Fetch terrain elevation for all waypoints via OpenTopoData |

**Note on `DestinationCoordinates`:** Originally defined in MapScripts. Moved to CommScripts to break a circular dependency (CommScripts needed it for dead reckoning). MapScripts now imports it.

**Note on style switching:** Custom GeoJSON sources and layers (flight path, mission, course line) are re-added inside a `map.on('style.load')` handler. This fires on every style change (`map.setStyle()`), ensuring custom layers survive style swaps at runtime.

---

### `InfoPanelScripts.js` — Data Panel Rendering (Level 2)

**Imports:** `CommScripts` (data, status flags, utilities) + `EfisScripts` (efis config, angle utils) + `MapScripts` (user location flag).

**Responsibilities:**
- Renders the telemetry data table (battery, GPS, navigation, flight times, efficiency)
- Generates status icon image paths (connection, GPS, battery, signal)
- Manages the blink state for alarm indicators (`blinkSlowSwitch`, `blinkFastSwitch`)
- Applies user unit preferences to the display (speed, altitude, distance, current, efficiency)
- Exposes `openGoogleMaps()` on `window` so it can be called from dynamically-generated HTML `onclick` strings

**Key exports:**

| Export | Type | Purpose |
|--------|------|---------|
| `blinkSlowSwitch` | `let` | Toggled every 2 s; used for slow-blink alarm indicators |
| `blinkFastSwitch` | `let` | Toggled every 250 ms; used for fast-blink alarm indicators |
| `toggleBlinkSlow()` | function | Flip `blinkSlowSwitch`. Called by PageScripts timer. |
| `toggleBlinkFast()` | function | Flip `blinkFastSwitch`. Called by PageScripts timer. |
| `updateDataView(data)` | function | Full redraw of the telemetry data panel. |
| `setUIUnits()` | function | Read unit prefs from localStorage and apply to `efis` config. |
| `openGoogleMaps(lat, lon)` | function | Open Google Maps / Apple Maps to a coordinate. Also assigned to `window.openGoogleMaps` for dynamic HTML. |

---

### `bsPageScripts.js` — Application Entry Point (Top Level)

**Imports:** All modules above.

This is the only `<script type="module">` tag in `basicui.html`. It is the wiring layer — it does not contain rendering logic, only orchestration.

**Responsibilities:**
- NoSleep integration (prevent screen sleep on mobile)
- Bootstrap offcanvas/modal panel open/close logic (Sessions, Monitor, Settings, Commands, Security, Broker, UI Settings)
- Flight session lifecycle: initialise IndexedDB on load, restore open session state, record live messages, handle replay stop
- MQTT broker settings UI (save, reset)
- UI settings UI (unit preferences save)
- Security panel: Ed25519 key pair generation (`crypto.subtle`), key status display (5 states)
- Commands panel: `rcCommands[]` state table, command history rendering
- Monitor panel: secondary aircraft topic list management
- Event listener registration for all interactive elements
- All application timers

**Session lifecycle (on DOMContentLoaded):**
1. `openDB()` → `getOpenSession()`
2. If an open session exists: restore `data` state by fast-forwarding all stored messages through `parseTelemetryData()`
3. If no open session: create one with a timestamp-based default name
4. Register `setOnMessageCallback` → appends every live MQTT message to IndexedDB
5. Register `setOnReplayStop` → restores open session state when replay ends

**Timers started on `DOMContentLoaded`:**

| Timer | Interval | Work done |
|-------|----------|-----------|
| `timerEFIS` | 100 ms | `estimateEfis()` + `renderEFIS()` |
| `timerMapAndData` | 250 ms | `toggleBlinkFast()`, `estimatePosition()`, map draws, `updateDataView()` |
| `timerOneSecond` | 1000 ms | Increment `powerTime`/`flightTime`, record flight path waypoints |
| `timerLowPriorityTasks` | 10 000 ms | Draw mission/home/user on map, re-centre map, fetch waypoint elevations, check UI version |
| `timerBlinkSlow` | 2000 ms | `toggleBlinkSlow()` |

---

## Shared Mutable State and the Setter Pattern

ES modules export **live bindings**: a module can export a `let` variable, and importers always see the current value — but only the **defining module** may write to it. This means cross-module writes require an explicit setter function.

Two cases in this codebase require this pattern:

### `updatingWpAltitudes` (owned by CommScripts)

`MapScripts.getMissionWaypointsAltitude()` needs to set this flag to prevent duplicate fetch requests. Since `CommScripts` owns the variable, `MapScripts` calls the exported setter:

```
CommScripts  ──exports──▶  updatingWpAltitudes (read by MapScripts, PageScripts)
             ──exports──▶  setUpdatingWpAltitudes(val)  ◀── called by MapScripts
```

### `user_moved_map` (owned by MapScripts)

The map's `pointerdown` event handler (inside MapScripts) sets this to `true`. The low-priority timer in `PageScripts` needs to reset it to `false`. Since `MapScripts` owns the variable:

```
MapScripts   ──exports──▶  user_moved_map (read by PageScripts)
             ──exports──▶  setUserMovedMap(val)  ◀── called by PageScripts timer
```

### Session hooks (owned by CommScripts, wired by PageScripts)

`CommScripts` fires callbacks into PageScripts for session-related events, avoiding any upward dependency:

```
CommScripts  ──exports──▶  setOnMessageCallback(fn)  ◀── PageScripts: fn appends line to IndexedDB
             ──exports──▶  setOnReplayStop(fn)        ◀── PageScripts: fn restores open session state
```

---

## Functions Moved to Break Circular Dependencies

Several functions had to be relocated from their "natural" home to a lower-level module to avoid import cycles. These are the key cases:

### `DestinationCoordinates` — was MapScripts, now CommScripts

`CommScripts.estimatePosition()` uses this to project the aircraft's next position using heading and speed. Since CommScripts cannot import MapScripts, the function was moved into CommScripts. MapScripts now imports it back from CommScripts.

### `getDistanceBetweenTwoPoints` — was InfoPanelScripts, now CommScripts

Used by both `InfoPanelScripts` (to calculate user-to-aircraft distance) and `CommScripts` itself (dead reckoning). Centralised in CommScripts; both InfoPanelScripts and MapScripts import it.

### `secondsToNiceTime` — was InfoPanelScripts, now CommScripts

Used by `CommScripts` to format playback timestamps in the log replay timer and by `PageScripts` to format session durations in the Sessions panel. Since CommScripts cannot import InfoPanelScripts, the function was moved to CommScripts. InfoPanelScripts imports it back.

### `pageSettings` — was PageScripts, now CommScripts

Contains the refresh interval constants used by PageScripts timers. Originally in PageScripts but needed to be accessible without importing from the top-level module.

---

## Angle Conversion Utilities

`AngleToRadians` and `RadiansToAngle` are defined in `EfisScripts.js` and exported for use by `MapScripts` and `InfoPanelScripts`. However, `CommScripts` also needs them (for `getDistanceBetweenTwoPoints` and `DestinationCoordinates`) but cannot import from `EfisScripts` without creating a dependency cycle. To solve this, **CommScripts contains private local copies** of both functions (not exported, only used internally).

```
EfisScripts  ──exports──▶  AngleToRadians, RadiansToAngle
                               ▲ imported by MapScripts, InfoPanelScripts

CommScripts  ──private──▶  AngleToRadians, RadiansToAngle (local copies)
                               used internally by getDistanceBetweenTwoPoints, DestinationCoordinates
```

---

## Cache Busting

ES module imports (`import { } from './Foo.js'`) have no query string by default, so browsers can serve stale cached module files even when the application has been updated.

The deploy pipeline replaces the `(UNIQUEID)` placeholder in `basicui.html` with the GitHub Actions run ID on every deploy. To extend this cache-busting to all imported modules, `basicui.html` includes an **import map** that remaps each module URL to its versioned equivalent:

```html
<script type="importmap">
{
  "imports": {
    "./js/bsPageScripts.js":         "./js/bsPageScripts.js?uid=(UNIQUEID)",
    "./js/CommScripts.js":           "./js/CommScripts.js?uid=(UNIQUEID)",
    "./js/EfisScripts.js":           "./js/EfisScripts.js?uid=(UNIQUEID)",
    "./js/MapScripts.js":            "./js/MapScripts.js?uid=(UNIQUEID)",
    "./js/InfoPanelScripts.js":      "./js/InfoPanelScripts.js?uid=(UNIQUEID)",
    "./js/SessionScripts.js":        "./js/SessionScripts.js?uid=(UNIQUEID)",
    "./js/MissionPlannerScripts.js": "./js/MissionPlannerScripts.js?uid=(UNIQUEID)"
  }
}
</script>
```

After `sed` runs in the pipeline, all UIDs are replaced with the current run ID. The import map intercepts every `import` statement in every module and adds the versioned query string, ensuring browsers always load the freshest version.

---

## The `data` Object

The `data` object is the single source of truth for all telemetry. It is defined and owned by `CommScripts`, exported as a live binding, and read by every other module. It is reset to defaults by `resetDataObject()` on startup and on log replay start.

Key fields:

| Field | Unit | Description |
|-------|------|-------------|
| `rollAngle` | decimal degrees | Aircraft roll, −180 to 180 |
| `pitchAngle` | decimal degrees | Aircraft pitch, −90 to 90 |
| `heading` | decimal degrees | Magnetic heading, 0 to 360 |
| `altitude` | centimetres | Altitude above home |
| `altitudeSeaLevel` | metres | Altitude above sea level |
| `groundSpeed` | cm/s | GPS ground speed |
| `airSpeed` | cm/s | Pitot airspeed (if available) |
| `verticalSpeed` | cm/s | Climb/sink rate |
| `gpsLatitude` / `gpsLongitude` | decimal degrees | Aircraft GPS position |
| `gpsSatCount` / `gpsHDOP` | — | GPS fix quality |
| `gps3DFix` | 0/1 | Whether a 3D fix is available |
| `homeDirection` / `homeDistance` | degrees / metres | Vector to home point |
| `batteryVoltage` / `batteryCellCount` | V / count | Battery state |
| `currentDraw` / `capacityDraw` | A / mAh | Power consumption |
| `rssiPercent` | % | RC link signal strength |
| `flightMode` | string | Active flight mode name |
| `uavIsArmed` | 0/1 | Whether the aircraft is armed |
| `callsign` | string | Aircraft identifier |
| `currentMissionWaypoints` | array | Waypoint objects from the flight controller |
| `currentFlightWaypoints` | array | GPS track recorded since arm |
| `estimations` | object | Dead-reckoned values between telemetry frames |
| `downlinkStatus` | 0/1 | Whether the firmware is subscribed to the command topic (`dls` field) |
| `mspRcOverride` | 0/1 | Whether the `MSP RC OVERRIDE` flight mode is active on the FC (`mro` field). Must be 1 for RC channel commands to work. |
| `firmwarePublicKey` | string | Ed25519 public key broadcast by the firmware (`pk` field, base64, 44 chars). Empty string if not yet received. |
| `protocolVersion` | number | Protocol version reported by firmware (`pv` field). Defaults to 1 if missing. |

---

## HTML Structure

`basicui.html` is a Bootstrap 5 single-page app. Layout is CSS flexbox (portrait: column, landscape: row).

```
body
├── nav.navbar (fixed-top)          ← Gear button + status icon bar
├── #main (position:fixed)          ← Below navbar, fills remaining screen
│   ├── #left-col                   ← Left half in landscape (display:contents in portrait)
│   │   ├── #panel-info             ← Telemetry table
│   │   └── #panel-efis / #hudview  ← EFIS canvas (#cvsEFIS)
│   └── #panel-map / #map           ← MapLibre GL JS map
├── #sideMenu (offcanvas-start)     ← Main navigation sidebar
├── #bsOffcanvasSettings (offcanvas-start) ← Settings navigation
├── #bsOffcanvasSessions (offcanvas-start) ← Flight sessions panel
├── #bsOffcanvasMonitor (offcanvas-start)  ← Monitor other UAVs panel
├── #bsModalBroker (modal)          ← MQTT broker configuration
├── #bsModalUI (modal)              ← Unit preferences
├── #bsModalSecurity (modal)        ← Ed25519 key management
├── #bsModalCommands (modal)        ← RC commands + command history
├── #missionPlanner (position:fixed)← Full-screen mission planner overlay
└── #fabCommands                    ← Floating action button (CMD)
```

Portrait layout: `#panel-info` (order 1) → `#panel-map` (order 2) → `#panel-efis` (order 3), equal thirds.
Landscape layout: left column (`#left-col`, 50%) holds Info + EFIS stacked; right side is Map.

---

## Third-Party Libraries

| Library | Source | Purpose |
|---------|--------|---------|
| Bootstrap 5.3.8 | CDN (CSS + JS in `basicui.html`) | Layout, components (modals, offcanvases, navbar) |
| Bootstrap Icons 1.11.3 | CDN (CSS in `basicui.html`) | Icon font for sidebar nav and buttons |
| Ubuntu font | Google Fonts CDN | UI typography |
| MapLibre GL JS 4 | CDN (`<link>`/`<script>` in `basicui.html`) | WebGL vector map rendering |
| Paho MQTT JS | `UI/js/` (bundled) | MQTT over WebSocket |
| NoSleep.js | `UI/js/NoSleep.min.js` (bundled) | Prevent screen sleep on mobile |
| Open Location Code | `UI/js/olc.min.js` (bundled) | Convert GPS to Plus Codes |

All libraries except Bootstrap (which uses its own module pattern) are loaded as traditional `<script src>` tags before `bsPageScripts.js`, making their globals (`maplibregl`, `Paho`, `NoSleep`, `OpenLocationCode`) available to all modules via the global scope.
