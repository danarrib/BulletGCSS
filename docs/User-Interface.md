The User Interface (UI) is the part of Bullet GCSS that shows information to the UAV pilot.

The Bullet GCSS UI is a Progressive Web App (PWA), with means that it can both run directly on the SmartPhone Web Browser (Apple Safari, Google Chrome), or can be installed as an App on the SmartPhone home screen, and be used as any other regular App. [This is how to do it](How-to-install-Bullet-GCSS-on-SmartPhone.md).

It also works fine on PC too (Windows, Mac, Linux).

It works fine both on portrait mode (vertical screen) and landscape mode (horizontal screen). Works fine on every screen resolution and pixel ratios, and also works fine with non-rectangular screens (like the newer iPhones screens that has the sensors array taking the top part of the screen).

![Deploy BulletGCSS UI on Outros.net](https://github.com/danarrib/BulletGCSS/workflows/Deploy%20BulletGCSS%20UI%20on%20FPV%20Sampa/badge.svg)

You can use [Bullet GCSS directly from Outros.net](https://bulletgcss.outros.net), or you can download it and [host on your own Web server](Host-the-user-interface.md).

> **Note for Mozilla Firefox users with self-hosted Mosquitto MQTT Broker**
> Mozilla Firefox sometimes has problems to validate the SSL certificate. To fix this:
> 1. Open Firefox and type `about:config` on it's address bar
> 2. Find the key `network.http.spdy.websockets` and set it to `false`
> 3. Restart Firefox

## User Interface components

Screen is always divided into 4 main parts: The Status lights, the Information Panel, the Map and the EFIS PFD (Electronic Flight Information System Primary Flight Display). It's important that you learn how every of this parts works to enjoy all the Bullet GCSS capabilities.

### Status lights
![Status lights](screenshots/ui_statuslights.png)

Status lights tells the overall health of the system and aircraft. Each icon has meaning:

#### Connection icon

Shows how is the connection between UI and Aircraft thru MQTT Broker. There are 4 possible states:

Broken connection | Aircraft connection problem | Latency is too high | Connection ok
-- | -- | -- | --
<img src="https://user-images.githubusercontent.com/17026744/104824245-d75f0280-582e-11eb-9e88-fd8ebb16bb2d.png" width="50"> | <img src="https://user-images.githubusercontent.com/17026744/104824349-ae8b3d00-582f-11eb-8195-a8e9a2083754.png" width="50"> | <img src="https://user-images.githubusercontent.com/17026744/104824391-07f36c00-5830-11eb-85d5-371c31c84905.png" width="50"> | <img src="https://user-images.githubusercontent.com/17026744/104824418-4d179e00-5830-11eb-8c15-1b9b219979d1.png" width="50">
Means that the UI is not connected to the MQTT Broker. Without this connection, UI cannot receive any messages from aicraft. You should fix it by making sure your SmartPhone is connected to the Internet and by reviewing the MQTT Broker settings. | Means that the UI is connected to the MQTT Broker, but it's not receiving any messages for more than 30 seconds. It usually means that the aircraft is not sending any messages (due lack of connectivity or some other problem). | Means that both UI and Aircraft are connected to the MQTT Broker, but messages are taking too long to arrive. The yellow icon will show up if the last message sent from aircraft is more than 10 seconds old. | Means that both UI and Aircraft are connected to the MQTT Broker and messages are arriving on time.

#### Command icon

Shows how is the connection between Aircraft and the UI thru MQTT Broker. There are 3 possible states:

Broken connection | Connected but not controlling | Control ok
-- | -- | --
<img src="../UI/img/command_error.png" width="50"> | <img src="../UI/img/command_warning.png" width="50"> | <img src="../UI/img/command_ok.png" width="50"> | <img src="https://user-images.githubusercontent.com/17026744/104824418-4d179e00-5830-11eb-8c15-1b9b219979d1.png" width="50">
Means that the Aircraft is not connected to the MQTT Broker. Without this connection, UI cannot send any commands to the aicraft. | Means that the aircraft is connected to the MQTT Broker, but UI can't send any commands because the encryption keys don't match, or because the RC Override mode is not active. | Means that both UI and Aircraft are connected to the MQTT Broker and commands can be sent just fine.

#### Signal bars

Show the signal strength of the aircraft onboard modem.

Bad | Good | Excellent
-- | -- | --
<img src="https://user-images.githubusercontent.com/17026744/104824492-d929c580-5830-11eb-9f49-25b1686bcc4b.png" width="100"> | <img src="https://user-images.githubusercontent.com/17026744/104824495-db8c1f80-5830-11eb-8181-5e1bff06f641.png" width="100"> | <img src="https://user-images.githubusercontent.com/17026744/104824496-dd55e300-5830-11eb-898a-cb1293ea6ef5.png" width="100">
2 red bars | 3 yellow bars | 4 green bars

#### Radio controller signal strength

Shows the radio control signal strength. There are 4 possible states:

Broken | Bad | Poor | Ok
-- | -- | -- | --
<img src="https://user-images.githubusercontent.com/17026744/104827470-b7ced680-583c-11eb-96cd-0e4af5d37743.png" width="100"> | <img src="https://user-images.githubusercontent.com/17026744/104827466-b7364000-583c-11eb-97d0-bf8458f96c94.png" width="100"> | <img src="https://user-images.githubusercontent.com/17026744/104827473-b8676d00-583c-11eb-8662-351b72e67035.png" width="100"> | <img src="https://user-images.githubusercontent.com/17026744/104827472-b8676d00-583c-11eb-9069-3f7dd6c88e58.png" width="100">
RSSI < 10%, very poor or no control at all | RSSI between 10% and 29%, bad radio control link, but still alive | RSSI between 30% and 49%, poor control link | RSSI above 50%, radio control link working fine

#### Aircraft battery

It's a 6-state indicator showing the remaining battery capacity. This is informed by Flight Controller.

0% - 4% | 5% - 20% | 21% - 40% | 41% - 60% | 61% - 80% | 81% - 100%
-- | -- | -- | -- | -- | --
<img src="https://user-images.githubusercontent.com/17026744/104827562-b7830b00-583d-11eb-8f45-86f467e69955.png" width="100"> | <img src="https://user-images.githubusercontent.com/17026744/104827563-b7830b00-583d-11eb-889e-50521787b242.png" width="100"> | <img src="https://user-images.githubusercontent.com/17026744/104827564-b7830b00-583d-11eb-91d8-0fc30ee63eda.png" width="100"> | <img src="https://user-images.githubusercontent.com/17026744/104827565-b81ba180-583d-11eb-896c-360ce24da83d.png" width="100"> | <img src="https://user-images.githubusercontent.com/17026744/104827566-b81ba180-583d-11eb-8a47-78ee1cf28df4.png" width="100"> | <img src="https://user-images.githubusercontent.com/17026744/104827560-b651de00-583d-11eb-8930-82e9f54c2380.png" width="100">

#### Aircraft GPS Health

Shows the health of the aircraft GPS sensor. A bad GPS reception means that aircraft will not do autonomous flight properly, so it's a very important information.

Broken | Bad | Poor | Good
-- | -- | -- | --
<img src="https://user-images.githubusercontent.com/17026744/104827632-9ff85200-583e-11eb-92cd-8f3cf97c6c5e.png" width="100"> | <img src="https://user-images.githubusercontent.com/17026744/104827631-9f5fbb80-583e-11eb-9e00-2936fdf7760c.png" width="100"> | <img src="https://user-images.githubusercontent.com/17026744/104827634-a090e880-583e-11eb-97d3-f9b02fc0ae7f.png" width="100"> | <img src="https://user-images.githubusercontent.com/17026744/104827633-a090e880-583e-11eb-806e-e187e01cca45.png" width="100">
HDOP = 0 or > 5, means that there's no position at all | Less than 7 satellites, but there's 3D fix. It's flyable, but it's dangerously close to the unflyable edge. | Between 8 and 10 satellites, it's ok to fly, but precision can be not as great. | 11 satellites or more, good GPS reception.

#### Aircraft Hardware Health

It's a two-state indicator that shows the overall aircraft health.

Bad | Good
-- | --
<img src="https://user-images.githubusercontent.com/17026744/104827702-74299c00-583f-11eb-9ffd-33fb4d256c91.png" width="100"> | <img src="https://user-images.githubusercontent.com/17026744/104827701-73910580-583f-11eb-9c03-c7ec916f7f7e.png" width="100">
Means that there's some hardware error on the aircraft. Maybe a sensor is missing or not powered, or some calibration that was not done properly. Aircraft probably will not arm on that condition. | Hardware health is good, all sensors are working fine.

#### Command Channel Status

A single icon showing the combined health of the downlink command channel and the MSP RC Override flight mode on the flight controller. There are three states:

Error | Warning | OK
-- | -- | --
Red/broken icon | Attention icon | Green icon
The UI is not connected to MQTT, or the firmware has not confirmed it is subscribed to the command topic. Commands cannot be received. | The firmware is subscribed to the command topic, but the `MSP RC OVERRIDE` flight mode is not active on the flight controller. Ping will work, but RC channel commands (RTH, altitude hold, etc.) will be ignored by INAV. | The firmware is subscribed to the command topic and `MSP RC OVERRIDE` is active. All commands — including flight mode toggles — will be accepted by the flight controller.

### Information Panel

![Information Panel](screenshots/ui_infopanel.png)

Information panel gives you mostly TEXT information about aircraft status. There are some visual indications though.

#### Callsign

Callsign is the name of the aircraft, as it's defined on INAV. Aircraft will send the Callsign only each minute, so it may take up to 1 minute to this information appear on the screen.

#### Mode

It's the active flight mode. It tries to mimic the same flight mode OSD element of INAV. 

The possible values are:
* **MANUAL** - Manual mode
* **ACRO** - Acrobatic mode
* **ANGLE** - Stabilized flight mode
* **HORIZON** - Horizon flight mode
* **ALT H** - Altitude Hold
* **POS H** - Position Hold
* **P+AH** - Altitude and Position Hold
* **CRS** - Cruise mode (Will keep the course)
* **3CRS** - 3D Cruise mode (It'll keep the altitude and the course)
* **RTH** - Return to home
* **WP** - Waypoint mission mode

This element also indicates if aircraft is armed or not. If it's **GREEN**, means that aircraft is armed. If it is not armed, then this element is **WHITE**.

Also, this element will indicate if aircraft is in failsafe state or not. If the element is blinking and alternating the flight mode with "**!FS!**", means that aircraft is in failsafe state.

#### UAV Location

It's a PlusCode indicating the exact location of the aircraft. PlusCodes can be converted into GPS Coordinates. But you don't have too. Just click it and it'll open the Google Maps (or Apple Maps on iOS) and show the aircraft location in a way that you can make a route to it. Plus Codes are supported by Google maps, so you can type it directly on the search bar too.

#### Home distance

It's the distance between the home point and the aircraft.

#### WP Mission

I'll show the mission status, if there's a mission loaded.

It shows the current waypoint and the total number of waypoints. Also, it'll be green if the mission is valid.

#### GPS Status

Show the GPS satellite count and the HDOP value.

It also becomes green if there's a 3D fix, or red if there's no 3D fix.

#### Battery Voltages

Shows the total voltage of the aircraft battery pack, and the average cell voltage of the battery.
It also changes the color based on battery capacity percentage informed by Flight Controller.

* Less than 20% = Red
* Between 21% and 60% = Yellow
* More than 60% = Green

#### RSSI

Shows the Signal Strength of the Radio Control. It also changes the color based on the value.

* Less than 20% = Red
* Between 21% and 60% = Yellow
* More than 60% = Green

#### Amp Draw

Show the current draw of the aircraft. It's useful to see if aircraft is pulling too much power from the battery.

#### mAh Used

Shows the used capacity of the battery.

#### Throttle

Show the throttle percentage, it should match the throttle stick position.

It also changes color indicating if flight controller is controlling the throttle (like on an automatic flight mode). When auto throttle is enabled, this element becomes yellow.

#### Efficiency

Shows the instantaneous efficiency of the aircraft (by default, mAh per Km). It's calculated by UI using the current speed and current amp draw, so this value may be a bit different from the one shown by INAV OSD.

#### Azimuth / Elevation

Show the Azimuth and Elevation to the aircraft.
If the element is GREEN, it's showing the Azimuth and elevation from the USER to the Aircraft. It means that user can move freely and the element will recalculate the Azimuth and Elevation from user location. This information is only available if user granted location access to the UI.

If the element is WHITE, it's showing the Azimuth and elevation from the HOME POINT to the Aircraft. It means that if user moves, element will not recalculate based on his/her location, it'll keep recalculating based only on the home point.

#### Uptime / Flight time

Both information keeps alternating if aircraft is not armed, but if aircraft is armed, only Flight time is shown.

Uptime shows how much time the aircraft is powered on, and Flight time shows how much time aircraft is armed.

Aircraft will update this timers only each minute, and UI will keep counting the time, so it may take up to 1 minute to this information appear on the screen.

#### Status messages

The bottom-most element is a space for text messages. It tries to emulate the same element on the INAV OSD, showing messages to the user regarding flight status. Some of the messages that it'll show are:

* MQTT Broker not connected
* Last message: (time) - It'll show when there are more than 5 seconds without messages from the aircraft
* En route to home
* En route to WP [currentWp] / [WpCount]
* Landing
* Emergency landing
* !RX RC Link lost!

### Map

The map takes a big portion of the screen. On the map, you can see:

![Map](screenshots/ui_map.png)

#### Home point

<img src="https://user-images.githubusercontent.com/17026744/104850455-f8c2fb80-58cd-11eb-87c0-18ddc5ba7627.png" width="100">

The little green house icon is the home point location (as set by INAV). Next to it, it's the altitude in relation to mean sea level (MSL).

#### Mission Waypoints

Grey | Green | Blue
-- | -- | --
<img src="https://user-images.githubusercontent.com/17026744/104850395-9407a100-58cd-11eb-9d9a-0d70440faadb.png" width="100"> | <img src="https://user-images.githubusercontent.com/17026744/104850396-9407a100-58cd-11eb-8fb4-205e38a1cd3d.png" width="100"> | <img src="https://user-images.githubusercontent.com/17026744/104850394-92d67400-58cd-11eb-9883-e3fb6c0fbd6e.png" width="100">
Past waypoints | Current waypoint | Next waypoints

Each waypoint will also have some text next to it, indicating the Elevation (the ground altitude in relation to mean sea level), the Waypoint altitude (set by the mission on INAV), and the calculated aircraft altitude of the waypoint in relation to the ground.

#### Map Controls

Two buttons are available in the top-right corner of the map:

- **Compass** — shows the current map bearing. Tap it to reset the map to north-up orientation.
- **Center on aircraft** — re-centres the map on the aircraft and resumes automatic tracking (same as if the user had not panned away).

#### User location

<img src="https://user-images.githubusercontent.com/17026744/104850729-9a971800-58cf-11eb-8df2-dbd8d31242d2.png" width="100">

Shows the user location and heading on the map. It'll show only if you allowed the web browser to use your location.

#### Aircraft

<img src="https://user-images.githubusercontent.com/17026744/104850757-bdc1c780-58cf-11eb-8251-9eb3d1f945f6.png" width="100">

Shows the aircraft location and heading on the map.

### EFIS PFD

![EFIS PFD](screenshots/ui_efis.png)

The Electronic Flight Information System Primary Flight Display shows the aircraft most important attitude information.

#### Artificial Horizon

Is an attitude indicator, a flight instrument that informs the pilot of the aircraft orientation relative to Earth's horizon, and gives an indication of orientation changes. Attitude is always presented to users in the unit degrees (°).

#### Ground Speed

The bar on the left-hand side of the EFIS is a Ground Speed indicator. It shows the speed using the unit set on the UI settings page. The default is "Km/h".

#### Altitude

The bar on the right-hand side of the EFIS is a Altitude indicator. The altitude is informed by INAV and is in relation to the home point altitude. It show the altitude using the unit set on the UI settings page. The default unit is meters (up to 1000 meters), then kilometers.

#### Vertical Speed

The gauge on the right-most side of the EFIS is a vertical speed indicator. The needle will indicate the vertical speed of the aircraft. The default measurement unit of this instrument is meters per second, but it can be changed on the UI settings screen.

#### Course

The bottom-most ruler bar is the course indicator. It will show the direction that aircraft is flying to.

### Sidebar Menu

Tapping the hamburger/menu icon opens the sidebar. The first-level menu items are:

- **Send command to UAV** — opens the Commands panel (see below). Also accessible via the floating **CMD** button that appears in the bottom-right corner of the screen.
- **Flight Sessions** — opens the Flight Sessions panel to manage recorded flight sessions.
- **Monitor other UAVs** — opens the secondary aircraft monitoring panel (see below).
- **Mission planner** — opens the full-screen mission planner (see below).
- **Keep screen awake** — keeps the smartphone screen lit at all times (see below).
- **Enable compass heading** — (iOS only) requests permission to use the device compass sensor so the user location marker rotates to match the direction you are facing. On Android and desktop the compass starts automatically; this button only appears on iOS 13+.
- **Settings** — opens the Settings submenu (see below).
- **Refresh App** — reloads the page to pick up a new version.
- **Help** — links to the documentation.
- **Install App** — appears on compatible browsers; installs the PWA to the home screen.

### Settings Menu

Tapping **Settings** opens a settings submenu with the following sections:

#### Broker Settings

Allows the user to set up the MQTT Broker details: host, port, username, password, and two topics:

- **Telemetry Topic** — the topic the firmware publishes telemetry on (format: `bulletgcss/telem/<callsign>`). The UI subscribes to this topic to receive aircraft data.
- **Command Topic** — the topic the UI publishes commands on (format: `bulletgcss/cmd/<callsign>`). The firmware subscribes to this topic to receive commands from the UI.

Both topics must use the same callsign suffix and match the values set in `Config.h` on the firmware.

#### UI Settings

Allows the user to set the units of measurement for every aspect of the UI. Speed, Altitude, Distance, Current, Power, etc.

Also allows the user to choose the Terrain elevation data provider. The default provider is "Open Topo Data (Direct)", which works best, but there are two alternative options if the primary method does not work.

Also allows the user to choose the **Map Style**:
- **Liberty (OpenFreeMap)** — default. A clean vector map with terrain contours, water, and land cover. Suitable for most flying conditions.
- **Dark (CARTO)** — a dark-themed map that matches the UI and is easier to read in bright sunlight.

#### Security

Opens the Security panel, where you manage the Ed25519 key pair used to authenticate commands sent to the aircraft.

- **Status line** — shows the current key state:
  - *No key pair configured* — no key has been generated yet.
  - *Key pair ready. Waiting for firmware telemetry…* — a key exists in the browser but the firmware's public key field (`pk`) has not been received yet.
  - *⚠ Firmware has no key* — the firmware is broadcasting an all-zeros public key. Paste the public key declaration into `Config.h` and re-flash.
  - *⚠ Firmware has a key but the UI key is missing* — the browser's key was cleared. Regenerate and re-flash.
  - *✗ Key mismatch* — the firmware has a different key than the UI. Re-flash with the current public key.
  - *✓ Keys match* — both sides share the same key; signed commands will be accepted.
- **Generate key pair** — creates a new Ed25519 key pair using the browser's Web Crypto API. The private key is stored in `localStorage` (never leaves the device); the public key is stored as hex and base64.
- **Public Key for Config.h** — a ready-to-paste C declaration for the `commandPublicKey` array. Copy it into `Config.h` and re-flash the firmware to enable command verification.

> **Browser support:** Ed25519 requires Chrome 113+, Firefox 130+, or Safari 17+.

#### INAV Settings

Reserved for future use — will allow reading and writing INAV settings from the UI.

### Monitor Other UAVs Panel

Tapping **Monitor other UAVs** opens a panel where you can track additional aircraft on the same map alongside the primary aircraft. Secondary aircraft are read-only — no commands, no EFIS — just situational awareness.

The panel shows the list of currently monitored topics, each with a colour swatch, callsign (once received), MQTT topic, and a **Remove** button. To add an aircraft, type its full telemetry topic (e.g. `bulletgcss/telem/GOLF2`) into the input field and tap **Add topic**. The input is pre-filled with the same topic prefix as the primary aircraft to make it easy to add aircraft on the same broker.

Each secondary aircraft appears on the map as a coloured icon (different hue for each) with a small label showing callsign, altitude, climb/descent indicator, and speed in the units configured in UI Settings. Tapping a secondary aircraft marker opens a popup with its last-seen time and Plus Code location. The popup also has a **Stop tracking** button.

Secondary aircraft that have not sent a message for more than 10 seconds are dimmed to 50% opacity. They are never removed automatically — only when the user explicitly removes the topic.

The monitored topic list is saved to `localStorage` and restored automatically on page load and MQTT reconnect.

---

### Mission Planner

<img src="screenshots/ui_missionplanner_overview.png" width="400">

Tapping **Mission planner** opens a full-screen view that completely replaces the monitoring UI. Tap **Close** to return to monitoring at any time.

#### Top bar

The top bar shows:
- **Connection icon** and **Command channel icon** — mirrors of the main UI status icons, so you can monitor aircraft connectivity without leaving the planner.
- **Mission validity dot** — a small circle that is **green** when the flight controller reports the loaded mission as valid (`wpv:1`) or **red** otherwise. This reflects the mission currently on the FC, not the one being planned.
- **Mission name** — the name of the currently loaded/saved mission, or *Untitled mission* if unsaved. An asterisk (`*`) is appended when there are unsaved changes.
- **Waypoint count and total distance** — updates live as waypoints are added, moved, or deleted.

#### Placing and editing waypoints

<img src="screenshots/ui_missionplanner_modal.png" width="400">

Tap or click any empty point on the map to add a waypoint there. The waypoint parameter modal opens immediately.

The modal shows:
- **WP #** — waypoint number (read-only).
- **Action** — dropdown: Waypoint (fly-through), Loiter (hold position for N seconds), RTH (return to home), Land.
- **Altitude** — metres above the home/takeoff point.
- **Terrain elevation** — shown automatically (when an elevation provider is configured) as *Terrain: XXX m | ~YYY m above terrain*, where YYY is the clearance above the ground at that point relative to the first waypoint's elevation. Updates live as you type the altitude.
- **Speed** — m/s cruise speed to this waypoint (0 = use FC default). Shown for Waypoint and Loiter actions.
- **Loiter time** — seconds to hold (shown for Loiter action only).

Confirm with **OK** or tap outside the modal to save changes. Use **Delete** to remove the waypoint.

> **RTH constraint:** RTH must be the last waypoint in a mission. If you set a waypoint to RTH and there are waypoints after it, the planner will ask to delete them. You cannot add new waypoints after an RTH.

Tap an existing waypoint marker to reopen its modal. Drag a marker to move it — the route line updates in real time.

#### Toolbar

![Mission planner workflow](screenshots/mp_workflow.png)

Two rows of buttons at the bottom of the screen:

**Row 1 — mission file management:**
- **Save** — saves the current mission to browser local storage with a name you provide. Overwrites an existing mission of the same name after confirmation.
- **Load** — opens a list of saved missions; tap one to load it (replaces the current mission after confirmation).
- **Export** — downloads the mission as an INAV-compatible JSON file (same format as INAV Configurator).
- **Import** — loads a mission from an INAV JSON file on your device.

**Row 2 — aircraft operations:**
- **Upload** — sends the planned mission to the aircraft over the encrypted command channel. Each waypoint is sent individually and acknowledged by the firmware before the next is sent. The FC's mission is only updated once the entire mission passes validation. Disabled when not connected or when WP Mission mode is active on the FC.
- **Download** — fetches the mission currently stored on the aircraft and loads it into the planner. Useful to review or modify a previously uploaded mission.
- **Clear** — removes all waypoints (confirmation required).
- **Close** — returns to the normal monitoring view.

#### Terrain elevation

If a terrain elevation provider is configured in UI Settings, the planner automatically queries ground elevation for each waypoint:
- **When placing a new waypoint** — elevation is fetched immediately and shown in the modal.
- **When loading, importing, or downloading** — elevations for all waypoints are fetched in a single batch request.

The first waypoint's elevation is used as the base (approximating the home/launch point). The *above terrain* figure in each modal is: `altitude above home − (terrain elevation at this WP − terrain elevation at WP 1)`.

---

### Flight Sessions Panel

Tapping **Flight Sessions** opens the panel, where you can manage recorded flight sessions. Every MQTT message received during a live flight is automatically saved to a local database (IndexedDB) in the browser. Sessions persist across page refreshes and browser restarts.

The panel shows:
- **Current session** — the name of the active session. You can rename it and tap "Rename" to save, or tap "New Session" to close the current session and start a fresh one.
- **Saved sessions** — a list of all sessions (newest first), showing the session name, start date, and duration. Each closed session has a **Replay** button to review it in the playback UI, and a **Delete** button to remove it permanently.
- **Export session** — saves the current session's messages as a TXT file. Useful for debugging or sharing a flight log.
- **Import session** — opens a file picker to load a previously exported session file and replay it.

When a session replay ends (or is stopped manually), the UI automatically restores the last known state of the live session.

### Send Command to UAV Panel

Tapping **Send command to UAV** (or the **CMD** floating action button) opens the Commands panel, where you can send signed commands to the aircraft.

Available commands:

| Command | Description |
|---|---|
| **RTH** | Activates/deactivates Return to Home |
| **Altitude Hold** | Activates/deactivates Altitude Hold mode |
| **Cruise** | Activates/deactivates Cruise mode |
| **Waypoint Mission** | Activates/deactivates Waypoint/Mission mode |
| **Beeper** | Activates/deactivates the aircraft beeper |
| **Ping** | Sends a no-op command to verify the downlink channel is working |

Each command has **ON** and **OFF** buttons. The buttons show visual state feedback: green border (mode confirmed active), dimmed (mode confirmed inactive), default (state unknown). State is updated from telemetry — buttons reflect the actual FC state, not just the last command sent.

The **command history** list shows every command sent in this session, with its type, timestamp, and status (sent / received / lost). A command is marked **received** when the firmware sends back an acknowledgement (`cid` match), or **lost** if no ack arrives within 10 subsequent telemetry messages.

Commands require a private key in the Security panel — if none is present, the command will not be sent. Check the [Command Channel Status](#command-channel-status) icon before sending flight-mode commands — the icon must be in the **OK** (green) state for RC commands to take effect on the flight controller.

### Keep Screen Awake

When enabled, it keeps the smartphone screen lit at all times. Bullet GCSS needs the screen on, otherwise the browser may suspend and stop receiving messages.

This feature does not work reliably on Apple devices. On iPhone/iPad, set the display to never sleep: **Settings → Screen & Brightness → Auto-Lock → Never**.
