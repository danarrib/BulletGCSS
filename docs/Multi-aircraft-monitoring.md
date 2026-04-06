# Monitoring Multiple Aircraft

Bullet GCSS can display multiple aircraft simultaneously on the same map. Each additional aircraft is monitored as a **secondary aircraft** — its position, telemetry, and flight path appear alongside the primary aircraft without replacing any of the primary data or EFIS readings.

---

## How it works

Each ESP32 modem publishes telemetry to a unique MQTT topic derived from the aircraft's callsign:

```
bulletgcss/telem/<callsign>
```

The primary aircraft topic is configured in **Settings → Broker Settings**. Secondary aircraft are additional topics you subscribe to manually. The UI subscribes to all topics simultaneously and maintains independent state for each aircraft.

Secondary aircraft are automatically assigned a distinct icon colour so you can tell them apart at a glance on the map. The colour is preserved across browser sessions — the same topic always gets the same colour within a session.

Monitored topics are saved in the browser's `localStorage` and restored automatically every time you open the app.

---

## Adding a secondary aircraft

1. Open the **side menu** (gear icon, top-left).
2. Tap **Monitor other UAVs**.
3. In the text field, enter the MQTT topic of the aircraft you want to monitor. The field is pre-filled with the same broker prefix as your primary aircraft — you usually only need to change the callsign at the end.

   **Example:** if your primary topic is `bulletgcss/telem/FALCON1`, type `bulletgcss/telem/FALCON2`.

4. Tap **Add topic**.

The aircraft will appear on the map as soon as telemetry messages start arriving.

---

## Viewing secondary aircraft on the map

Each secondary aircraft is shown with:

- A **coloured aircraft icon** (rotated to match heading).
- A **label** showing callsign and altitude relative to the primary aircraft's home point.
- A **flight path trail** drawn in the same colour as the icon.

Tap the aircraft icon or its label to open the **aircraft detail popup**.

---

## Aircraft detail popup

Tapping a secondary aircraft opens a popup with a full telemetry summary:

| Field | Description |
|---|---|
| **Callsign** | Aircraft callsign (coloured border matches map icon) |
| **Last seen** | Time since the last MQTT message was received |
| **Plus Code** | Open Location Code coordinate — tap to copy to clipboard |
| **Altitude** | Altitude relative to the primary aircraft's home point |
| **Vert. Speed** | Vertical speed (positive = climbing) |
| **Horiz. Speed** | Ground speed |
| **Heading** | Magnetic heading in degrees |
| **Distance** | Distance from the primary aircraft |
| **Azimuth** | Bearing from the primary aircraft to this aircraft |
| **Battery** | Battery charge percentage |
| **Flight Time** | Time the aircraft has been armed and flying |
| **Home Distance** | Distance from this aircraft to its own home point |

The popup stays open and refreshes automatically. Tap **Close** or anywhere outside the popup to dismiss it.

---

## Removing a secondary aircraft

There are two ways to stop monitoring an aircraft:

**From the popup:** Tap the aircraft icon on the map to open the detail popup, then tap **Stop tracking**. A confirmation dialog will appear before the aircraft is removed.

**From the monitor list:** Open the side menu → **Monitor other UAVs**. Each monitored aircraft is listed with a **Remove** button next to it.

Removing an aircraft unsubscribes from its MQTT topic, clears its icon and flight path from the map, and removes it from `localStorage` so it will not be restored on the next app launch.

---

## Notes

- The primary aircraft's data panel, EFIS, and commands panel always show the **primary aircraft only**. Secondary aircraft are map-only.
- There is no hard limit on the number of secondary aircraft, but performance will degrade if you add a very large number of topics simultaneously.
- Secondary aircraft icons and flight paths are not shown during mission replay sessions.
- The secondary aircraft's altitude shown on the map and in the popup is calculated **relative to the primary aircraft's home point**, so all altitudes on the map use the same reference even when the aircraft took off from different locations.
