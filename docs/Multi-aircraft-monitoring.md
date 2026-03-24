# Monitoring Multiple Aircraft

> **Note:** Multi-aircraft monitoring is not yet supported. The current UI can only subscribe to one MQTT topic at a time and displays telemetry for a single aircraft. This page documents the planned feature.

---

## Current behaviour

Each ESP32 modem reads the aircraft's callsign from the flight controller at startup and publishes telemetry to:

```
bulletgcss/telem/<callsign>
```

The UI subscribes to one topic at a time (configured in **Broker settings**). To monitor a different aircraft you must change the topic and reconnect — you cannot watch two aircraft in the same browser tab simultaneously.

As a workaround, you can open multiple browser tabs, each configured to a different topic, but each tab has its own independent map and data panel with no combined view.

---

## Planned feature

Multi-aircraft monitoring is planned as a future enhancement. The intended design:

- The UI subscribes to **multiple topics** simultaneously, one per aircraft.
- All aircraft appear on the **same map**, each with its own icon.
- A **quick-selector** in the UI lets the operator switch which aircraft's telemetry is shown in the data and EFIS panels.

See the [TODO list](TODO.md) for the current status of this feature.
