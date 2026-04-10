# Modem Task Refactor Plan

## Goal

Move all TinyGSM/MQTT networking code out of the Arduino `loop()` task and into a
dedicated FreeRTOS task (`modemTask`). This isolates the blocking modem operations so
that `fcTask` can detect a frozen modem, kill the task, hardware-reset the SIM7600,
and restart it — without rebooting the entire ESP32.

---

## Current Architecture

```
Core 1
  ├── fcTask    (priority 2) — all MSP/FC communication
  └── loop()   (priority 1) — TinyGSM, MQTT connect, client.publish(), client.loop()
```

The problem: when `loop()` blocks inside `AT+CIPSEND` (e.g. half-open TCP from
cellular NAT timeout), `fcTask` can detect the freeze via the `loopLastAliveMs`
heartbeat, but it cannot recover without `ESP.restart()` — because TinyGSM is stuck
inside the Arduino loop task and cannot be safely called from `fcTask`.

---

## Proposed Architecture

```
Core 1
  └── fcTask     (priority 2) — MSP/FC communication + modem watchdog

Core 0
  └── modemTask  (priority 1) — TinyGSM, MQTT connect, publish, client.loop()

loop()  — empties after setup(), does nothing
```

Because `modemTask` owns TinyGSM exclusively (no other task ever calls TinyGSM
functions), thread safety within TinyGSM is not an issue. If `modemTask` hangs,
`fcTask` can kill it and restart it cleanly.

---

## Data Flow

The existing shared-state mechanism (`publishedStatus`, `publishedMission[]`,
`dataMutex`, `cmdMutex`) is already correct and does not need to change. The only
difference is that the reader side moves from `loop()` to `modemTask`:

```
fcTask  →  writes publishedStatus / publishedMission[]  (under dataMutex)
             ↓
modemTask  →  reads publishedStatus / publishedMission[] (under dataMutex)
             → builds MQTT message strings
             → client.publish()
```

`mqttCommandCallback` is invoked from within `client.loop()`, which runs inside
`modemTask`. The callback already writes command state under `cmdMutex`, which
`fcTask` reads under the same mutex — no change needed.

`downlinkActive` moves from being set in `loop()` to being set in `modemTask`.
It is still a `volatile bool`, still read by `fcTask` — no change needed.

---

## Recovery Mechanism

### Heartbeat

Replace `loopLastAliveMs` (written by `loop()`) with `modemLastAliveMs` (written by
`modemTask` at the top of every iteration):

```cpp
volatile uint32_t modemLastAliveMs = 0;
```

`fcTask` reads this the same way it currently reads `loopLastAliveMs`.

### Watchdog thresholds (unchanged)

| Silence | Action |
|---|---|
| > 15 s | Log warning once per second |
| > 30 s | Kill `modemTask`, hardware-reset modem, restart `modemTask` |

### Recovery sequence (new — replaces `ESP.restart()`)

When `fcTask` decides to recover:

1. **Kill the frozen task**
   ```cpp
   vTaskDelete(modemTaskHandle);
   modemTaskHandle = NULL;
   ```

2. **Hardware-reset the modem** (GPIO only — safe from any task)

   For SIM7600:
   ```cpp
   digitalWrite(MODEM_POWER_ON, LOW);
   vTaskDelay(pdMS_TO_TICKS(500));
   digitalWrite(MODEM_POWER_ON, HIGH);
   vTaskDelay(pdMS_TO_TICKS(500));
   digitalWrite(MODEM_PWKEY, HIGH);
   vTaskDelay(pdMS_TO_TICKS(500));
   digitalWrite(MODEM_PWKEY, LOW);
   ```

   For SIM800:
   ```cpp
   digitalWrite(MODEM_RST, LOW);
   vTaskDelay(pdMS_TO_TICKS(200));
   digitalWrite(MODEM_RST, HIGH);
   ```

3. **Reset modem-side state** so `connectToGprsNetwork()` runs the full cold-start
   path again:
   ```cpp
   modemInitialized    = false;
   gprsFailureCounter  = 0;
   failureCounter      = 0;
   downlinkActive      = false;
   modemLastAliveMs    = 0;
   ```

4. **Restart the task**
   ```cpp
   xTaskCreatePinnedToCore(modemTask, "modemTask", 8192, NULL, 1, &modemTaskHandle, 0);
   ```

`modemTask` starts fresh, calls `connectToGprsNetwork()` (cold-start path), reconnects
to the broker, and resumes publishing. Total recovery time: ~15–30 s (modem boot +
network registration), instead of a full ESP32 reboot.

---

## Step-by-Step Implementation

### Step 1 — Rename the heartbeat variable

- Rename `loopLastAliveMs` → `modemLastAliveMs` everywhere.
- Change the warning/restart log messages from "loop task frozen" to "modem task frozen".

### Step 2 — Store the modem task handle

Add a global task handle so `fcTask` can kill it:

```cpp
TaskHandle_t modemTaskHandle = NULL;
```

### Step 3 — Create `modemTask()`

Extract the body of `loop()` and `sendMessageTask()` into a new function:

```cpp
void modemTask(void* param) {
    // Cold-start: connect modem / WiFi
    connectToTheInternet();

    while (true) {
        modemLastAliveMs = millis();

        // Reconnect if needed (same logic as sendMessageTask today)
        connectToTheInternet();
        connectToTheBroker();

        // Build and publish telemetry
        sendMessageTask();

        // Drive PubSubClient receive loop (keepalive, callbacks)
        client.loop();

        // Check failure counter
        if (failureCounter >= 10) {
            LOGLINE("Failure count too high — restarting modem task");
            // Signal fcTask to do a clean modem restart instead of crashing
            modemLastAliveMs = 0; // force watchdog to fire
            vTaskDelay(portMAX_DELAY); // block here; fcTask will kill us
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // yield briefly between iterations
    }
}
```

> **Note:** `sendMessageTask()` already has its own interval check
> (`MESSAGE_SEND_INTERVAL`), so calling it in a tight loop with a short `vTaskDelay`
> is fine — it will no-op until the interval expires.

### Step 4 — Update `fcTask()` recovery logic

Replace the `ESP.restart()` call with the recovery sequence from the section above:

```cpp
if (modemSilenceMs > LOOP_FREEZE_RESTART_MS) {
    LOGLINE("FATAL: modem task frozen for %lu s — killing and restarting modem",
            modemSilenceMs / 1000);

    vTaskDelete(modemTaskHandle);
    modemTaskHandle = NULL;

    // Hardware reset (SIM7600 / SIM800 — compile-time selected)
    #ifdef TINY_GSM_MODEM_SIM7600
    digitalWrite(MODEM_POWER_ON, LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(MODEM_POWER_ON, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(MODEM_PWKEY, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(MODEM_PWKEY, LOW);
    #elif defined(TINY_GSM_MODEM_SIM800)
    digitalWrite(MODEM_RST, LOW);
    vTaskDelay(pdMS_TO_TICKS(200));
    digitalWrite(MODEM_RST, HIGH);
    #endif

    // Reset modem state
    modemInitialized   = false;
    gprsFailureCounter = 0;
    failureCounter     = 0;
    downlinkActive     = false;
    modemLastAliveMs   = 0;

    // Restart modemTask
    xTaskCreatePinnedToCore(modemTask, "modemTask", 8192, NULL, 1, &modemTaskHandle, 0);
    LOGLINE("modemTask restarted — waiting for reconnect");
}
```

### Step 5 — Update `setup()`

- Remove the `connectToGprsNetwork()` call from `setup()` (modemTask handles it).
- Launch `modemTask` instead of relying on `loop()`:

```cpp
void setup() {
    SerialMon.begin(115200);
    // ... NVS load, mspSerial, client.setServer, client.setCallback ...

    dataMutex = xSemaphoreCreateMutex();
    cmdMutex  = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(fcTask,    "fcTask",    4096, NULL, 2, NULL,              1);
    xTaskCreatePinnedToCore(modemTask, "modemTask", 8192, NULL, 1, &modemTaskHandle,  0);
}
```

### Step 6 — Empty `loop()`

```cpp
void loop() {
    vTaskDelay(portMAX_DELAY); // loop() is unused; yield forever
}
```

### Step 7 — Remove `loopLastAliveMs` write from `loop()`

Already gone since `loop()` is now empty. `modemLastAliveMs` is written inside
`modemTask` instead.

---

## Stack Size

`modemTask` needs more stack than the current Arduino loop task because:
- `buildTelemetryMessage()` and `buildLowPriorityMessage()` use a local `char message[512]`
- TinyGSM + SSLClient have their own internal buffers on the stack

Recommended: **8192 bytes** (vs. 4096 for `fcTask`). Monitor with
`uxTaskGetStackHighWaterMark(modemTaskHandle)` if stack issues arise.

---

## WiFi Path

The WiFi path (`USE_WIFI`) does not block in the same way (the ESP32 TCP stack has
native non-blocking timeouts), so the watchdog-triggered modem restart is guarded by
`#ifndef USE_WIFI`. For WiFi builds the recovery path remains `ESP.restart()` via
`failureCounter`, which is sufficient.

---

## Risks and Open Questions

| Risk | Mitigation |
|---|---|
| `vTaskDelete()` on a task holding a mutex leaves the mutex locked forever | `modemTask` only holds `dataMutex` for a brief `memcpy` at message-build time. If killed mid-hold, `fcTask`'s next `xSemaphoreTake(dataMutex)` will deadlock. **Fix:** use a timeout (`xSemaphoreTake(dataMutex, pdMS_TO_TICKS(500))`) rather than `portMAX_DELAY` in `fcTask`, and reset the mutex after killing the task (`vSemaphoreDelete` + `xSemaphoreCreateMutex`). |
| `modemTask` killed while `gsmClient` is mid-write corrupts SerialAT state | The hardware modem reset (Step 4) immediately follows the kill, which clears the SIM7600's internal state. `SerialAT` is re-initialised inside `connectToGprsNetwork()` (cold-start path). |
| GPIO reset sequence for SIM7600 may differ from board to board | The existing cold-start sequence in `connectToGprsNetwork()` already works. The reset sequence in Step 4 mirrors it. Worth verifying on the actual hardware after the refactor. |
| `modemTask` stack overflow during reconnect | Set stack to 8192 bytes and add a high-water-mark log at the end of each successful reconnect cycle. |
