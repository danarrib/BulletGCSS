Bullet GCSS is an **Internet-based** ground control station system, which means that both the aircraft and the pilot needs to be connected to the internet.

![Network-Diagram](https://user-images.githubusercontent.com/17026744/104266799-21329c00-546f-11eb-8683-f210ffe56ae3.png)

The quick answer to that question is: **Around 470 KB per hour**, using the default settings.

But to understand this number, you have to know how Bullet GCSS works.

Bullet GCSS sends "messages" to the MQTT broker on a regular interval. The default interval for standard telemetry messages is 1 second, which means that every second, the aircraft will send this message type.

Additionally, at every 30 standard messages, it will also send one additional message for each waypoint loaded on a mission, which means that, if there's a mission loaded on INAV, every 30 seconds (standard telemetry message interval * 30) the aircraft will send one message for each waypoint. If you have a mission with 10 waypoints, it'll send 10 messages, all of them at same time (no delay between messages). 

Also, it'll send one additional message for the home point, and another additional message if the mission ends with a RTH command.

Ok, so for now we have:
- One message every second (60 messages per minute)
- N messages every minute (N is the waypoint count, +1 or +2, depending on the mission)

But what's the size of each message? It depends too.

### Standard telemetry messages

This type of message has every telemetry information, except for waypoint mission parameters. Some of the data that can be sent on this message type:
- Aircraft attitude information (roll angle, pitch angle)
- Aircraft GPS coordinates and Home point coordinates
- Aircraft Ground Speed, Vertical Speed, Altitude
- Battery voltage, power consumption, used battery capacity
- Flight mode, aircraft name (Callsign), Failsafe State, Navigation state
- and LOTs of other telemetry data.

But the modem device will not send all the data on every single message. It'll send only what changed since the last message.
If aircraft speed didn't change, it will not send this information again immediately. If battery voltage also didn't change, it'll not send it neither.

But there's a catch: With every message sent, modem will send SOME fields that was already sent, even if this particular field didn't change. It allows a user to open the UI at any time, even if the aircraft is already flying, and the UI will gradually filling all the needed information to present on the screen.

This way, at any given time, user can open the UI and, in less than 15 seconds, get it completely filled with current aircraft status.

Ok, so now you probably is thinking that is impossible to predict the messages sizes. Yes, that's true. But by my experience, each message has **an average of 110 bytes**. Some can get really small (less than 50 bytes), but some has more than 200 bytes.

This message is sent, by default, every second. Which means 3600 messages per hour, with an average of 110 bytes per message, 396KB.
You can change this to 1/2 second, or even less... But don't go too fast on this, because the 2G network is not as much reliable as 4G and can easily become too busy to send your messages.

### Waypoint messages

Every 30 standard messages, the modem device will send the entire loaded mission (if any). Each waypoint will take an average of 50 bytes. 
It'll also send the home point waypoint. If your waypoint mission ends with a RTH command, it'll send another message for this.

So if you have 10 waypoints, you can expect another 1200~1400 bytes per minute on the default setting. Which gives another 84KB per hour.

### MQTT Protocol overhead

But the messages from Bullet GCSS are not the only data that goes thru the network. The messages itself are wrapped into a MQTT request, and the MQTT protocol adds some amount of data. Also, the MQTT protocol itself relies on the TCP protocol, which also adds some overhead. So the final data usage will be slightly bigger. Let's take a look:

Data usage for transmitting aircraft telemetry for one hour with a 10-waypoint mission loaded 
- 3600 standard messages (110 bytes each)
- 1440 waypoint messages (50 bytes each)

| Activity | Bullet GCSS | MQTT | TCP | Total
|--|--|--|--|--|
| TCP Connect |  |  | 96 | 96 bytes
| MQTT Connect |  | 64 | 128 | 192 bytes
| MQTT Heartbeat |  | 4 | 96 | 100 bytes
| Bullet Std messages | 396000 | 15 | 64 | 396079 bytes
| Bullet Wp messages | 72000 | 15 | 64 | 72079 bytes
| **Total data per hour** |  |  |  | **468158 bytes** (468 KB)

Also, you have to remember that every message sent by the aircraft will be received by the SmartPhone, so the same amout of data will be used on the SmartPhone too.