![Banner](https://user-images.githubusercontent.com/17026744/109589244-ca4d5880-7ae8-11eb-9c2f-4cdb2da3160a.png)

**Bullet GCSS is a high caliber ground control station system designed for the 21st century lifestyle.**

* [Thread on RCGroups](https://www.rcgroups.com/forums/showthread.php?3804715-Bullet-GCSS-Semi-asynchronous-internet-based-ground-control-station-system)
* [Official group on Telegram](https://t.me/bulletgcss)

## Welcome to the Bullet GCSS wiki!

Bullet GCSS is a semi-asynchronous internet-based ground control station system for INAV-Based UAVs.

**It's semi-asynchronous**, because it relies on the MQTT messaging protocol. Aircraft sends telemetry messages to a MQTT Broker on the Internet on a regular interval (default is 1 message per second). And the MQTT Broker sends this messages to the User Interface, which can be a SmartPhone or a PC connected to the Internet.

**It's internet-based**, because it uses a GPRS or LTE modem to send the messages over the Internet using the 2G/3G/4G cellular network. Messages sent by the aircraft has, on average, 60 bytes, which means that on the default 1Hz message frequency, a one hour flight should spend less than 300 kilobytes from your data plan.

### But how about the latency?

Latency will depend mostly on the update frequency. For the default 1Hz setting, it'll be ~1 second of latency.

**Low latency is not the first priority of Bullet GCSS**, it's not intended to allow pilot to manually control the aircraft by it. Instead, it's main purpose is to allow the pilot to know if the aircraft is flying as it should when it's beyond the radio control and video feed range, while performing an autonomous flight, like a pre-programmed waypoint mission, or returning to home (RTH) by itself.

### How it works?
There are two fundamental parts on Bullet GCSS: The **Modem** and the **User Interface (UI)**.

#### Modem
![TTGO T-Call and T-PCIE boards](https://user-images.githubusercontent.com/17026744/109437357-41a8bc80-7a03-11eb-9e79-1fe1f83f642a.png)

Modem can be a **[TTGO T-Call Board](https://www.banggood.com/custlink/DDvySdDajK)** or a **[TTGO T-PCIE Board with a SIM7600 Module](https://www.banggood.com/custlink/K3vdZaFKHl)**. Both are prototyping boards that contains an ESP32 Microcontroller Unit (MCU). The T-Call board contains a SIM800L module, which is a GPRS (2G) Modem, and the T-PCIE Board uses a SIM7600 module, which is a LTE (4G) Modem.

Both boards will work just fine. The difference between them is that the SIM800L works only on 2G networks and is cheaper, while the SIM7600 works over 4G networks and is more expensive and a bit bigger and heavier.

The board must be connected to the Flight Controller board (running [INAV](https://github.com/iNavFlight/inav)), on any available UART, using MSP protocol.

Program that runs on the ESP32 MCU (Firmware) talks to the Flight Controller board to get all the telemetry data that will be presented to the user.

Then, the firmware sends this data to a MQTT Broker on the Internet. This Broker will be the central data exchange point between the aircraft and the user.

#### User Interface (UI)

![image](https://user-images.githubusercontent.com/17026744/109589480-23b58780-7ae9-11eb-90e8-5faaf46a89d7.png)

The UI is a single HTML page, that can be hosted on any web server. This page has an EFIS (Electronic Flight Instrument System), a map and an information panel. Page works fine both on portait and landscape modes, and works fine with non-rectangular screens as well.

The UI fetch all the information from the same MQTT Broker that modem uses to send the data, using WebSockets.

### How can I use it?

* [What do I need?](Required-hardware.md)
* [How to configure the modem device?](Setup-modem.md)
* [How to install the modem device on my aircraft?](Wiring.md)
* [How to Host the UI?](Host-the-user-interface.md)
* [How to configure the UI?](User-Interface.md)
* [How to find a MQTT Broker](Find-a-MQTT-Broker.md)

### Can I see it?

![Deploy BulletGCSS UI on FPV Sampa](https://github.com/danarrib/BulletGCSS/workflows/Deploy%20BulletGCSS%20UI%20on%20FPV%20Sampa/badge.svg)

There's a Bullet GCSS UI hosted by Outros.net, which you can use free of charge. It has a continuous integration with the `master` branch of this repository, so every time the `master` branch is updated, the UI will be automatically updated on this address.

[https://bulletgcss.outros.net/](https://bulletgcss.outros.net/)

However, it will not show any data if there's no aircraft broadcasting on the default topic. It's nice to see anyway.

If you already set your modem up, you can use this UI and configure it to use for your particular settings.