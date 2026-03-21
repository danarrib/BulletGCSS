The MQTT Broker is the central point between the aircraft and the user. The modem module sends messages with the telemetry data to the Broker, and the user interface that runs on user's SmartPhone receives this messages immediately after they are received by the Broker.

![Network-Diagram](https://user-images.githubusercontent.com/17026744/104266799-21329c00-546f-11eb-8683-f210ffe56ae3.png)

## What is MQTT ?

The MQTT is a lightweight messaging protocol designed for IoT devices (Internet of Things). It is designed as an extremely lightweight publish/subscribe messaging transport that is ideal for connecting remote devices with a small code footprint and minimal network bandwidth. If you want to know more about MQTT protocol, check it's [official website](https://mqtt.org/).

The MQTT Broker is a pre-requisite for Bullet GCSS, so you need to get access to one in order to use the system.

## What is the MQTT Broker used by Bullet GCSS?

There are some free (public or private) MQTT Brokers provided by different companies, with different levels of reliability. There are also paid ones, with different price ranges and capabilities.

You can also host your own MQTT Broker if you have a server somewhere over the Internet, there are plenty of free and paid software that you can use for this. [Check out this list](https://mqtt.org/software/) if you're interested in host your own MQTT Broker.

By default, Bullet GCSS uses the service provided for free by [EQM](https://www.emqx.io/), the EMQ X Cloud, which works fine most of the time. To connect to this Broker, just use the following details:

### On the modem device:

To change the MQTT parameters on the modem, edit the `Config.h` file before [flashing the firmware](https://github.com/danarrib/BulletGCSS/wiki/Setup-modem).

![image](https://user-images.githubusercontent.com/17026744/104539414-3ab51e80-55fc-11eb-8d15-e778d69ca5b2.png)

* Host: `broker.emqx.io`
* Port: `1883`
* Username: (any username will work)
* Password: (any password will work)
* Topic: (any topic that was at least 3 levels. Example: `bulletgcss/uavs/myaircraftname`)

### On the User Interface:

To change the MQTT parameters on the UI, click on the _gear_ icon on the top of the screen, and choose the _Broker settings_ option.

![image](https://user-images.githubusercontent.com/17026744/104539152-bfec0380-55fb-11eb-9279-4bd63f646313.png)

Settings will be saved on your device, so next time you open Bullet GCSS, it'll connect using the same settings.

* Host: `broker.emqx.io` (the same address used on the modem)
* Port: `8084` (this is the "WebSockets" secure port for this Broker)
* Username: (any username will work)
* Password: (any password will work)
* Topic: (The same topic used on the modem)
* Use TLS: Yes (keep the checkbox checked)

As you may noticed, this is a public broker, which means that your aircraft will send the flight telemetry to a place where anyone can read. Also, other people can even send messages to this same topic. This messages will be sent to the UI running on your SmartPhone and may show you incorrect telemetry data. 

**If privacy is a concern for you, then you should NOT use public brokers.**

> Note: For now, Bullet GCSS doesn't control the aircraft, so there's no risk of someone interfering on the actual flight.

## What if I want to use another MQTT broker provider?

You're highly encouraged to do so. You can subscribe to any service provider that offers you a MQTT broker service. Here are the requirements:
* MQTT Broker hostname (or IP address)
* Port for non-encrypted communication (used on the modem)
* Port for encrypted (TLS) WebSockets communication (used on the UI)
* Username and Password for both modem and UI
* Instructions on topic structure (if there are any constrains about that)
* Being able to handle ~5000 messages per hour per aircraft (around 500KB per hour) *** 

> *** This numbers refers to the default settings of Bullet GCSS (1 message per second). This is an incredible low number of messages. MQTT protocol is designed to handle millions of messages per minute.

Bullet GCSS doesn't sends your connection details outside the UI. So you don't need to worry about it sending your connection details to anyone.

## How to find MQTT providers?

Easiest way is just Google for "Hosted MQTT Broker". It'll give lots of results. I can't recommend any since I do not use any other than the one above.

