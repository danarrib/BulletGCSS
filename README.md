

# Bullet GCSS
Bullet GCSS is a high caliber ground control station system designed for the 21st century lifestyle.

![Deploy BulletGCSS UI on FPV Sampa](https://github.com/danarrib/BulletGCSS/workflows/Deploy%20BulletGCSS%20UI%20on%20FPV%20Sampa/badge.svg)

![github-social-media](https://user-images.githubusercontent.com/17026744/109589244-ca4d5880-7ae8-11eb-9c2f-4cdb2da3160a.png)

Bullet GCSS allows an UAV pilot/operator to get the most important telemetry data right on his/her SmartPhone or computer screen. Information such as aircraft location, distance, altitude, battery status, navigation status are always available.

The main differences between Bullet GCSS and other traditional ground station systems are:

 - It works using Cellular Data network, which means that there's no maximum range. You'll know about your aircraft as long as it's inside a the cellular network coverage area.
 - It doesn't require any app to be installed on your SmartPhone or computer. It's just a Web page that opens directly inside the Web Browser.
 - Bullet GCSS can also be installed on the SmartPhone as a Web App, giving the same experience but taking the full SmartPhone screen, making it look even nicer!
 - It works both on Android Phones/Tablets and iPhone/iPad, works on any PC too (Windows, Linux, Mac). In fact, it probably works on your Windows Phone too. That's the beauty of Web Apps.

## How it works?
There are two fundamental parts on Bullet GCSS: The **Modem** and the **User Interface (UI)**.

Modem talks to the Flight Controller on the aircraft to get the telemetry data, and sends this data to a MQTT Broker on the Internet. The channel is bidirectional — the UI can also send commands to the aircraft (such as a ping, with more command types planned).

The UI is connected to this same MQTT Broker, and every time it gets a new telemetry message, it'll display it on the screen.

Check out this [Demonstration Video](https://youtu.be/Iwv_Eo0fOuc?si=-nH5KV7GBwPIXf3V&t=623), action starts at 10:23.

## A note on privacy

By default, Bullet GCSS uses a public MQTT broker. This means your aircraft's telemetry (GPS position, altitude, battery, etc.) can be seen by anyone who subscribes to the same topic. For most hobbyist flights this is perfectly fine — but if you'd rather keep your flights private, it's easy to set up your own broker. See [How to self-host a MQTT Broker](docs/Self-Hosting-a-MQTT-server--(broker).md).

## How can I use it?

- [What do I need?](docs/Required-hardware.md)
- [How to configure the modem device?](docs/Setup-modem.md)
- [How to install the modem device on my aircraft?](docs/Wiring.md)
- [How to Host the UI?](docs/Host-the-user-interface.md)
- [How to configure the UI?](docs/User-Interface.md)
- [How to find a MQTT Broker](docs/Find-a-MQTT-Broker.md)
- [How to self-host a MQTT Broker](docs/Self-Hosting-a-MQTT-server--(broker).md)
- [How much data will Bullet GCSS use?](docs/How-much-will-Bullet-GCSS-use-from-my-GPRS-data-plan?.md)
- [How to install Bullet GCSS on a SmartPhone](docs/How-to-install-Bullet-GCSS-on-SmartPhone.md)
- [Terrain elevation feature](docs/Terrain-elevation.md)
- [Communication Protocol Reference](docs/BulletGCSS_protocol.md)
- [Monitoring multiple aircraft simultaneously](docs/Multi-aircraft-monitoring.md)
- [Troubleshooting](docs/Troubleshooting.md)
