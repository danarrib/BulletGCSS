

# Bullet GCSS
Bullet GCSS is a high caliber ground control station system designed for the 21st century lifestyle.

![github-social-media](https://user-images.githubusercontent.com/17026744/103979227-1ff62c00-515c-11eb-922b-a433d89bbf5e.png)

**This is a Work in progress! It's not ready for consistent use. Be advised!**

Bullet GCSS allows an UAV pilot/operator to get the most important telemetry data right on his/her SmartPhone or computer screen. Information such as aircraft location, distance, altitude, battery status, navigation status are always available.

The main differences between Bullet GCSS and other traditional ground station systems are:

 - It works using Cellular Data network, which means that there's no maximum range. You'll know about your aircraft as long as it's inside a the cellular network coverage area.
 - It doesn't require any app to be installed on your SmartPhone or computer. It's just a Web page that opens directly inside the Web Browser.
 - Bullet GCSS can also be installed on the SmartPhone as a Web App, giving the same experience but taking the full SmartPhone screen, making it look even nicer!
 - It works both on Android Phones/Tablets and iPhone/iPad, works on any PC too (Windows, Linux, Mac). In fact, it probably works on your Windows Phone too. That's the beauty of Web Apps.

## How it works?
There are two fundamental parts on Bullet GCSS: The **Modem** and the **User Interface (UI)**.

### Modem
Modem is a **[TTGO T-Call Board](https://www.banggood.com/custlink/DDvySdDajK)**. It's a prototyping single board that contains an ESP32 Microcontroller Unit (MCU) and a SIM800L module, which is a GPRS Modem.

![image](https://user-images.githubusercontent.com/17026744/103963263-8fa5f000-5137-11eb-8240-86cc32e3aefc.png)

The board must be connected to the Flight Controller board (running INAV), on any available UART, using MSP protocol.

Program that runs on the ESP32 MCU (Firmware) talks to the Flight Controller board to get all the telemetry data that will be presented to the user.

Then, the firmware sends this data to a MQTT Broker on the Internet. This Broker will be the central data exchange point between the aircraft and the user.
### User Interface (UI)
The UI is a single HTML page, that can be hosted on any web server. It doesn't require any special server side structure, there are no database, no server side scripts or programs, just a simple basic web server will be enough.

This page has one EFIS (Electronic Flight Instrument System), a map and an information panel.

![image](https://user-images.githubusercontent.com/17026744/103963170-62f1d880-5137-11eb-80bc-be0e77b52497.png)

The UI fetch all the information from the same MQTT Broker that modem uses to send the data, using WebSockets.

## How can I use it?
Check out the [Wiki](https://github.com/danarrib/BulletGCSS/wiki) for detailed instructions.
