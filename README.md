

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

Modem talks to the Flight Controller on the aircraft to get the telemetry data, and sends this data to a MQTT Broker on the Internet.

The UI is connected to this same MQTT Broker, and every time it gets a new telemetry message, it'll display it on the screen.

## How can I use it?
Check out the [Wiki](https://github.com/danarrib/BulletGCSS/wiki) for detailed instructions.
