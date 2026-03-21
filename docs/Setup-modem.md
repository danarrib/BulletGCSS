You'll need a computer to configure and flash the TTGO T-Call or the TTGO T-PCIE boards.

## Setting up the environment

### Install Arduino IDE

You'll need to install Arduino IDE 1.8.8 or newer on the official Arduino website: https://www.arduino.cc/en/software

### Add required libraries to the Arduino IDE

After installing Arduino IDE, open it and open the Menu _Tools > Manage Libraries..._

![image](https://user-images.githubusercontent.com/17026744/109442446-0fa15580-7a17-11eb-8d4a-b2496df76d21.png)

Install the following Libraries:
* WiFi by Arduino, version 1.2.7 or newer
* MQTT by Joel Gaehwiller, version 2.4.8 or newer
* PubSubClient by Nick O'Leary, version 2.8.0 or newer
* StreamDebugger by Volodymyr Shymanskyy, version 1.0.1 or newer
* TinyGSM by Volodymyr Shymanskyy, version 0.10.9 or newer

### Add ESP32 boards support for Arduino IDE

On Arduino IDE, click the menu _File > Preferences_, on the "Settings" tab, look for "Additional Boards Manager URLs" field, and fill it with the following text:

`https://dl.espressif.com/dl/package_esp32_index.json`

![image](https://user-images.githubusercontent.com/17026744/104496576-ebe59580-55b7-11eb-8647-9e4b141fa572.png)

After that, click "Ok" to close the Preferences window. Then, click on the menu _Tools > Board > Boards Manager...

![image](https://user-images.githubusercontent.com/17026744/109442562-52fbc400-7a17-11eb-9fc2-222456f612ae.png)
 
Install the following board 

* esp32 by Espressif Systems, version 1.0.4 or newer

### Setting up communication parameters

Connect your TTGO T-Call or TTGO T-PCIE board to the computer using an USB cable and wait for Windows to detect it and install the proper drivers for the COM port.

After that, on Arduino IDE, click the menu _Tools > Board. It'll list A LOT of different boards. Find the "DOIT ESP32 DEVKIT V1" and click to select it.

![image](https://user-images.githubusercontent.com/17026744/109442773-d3bac000-7a17-11eb-9fb2-56880f06f4dc.png)

Also, change the other parameters to match the required settings:
* Upload Speed: "921600"
* Flash Frequency: "80Mhz"
* Core Debug Level: "None"
* Port: It'll depend on your particular system. Windows will choose automatically a COM port. If you don't know which COM is your board, just disconnect it and check which COM port is now missing, then connect again and select the new COM port.

## Configuring the Bullet GCSS firmware

Download Bullet GCSS from it's official repository: https://github.com/danarrib/BulletGCSS

![image](https://user-images.githubusercontent.com/17026744/104498225-2c461300-55ba-11eb-8ceb-d24143dc83e3.png)

Unzip the file somewhere on your computer, then navigate to the **ESP32-Modem** directory and open the **ESP32-Modem.ino** file.

![image](https://user-images.githubusercontent.com/17026744/109443063-5fcce780-7a18-11eb-83a3-c3c81e7148d8.png)

Arduino IDE will open the file, and all the other files of the project on another tabs. Please notice if on the Status bar (bottom right corner of the Window) the Board parameters are correct.

![image](https://user-images.githubusercontent.com/17026744/109443157-9a368480-7a18-11eb-8cef-eca767d0f2d7.png)

Click on the `Config.h` tab to see the settings.

![image](https://user-images.githubusercontent.com/17026744/109443229-c81bc900-7a18-11eb-9812-df9c761708ff.png)

Every line on this file leads to a necessary setting that you need to choose. 

#### Use WiFi or GPRS

`#define USE_WIFI` - Tells the program if you want to use WiFi or GPRS. Use WiFi for bench tests, this way you don't need a GSM Sim Card or a Data plan. But for "actual" flight, you have to comment this line by adding two slashes at the beginning, like this: `// #define USE_WIFI`.

WiFi details can be provided on the `ssid` and `password` variables. `ssid` is the WiFi network name, and `password` is the password to connect to this network. Note: ESP32 can only connect to 2.4Ghz WiFi networks.

#### Modem type

You have to choose the proper modem you are going to use. 

If you have a TTGO T-Call board, then you have to keep the TINY_GSM_MODEM_SIM800 uncommented and the TINY_GSM_MODEM_SIM7600 commented:

![image](https://user-images.githubusercontent.com/17026744/109443431-4aa48880-7a19-11eb-846d-74eb710a83e1.png)

But if you have a TTGO T-PCIE with a SIM7600 module on it, then you have to comment the TINY_GSM_MODEM_SIM800 define and uncomment the TINY_GSM_MODEM_SIM7600 define:

![image](https://user-images.githubusercontent.com/17026744/109443523-89d2d980-7a19-11eb-8a0d-a3b72e0943d6.png)

#### MQTT settings

MQTT Broker details must be provided too. There's a specific topic for this subject [here](https://github.com/danarrib/BulletGCSS/wiki/Find-a-MQTT-Broker).

#### GSM details

Next, there are the GSM details. You need to find the correct APN, user and password for the GRPS connection. It'll depend on your local cellular carrier. You should inform this details regardless of using a 2G or a 4G network, because even using 4G, modem can fallback to 2G depending on network conditions.

#### Pooling settings

Polling settings tells to Bullet GCSS how frequently it needs to pool the telemetry from Flight Controller and send the messages to the Broker. The standard is 1000 (milliseconds), which is 1 second. You can increase it to a bigger number if you want it to be slower and transmits less, or to a smaller number if you want it to be faster and spend more from your data plan. Check [this topic](https://github.com/danarrib/BulletGCSS/wiki/How-much-will-Bullet-GCSS-use-from-my-GPRS-data-plan%3F) to know about it. If you are not sure what value to use, just keep the default 1000, it'll work just fine.

## Flashing the ESP32

Now it's time to flash the microcontroller with the Bullet GCSS. Click the second button on the Arduino IDE to compile the program and send it to the T-Call board.

![image](https://user-images.githubusercontent.com/17026744/104500218-e474bb00-55bc-11eb-9c02-dbd32b6fa263.png)

Wait until the Arduino IDE builds and sends the program. It'll confirm when the operation is complete.

![image](https://user-images.githubusercontent.com/17026744/104500531-52b97d80-55bd-11eb-939f-dde78b7307e5.png)

To check if it's working, open the Serial Monitor, it's the button on the top right corner.

![image](https://user-images.githubusercontent.com/17026744/104500704-83011c00-55bd-11eb-8e80-30c7b3243cf7.png)

Bullet GCSS will write some text messages on the Serial Monitor. If you can see this messages, then it's working fine.