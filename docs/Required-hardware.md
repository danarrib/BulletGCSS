In order to use Bullet GCSS, you'll need:
* An aircraft with an INAV-compatible flight controller board
* A TTGO T-Call Board, OR a TTGO T-PCIE Board with a SIM7600 module
* A SIM-Card with a data plan for the modem
* A SmartPhone or a computer with Internet connection

## Aircraft

![image](https://user-images.githubusercontent.com/17026744/104070490-c1759000-51e5-11eb-901e-03b07a787455.png)

Bullet GCSS was designed to work with [INAV](https://github.com/iNavFlight/inav). INAV is a Free and Open Source Flight Controller system what works remarkably well on multirotors and fixed wing aircrafts. 

Bullet GCSS do not provide any flying capabilities to the aircraft, this is the role of the Flight Controller. Bullet GCSS, however, keeps an active communication channel to the Flight Controller, so they can exchange telemetry data and commands.

The flight controller must have a free UART so you can wire the modem to it.

If you don't have a Flight Controller, or the flight controller you have doesn't has spare UARTs, I can recommend you to use one of the following ones:

* [Matek F405-Wing](https://www.banggood.com/custlink/mGKhPAPapj) - Nice flight controller for fixed-wing aircraft. 5 UARTs, Beefy BEC for servos and electronics, OSD and really good price.
* [Matek F765-Wing](https://www.banggood.com/custlink/mKGYIe3k51) - Ideal for fixed-wing aircraft. Lots of processing power, 7 UARTs, 12 PWM outputs, Beefy BEC for servos and electronics and OSD. A bit expensive tho.
* [Matek F405-SE](https://www.banggood.com/custlink/m3KdI6DzZr) - Nice All-in-one flight controller for multirotor aircraft if you're using standalone ESCs. 6 UARTs, 9 PWM outputs. It'll require you to use a separate BEC for the modem device.
* [Matek F405-STD](https://www.banggood.com/custlink/vDKYSaEkR3) - Works for both multirotor and fixed wing aircraft and requires an external 5V power supply. But it's super cheap and reliable. 5 UARTs, 7 PWM outputs, OSD. If using it on a Fixed wing aircraft, you can pair with a [Matek FCHUB-W PDB](https://www.banggood.com/custlink/vGm3D7af8A), and it'll have plenty of power for servos, modem and electronics.
* [Matek F722-MiniSE](https://www.banggood.com/custlink/vK3YSBvUSQ) - Ideal for multirotor aircraft with 4-in-1 ESCs. 5 UARTs, 8 PWM outputs, OSD and plenty of processing power. You'll need a dedicated 5V regulator for the modem device tho.

## Modem board

There are Two Modem boards supported by Bullet GCSS, both boards are based on the ESP32 microcontroller, the difference between them is the cellular modem.

The ESP32 is a 32-bit dual-core microcontroller that works at 240Mhz and has 4MB of Flash memory and 8MB of RAM. It also has onboard WiFi and Bluetooth LE.

### TTGO T-Call Board
![image](https://user-images.githubusercontent.com/17026744/104070497-c63a4400-51e5-11eb-8bb7-102ff29219f4.png)

It's an inexpensive prototyping device that contains an ESP32 MCU (Microcontroller Unit), a SIM800L GPRS Modem, and all the necessary electronics and connections to make this two main components work together.

SIM800L is a Quad-band GPRS Modem, capable of connecting to a cellular network at 850, 900, 1800 or 1900 Mhz frequencies. It's a 2G modem, so be aware of that. Most countries in the world still supports 2G cellular communications, but it's fading away, so check if there's 2G coverage where you're planning to use it.

[You can buy a TTGO T-Call board using this link](https://www.banggood.com/custlink/DDvySdDajK).

### TTGO T-PCIE Board + SIM7600 module

![TTGO T-PCIE SIM7600](https://user-images.githubusercontent.com/17026744/109437479-b976e700-7a03-11eb-8e28-f91e23004b20.png)

It's a slightly more expensive prototyping device that contains an ESP32 MCU, and a PCIE slot that can receive a modem module. It works over both 2G and 4G networks, and is a bit bigger and heavier than the previous board.

You have to buy both the TTGO T-PCIE and the SIM7600 module separately. They usually are NOT sold in bundles.

Bullet GCSS is compatible with SIM7600-series modules ONLY.

You have to pay attention on the module that you'll buy, because they are different depending on the region that you plan to use:
* **SIM7600SA** for South America, New Zealand and Australia
* **SIM7600A** for North America
* **SIM7600E** for Europe, Middle-east, Africa, Korea and Thailand
* **SIM7600JC** for Japan

[You can buy a TTGO T-PCIE board and it's module using this link](https://www.banggood.com/custlink/K3vdZaFKHl).

> Note: This modules comes with the proper cellphone network antenna and a GNSS (GPS) antenna too. The GPS antenna is NOT used by Bullet GCSS for now, so you don't need to use it. But keep it in a safe place because Bullet GCSS can start using it someday.

## SmartPhone or Computer

The best way to use the Bullet GCSS is on the SmartPhone. It's convenient to have a small screen near the control radio, so the pilot can easily see the telemetry data from the aircraft. SmartPhone also provides it's own location to the Bullet GCSS. A computer is bucky and a lot more difficult to deal piece of hardware, specially at a remote location without grid power and internet connection.

If you need to use the computer anyway, there's no problem. Bullet GCSS will work just as fine.

Any Android SmartPhone or iPhone should work just fine with Bullet GCSS. The user interface is built in pure HTML5, and it opens inside the Web Browser just as any other regular website. It may be a bit CPU-heavy since it renders a map and a EFIS on the Browser, but it should not be a problem to any modern device (from the last ~4 years).

![iphone-horizontal](https://user-images.githubusercontent.com/17026744/109589772-9e7ea280-7ae9-11eb-8709-5b2be365015f.png)
