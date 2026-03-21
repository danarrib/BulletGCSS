The modem board must be connected to the Flight Controller on the aircraft in order to Bullet GCSS work.

## Choosing a Free UART on the Flight Controller

Flight Controller must have a free UART to receive the modem module. This UART must be set to "MSP" on the Ports tab inside INAV configurator.

![image](https://user-images.githubusercontent.com/17026744/104481703-da46c280-55a4-11eb-88eb-b08b08228c7e.png)

On this example, the modem module will be wired to the UART1 (TX1 and RX1 pins on the Flight Controller).

## Wiring the modem module to the Flight Controller

Now you have to connect wires (by soldering or using pin headers) between the Flight Controller and the Modem board.

### TTGO T-Call Board (SIM800L)
![MatekF405-Wing-Wiring-Diagram](https://user-images.githubusercontent.com/17026744/104481981-27c32f80-55a5-11eb-909c-f26be8403843.jpg)

### TTGO T-PCIE Board (SIM7600)
![MatekF405-Wing-Wiring-Diagram2](https://user-images.githubusercontent.com/17026744/109437905-3f942d00-7a06-11eb-963f-795ed496e8ef.jpg)

On this examples, modem boards are wired to a Matek F405-Wing Flight Controller using UART1.

- Pin 19 from module connects to the TX1 pin on the Flight controller
- Pin 18 from module connects to the RX1 pin on the FC
- GND pin from module connects to any GND pin on the FC
- 5V pin from module connects to a 5V pin on the FC

If you choose another UART instead the UART1, you need to change the pins to the corresponding number of the UART...
- If you choose UART3, then the pins are RX3 and TX3
- If you choose UART4, then pins are RX4 and TX4
- And so on...

## Considerations about power

Both TTGO T-Call and TTGO T-PCIE modules draws a considerable amount of power when transmitting thru the cellular network. It can draw up to 2 amps during the transmission. **It WILL NOT WORK if powered only by the USB connection**. So if it's not working on your bench tests, be aware of that.

WiFi works fine by USB power, so it's ok for bench tests. Only the cellular modem requires more power.

That's why it's important to connect a good power source for it. Matek F405-Wing has a beefy 5V power regulator, so it will power the module without any problems as long as there isn't any other power-hungry device connected to this same 5V line.

Other flight controllers may not be able to power the module. In this case, you have to add a dedicated power regulator for the module. It must be able to provide 5V and 2 Amps reliably.

![MatekF405-Std-Wiring-Diagram](https://user-images.githubusercontent.com/17026744/104489130-0c105700-55ae-11eb-9225-0a207110eb04.jpg)

Another solution is to use another battery just for the module. It has a 1S battery connection on the back side. I don't like this solution too much because it adds another battery to manage. But if you prefer go this way, just don't wire the 5V connection (but keep the GND!!!) and you'll be fine. 

![Battery-power-TTGO-T-Call](https://user-images.githubusercontent.com/17026744/104490830-20555380-55b0-11eb-9d97-3cc9a9bb45a0.jpg)

Both TTGO boards has a power management circuit! So you can charge its battery using the USB-C connector. It also provides over-charge and over-discharge protection for the battery. 

But be advised: **Bullet GCSS DOES NOT HANDLE ANY BATTERY ASPECT**. There's no code that interacts with the IP5306 or AXP192 power management chips on Bullet GCSS. It may be added on the future, depending on user requests. But for now, the best way to power the board is by an external 5V voltage regulator.

