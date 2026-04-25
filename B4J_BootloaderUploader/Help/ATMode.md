# HC-05/HM-10 AT Command Mode
> Bootloader Uploader v8.04 now supports AT Command Mode. `DT-06` supports this mode, but use web browser option for simplicity. 

You need to set `HC05/HM10` baud to `57600` with Bootloader Uploader, alternative method online or https://www.deshide.com/News-detail_DSDTechTools.html .  The **Command Mode** allows you to configure the `HC-05` or HM-10 Bluetooth module, including changing:

- Baud rate  
- Password  
- Device name  
- Role (Master/Slave)  


## 🔌 HC-05/HM-10 to TTL USB Diagram
![Wiring diagram](BTtoTTL.png)


### Hardware Setup
Connect your `HC-05/HM-10` Bluetooth module to the **TTL USB** as follows:

- **TX of HC-05/HM-10 → RX of TTL USB**  
- **RX of HC-05/HM-10 → TX of TTL USB**  
- **GND → GND**  
- **VCC → 3.3V or 5V** (depending on your HC-05/HM-10 module)
- **EN** → VCC

  
### Enabling AT Command Mode
`2 Ways to connect`

`OPTION 1`
1. Connect the **EN** pin of `HC-05` to **VCC** (Mine was 3.3v).
2. Power on the module.
   
`OPTION 2`
1. **EN** pin not needed.  Hold Button Down.
2. Power on the module and release button after 3 seconds.

`OPTION 3 HM-10`
1. **EN** pin not needed or available. 
2. Power on the module it will enter AT Mode if BLE is not connected.

`Continue from OPTION 1, 2 or 3`
1. Should blink very slow.
2. By default, `HC-05` enters **AT command mode** at **38400 bps** and `HM-10` enters at **9600**.
3. Use the Bootloader Uploader or software https://www.deshide.com/News-detail_DSDTechTools.html
4. Change the baud rate to `57600` using the software (Note: AT command mode always uses 38400 `(HC-05)` and 9600 `(HM-10)` and is not affected).


### Common AT Commands
| Function | AT Command Example | Notes |
|----------|-----------------|-------|
| Check module | `AT` | Should respond `OK` |
| Change baud rate | `AT+UART=57600,0,0` | Sets baud rate to **57600 bps**, 1 stop bit, no parity |
| Change name | `AT+NAME=MyDevice` | Sets Bluetooth device name |
| Change password | `AT+PSWD=1234` | Sets pairing password |
| Set role | `AT+ROLE=0` | `0` = Slave, `1` = Master |

> ⚠️ **Important:** If you change the baud rate, your firmware must match it. So you need to ensure the HC-05/HM-10 baud matches your program settings.
