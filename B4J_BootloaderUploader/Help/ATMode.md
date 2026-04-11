# $\color{blue}{\text{HC-05/HC-08 AT Command Mode (Required by 3rd party Software)}}$

**Not supported** with Bootloader Uploader at this time.  You need to set HC05/HC08 baud to 57600 with alternative method online or https://www.deshide.com/News-detail_DSDTechTools.html .  The **Command Mode** allows you to configure the HC-05 or HC-08 Bluetooth module, including changing:

- Baud rate  
- Password  
- Device name  
- Role (Master/Slave)  

### Enabling AT Command Mode

`2 Ways to connect`

`OPTION 1`
1. Connect the **EN** pin of HC-05/08 to **VCC** (Mine was 3.3v).
2. Power on the module.
   
`OPTION 2`
1. **EN** pin not needed.  Hold Button Down.
2. Power on the module and release button after 3 seconds.

`Continue from Option`
1. Should blink very slow.
2. By default, HC-05 enters **AT command mode** at **38400 bps** and HC-08 enters at **9600**.
3. Use the software https://www.deshide.com/News-detail_DSDTechTools.html
4. Change the baud rate to 57600 using the software (Note: AT command mode always uses 38400 (HC-05) and 9600 (HC-08) and is not affected).

---

### Common AT Commands

| Function | AT Command Example | Notes |
|----------|-----------------|-------|
| Check module | `AT` | Should respond `OK` |
| Change baud rate | `AT+UART=57600,0,0` | Sets baud rate to **57600 bps**, 1 stop bit, no parity |
| Change name | `AT+NAME=MyDevice` | Sets Bluetooth device name |
| Change password | `AT+PSWD=1234` | Sets pairing password |
| Set role | `AT+ROLE=0` | `0` = Slave, `1` = Master |

> ⚠️ **Important:** If you change the baud rate, your firmware must match it. Currently, B4J Uploader **does not automatically change the baud rate**, so you need to ensure the HC-05/HC-08 baud matches your program settings.
