# PIC B4J Uploader (HC-05 SSP Bluetooth) - Coming Soon

This project provides software support for **HC-05 Bluetooth modules**, enabling easy communication with microcontrollers or PCs. The software will allow users to send and receive data over Bluetooth using the **Serial Port Profile (SPP)**.

> **Note:** The software is under development and will be available soon.  

---

## Features (Planned)

- Connect to HC-05 over Bluetooth SPP  
- Send and receive data via HC-05  
- Configure HC-05 module settings via **AT commands**  
- Set custom baud rates and device names  

---

## Hardware Setup

Connect your HC-05 Bluetooth module to the PIC microcontroller as follows:

- **TX of HC-05 → RX of PIC**  
- **RX of HC-05 → TX of PIC**  
- **GND → GND**  
- **VCC → 3.3V or 5V** (depending on your HC-05 module)

> ⚠️ Ensure voltage compatibility. Most HC-05 breakout boards accept **5V on VCC**, but logic levels are typically **3.3V**.

---

### Requirements

- Power your PIC microcontroller as required (**typically 5V or 3.3V** depending on the device).  
- Ensure a **common ground** between HC-05 and PIC.  
- The PIC must have a **serial bootloader firmware pre-installed** for uploading to work.  

---

### Notes

- TX/RX lines must be **crossed** (TX → RX, RX → TX).  
- HC-05 communicates using **UART (serial)** over Bluetooth SPP.  
- No USB-to-TTL adapter is required for normal operation — communication is **wireless via Bluetooth**.

---

## HC-05 Command Mode

Not supported with Bootloader Uploader at this time.  You need to set HC05 baud to 57600 with alternative method online or https://www.deshide.com/News-detail_DSDTechTools.html .  The **Command Mode** allows you to configure the HC-05 Bluetooth module, including changing:

- Baud rate  
- Password  
- Device name  
- Role (Master/Slave)  

### Enabling Command Mode

1. Connect the **`Key`** pin of HC-05 to **VCC**.  
2. Power on the module.  
3. By default, HC-05 enters **AT command mode** at **38400 bps**.  

---

### Common AT Commands

| Function | AT Command Example | Notes |
|----------|-----------------|-------|
| Check module | `AT` | Should respond `OK` |
| Change baud rate | `AT+UART=57600,0,0` | Sets baud rate to **57600 bps**, 1 stop bit, no parity |
| Change name | `AT+NAME=MyDevice` | Sets Bluetooth device name |
| Change password | `AT+PSWD=1234` | Sets pairing password |
| Set role | `AT+ROLE=0` | `0` = Slave, `1` = Master |

> ⚠️ **Important:** If you change the baud rate, your software must match it. Currently, this software **does not automatically change the baud rate**, so you need to ensure the HC-05 baud matches your program settings.  

---

### Setting Baud Rate to 57600

1. Enter **Command Mode** (Key pin high, default 38400 bps).  
2. Send the following command via a serial terminal (e.g., Arduino Serial Monitor, PuTTY, or B4J):  
