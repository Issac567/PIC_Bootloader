# HC-05 Bluetooth Software (SPP)

This software is a **simple and reliable tool for communicating with HC-05 Bluetooth modules** using the **Serial Port Profile (SPP)**. It allows you to send and receive data wirelessly with your HC-05 module.  

> **Note:** This software connects via **Bluetooth SPP**, not a physical COM port. Ensure your HC-05 is paired with your computer or device before using the software.

---

## Features

- Scan and connect to HC-05 modules over **Bluetooth SPP**  
- Send and receive data wirelessly  
- Support for **AT command mode** to configure device settings  
- Easy-to-use interface for communication and control  

---

## Hardware Setup

1. Power your HC-05 module properly (usually 5V or 3.3V depending on the module).  
2. For **AT command mode**, connect the **Key pin → VCC** before powering on.  
3. Pair the HC-05 with your computer via Bluetooth. Once paired, it will expose a virtual SPP port.  

---

## How to Use

1. Open the **HC-05 Bluetooth Software**.  
2. Scan for nearby Bluetooth devices and select your **HC-05 module**.  
3. Connect via **SPP (Serial Port Profile)**.  
4. To send data, type into the software’s interface and press **Send**.  
5. To enter **AT command mode** for configuration:  
   - `AT` – Test connection  
   - `AT+UART=57600,0,0` – Set baud rate to **57600 bps**  
   - `AT+NAME=MyDevice` – Change device name  
6. Wait for responses. Communication is wireless; do not power off the module during AT commands or data exchange.  

---

## Notes

- Default AT command mode baud is **38400 bps**. If you change it (e.g., to 57600 bps), make sure the software matches.  
- The software communicates over **Bluetooth SPP**, so no physical COM wiring is needed for data transfer.  
- Ensure the HC-05 is properly paired with your computer before connecting.  

---

## License

MIT License – Free to use, modify, and distribute.