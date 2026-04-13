# DT-06 WIFI TCP/IP
This project provides software support for **DT-06 WIFI modules**, enabling easy communication with microcontrollers or PCs. The software will allow users to send and receive data over TCP/IP.


> **Note:** This software communicates via **serial COM ports**. It requires a properly connected **DT-06 WIFI adapter** to the PIC target device.

- make sure to change baud to 57600 by using web browser at `192.168.4.1` and use port `9000`.
- connect to WIFI in OS settings first, then use web browser.

## 🔌 DT-06 to Microchip Diagram
![Wiring diagram](WIFItoPIC.jpg)


## Hardware Setup  
Connect your `DT-06` WIFI module to the PIC microcontroller as follows:

- **TX of DT-06 → RX of PIC**  
- **RX of DT-06 → TX of PIC**  
- **GND → GND**  
- **VCC → 3.0V to 3.6** (depending on your DT-06 module)


### Requirements
- Power your PIC microcontroller as required (**typically 5V or 3.3V** depending on the device).  
- Ensure a **common ground** between `DT-06` and PIC.  
- The PIC must have a **serial bootloader firmware pre-installed** for uploading to work.

  
### Notes
- TX/RX lines must be **crossed** (TX → RX, RX → TX).  
- DT-06 communicates using **UART (serial)** over WIFI.  
- No USB-to-TTL adapter is required for normal operation — communication is **wireless via WIFI**.


## How to Use  
1. Open the **B4J Bootloader Uploader** software.
2. Set Host IP to **192.168.4.1**
3. Set Port to **9000**  
4. **Click Connect** and wait for success connection.
5. **Select the PIC device** you want to program.
6. Click **Load Firmware** to select the **firmware file** (.hex) you want to upload.  
7. Press **Flash** to start the programming process.  
8. Wait until the software reports **success**. Do not disconnect the device during flashing. 


## Notes  
- Make sure your `DT-06` **COM port baud rate** matches the software settings (default is usually **57600 bps**).  
- Ensure the **PIC is powered properly** before attempting to flash firmware.  
