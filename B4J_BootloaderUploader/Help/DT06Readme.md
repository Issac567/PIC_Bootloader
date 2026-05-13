# DT-06 WIFI (Custom Rom)
DT06 requires a custom flash image and will not work correctly with the original ROM firmware.  Make **backups** of your original rom before flashing custom rom! Your original rom will be erased!


## Requirements
- Visual Studio Code
- PlatformIO extension installed in Visual Studio Code
- USB TTL adapter connected to DT06 RX/TX lines


## Flashing Instructions
1. Install Visual Studio Code  
2. Install the PlatformIO extension  
3. Open the attached  [DT06 Flashing folder](https://github.com/ElectronicFan/PlatformIO_Projects/tree/main/DT06%20Flashing) in Visual Studio Code 
4. Connect the DT06 module to a USB TTL adapter:
   - TX -> RX
   - RX -> TX
   - GND -> GND
   - 3.3V power
5. Put the DT06 into flashing mode by **powering off**, **push and hold G button** (not R), **power on** DT06 while holding button for **2 seconds** and release.
6. Click the PlatformIO Upload button (right arrow icon in bottom status bar)


## PlatformIO Board Configuration
Recommended PlatformIO board:
- board = esp8285

## Notes
- Generic ESP8266 AT firmware does not properly work with DT06 modules
- DT06 uses a custom flash layout and firmware structure
- Full flash images are recommended for recovery

## B4J Support
DT-06 in B4J Uploader is deprecated but still supports DT06 using the attached custom flash firmware.

## Performance Notes
- B4J to WIFI communication is optimized and can send bytes without delay
- PIC to WIFI communication may require a small delay in PIC firmware
- PIC delays have not yet been implemented because the current PIC firmware is shared with multiple devices

Current supported devices:
- HM10
- HC05
- USB TTL adapters

These devices currently operate flawlessly without delays from PIC.

---

## DT-06 WIFI TCP/IP
This project provides software support for **DT-06 WIFI modules**, enabling easy communication with microcontrollers or PCs. The software will allow users to send and receive data over TCP/IP.

> **Note:** This software communicates via WIFI to serial COM ports to PIC. It requires a properly connected **DT-06 WIFI adapter** to the PIC target device.
  
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
- `DT-06` communicates using **UART (serial)** over WIFI.  
- No USB-to-TTL adapter is required for normal operation — communication is **wireless via WIFI**.


## How to Use  
1. Windows WIFI Setting, Connect to **GoBox_Bridge** with password **12345678**");
2. Open the **B4J Bootloader Uploader** software.
3. Set Host IP to **192.168.4.1**
4. Set Port to **8080**  
5. **Click Connect** and wait for success connection.
6. **Select the PIC device** you want to program.
7. Click **Load Firmware** to select the **firmware file** (.hex) you want to upload.  
8. Press **Flash** to start the programming process.  
9. Wait until the software reports **success**. Do not disconnect the device during flashing. 


## Notes  
- Make sure your `DT-06` **COM port baud rate** matches the software settings (default is usually **57600 bps**).  
- Ensure the **PIC is powered properly** before attempting to flash firmware.  
