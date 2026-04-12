# PIC B4J Uploader (Serial COM - TTL USB)
This software is a **simple and reliable PIC microcontroller firmware uploader**. It allows you to flash firmware to your PIC devices using a **TTL-to-USB serial connection**.  

> **Note:** This software communicates via **serial COM ports**. It requires a properly connected **TTL-to-USB adapter** to the PIC target device.  

---

## 🔌 TTL USB to Microchip Diagram
![Wiring diagram](TTLtoPIC.jpg)

## Hardware Setup  
1. Connect your **TTL-to-USB adapter** to the PIC:  
   - **TX of USB → RX of PIC**  
   - **RX of USB → TX of PIC**  
   - **GND → GND**  
   - Power your PIC as required (usually 5V or 3.3V depending on your PIC).  
2. The PIC must have a serial bootloader firmware pre-installed for uploading to work.  

---

## How to Use  
1. Open the **B4J Bootloader Uploader** software.
2. Select **TTL USB Serial** Tab 
3. **Select the PIC device** you want to program.  
4. **Select the COM port** that corresponds to your TTL-to-USB adapter.  
5. Click **Load Firmware** to select the **firmware file** (.hex) you want to upload.  
6. Press **Flash** to start the programming process.  
7. Wait until the software reports **success**. Do not disconnect the device during flashing.  

---

## Notes  
- Make sure your **COM port baud rate** matches the software settings (default is usually **57600 bps**).  
- Ensure the **PIC is powered properly** before attempting to flash firmware.  
- This uploader works via **serial connection only**, using TTL-to-USB.  

