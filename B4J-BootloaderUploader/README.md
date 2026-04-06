# PIC B4J Uploader (Serial COM - TTL USB)  

This software is a **simple and reliable PIC microcontroller firmware uploader**. It allows you to flash firmware to your PIC devices using a **TTL-to-USB serial connection**.  

> **Note:** This software communicates via **serial COM ports**. It requires a properly connected **TTL-to-USB adapter** to the PIC target device.  

---

## Features  

- Select **PIC device type** from a list of supported microcontrollers  
- Choose the **COM port** connected to your PIC via TTL-to-USB  
- Load a **firmware file** (.hex) from your computer  
- Flash the firmware to the PIC microcontroller with a single click  
- Simple and intuitive user interface  

---

## Hardware Setup  

1. Connect your **TTL-to-USB adapter** to the PIC:  
   - **TX of USB → RX of PIC**  
   - **RX of USB → TX of PIC**  
   - **GND → GND**  
   - Power your PIC as required (usually 5V or 3.3V depending on your PIC).  
2. Make sure the PIC is flashed with bootloader firmware.  

---

## How to Use  

1. Open the **B4J Bootloader Uploader** software.  
2. **Select the PIC device** you want to program.  
3. **Select the COM port** that corresponds to your TTL-to-USB adapter.  
4. Click **firmware** to select the **firmware file** (.hex) you want to upload.  
5. Press **Flash** to start the programming process.  
6. Wait until the software reports **success**. Do not disconnect the device during flashing.  

---

## Notes  

- Make sure your **COM port baud rate** matches the software settings (default is usually **57600 bps**).  
- Ensure the **PIC is powered properly** before attempting to flash firmware.  
- This uploader works via **serial connection only**, using TTL-to-USB.  

---

## License  

MIT License – Free to use, modify, and distribute.
