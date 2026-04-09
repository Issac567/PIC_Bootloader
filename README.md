# PIC Microcontroller Projects & B4J Uploader

This repository contains a collection of firmware projects for various **PIC microcontroller families**. It includes both the source code for the microcontrollers and the necessary PC-side utilities for firmware deployment.

---

## 📂 Repository Structure

* **`MPLAB Projects/`**: Organized by PIC family. Each directory contains the complete source code, headers, and build files.
* **`B4J-BootloaderUploader/`**: Contains the **B4J uploader** utility used to flash compiled `.hex` files to the target hardware via a PC.

## 🛠 Requirements

| Requirement | Purpose |
| :--- | :--- |
| **MPLAB X IDE** | Compiling and building the firmware projects. |
| **XC Compilers** | Required for the build process (XC8, XC16, or XC32). |
| **B4J Runtime** | Necessary to run the uploader utilities on your PC. |

---

## 🛠 Setup & Installation

* [MPLAB Ecosystem – Microchip (Recommend MPLAB X 6.05 for both Pickit 3/MBLAP Snap)](https://www.microchip.com/en-us/tools-resources/archives/mplab-ecosystem)
* [MPLAB Compiler – Microchip](https://www.microchip.com/en-us/tools-resources/develop/mplab-xc-compilers)
* [B4J – B4X](https://www.b4x.com/b4j.html)

# Hardware Required (Minimum one of these)
* USB-to-TTL serial adapter [Aliexpress](https://www.aliexpress.us/w/wholesale-USB%2525252dto%2525252dTTL-serial-adapter.html?spm=a2g0o.productlist.search.0)
* HC-05 Bluetooth SSP adapter [Aliexpress](https://www.aliexpress.us/w/wholesale-hc05-bluetooth-module.html?osf=auto_suggest&spm=a2g0n.productlist.header.0)

---

## 🛠 Setup & Installation

Follow these steps to correctly link the Application and Bootloader projects:

1. **Download the Repository**: Click the green **Code** button and select **Download ZIP**, then extract it to your local machine.
2. **Open MPLAB X**: Launch the IDE and open the **Bootloader** project (e.g., `BootLoader16F88.X`).
3. **Import Loadables**:
   * In the Projects pane, locate your **BootLoader16F88** project.
   * Right-click on the **Loadables** folder.
   * Select **Add Loadable Project...**
   * Select the corresponding project folder that has the **BootLoaderApp16F88.X** suffix.
   * Right click **BootLoader16F88** folder and **Set as Main Project**
4. Make sure to click **Resolve** packs if neccessary under properties or select a version that works.  Unresolved will not compile successfully. 
5. Click **Production** --> **Clean and Build Project**: This will automatically compile the bootloader/application and also merge them into a single `.hex` file.  **Bootloader folder** have `BootLoader16F88.X.production.hex`, **Application folder** have `BootLoaderApp16F88.X.production.hex` and **Bootloader folder** also have both merged into `BootLoader16F88.X.production.unified.hex`.
6. Flash **(Chip with download icon)**  the bootloader and application with MBLAB.  Which allows bootloader to flash, erase and verify in B4J.  We need the bootloader minimal.  If set as loadables, it will flash both bootloader and application.
7. **B4J Uploader** is what you use to upload the **Application** Intex Hex file to your **16F88** eg.  File location `..\BootloaderApp16F88.X\dist\default\production\BootLoaderApp16F88.X.production.hex`.
> **Note:** B4J Uploader- Serial COM TTL USB is available in Repository. SSP Bluetooth HC05 is also available. 

---

## 🚀 Usage Instructions

### 1. Build the Project (MPLAB)
1. Open the specific project folder in **MPLAB X IDE**.
2. Select your target configuration.  Under properties make sure your programmer is selected.
4. Run **Clean and Build**. 
5. Make sure minimal bootloader is flashed before B4J usage.  **Chip with download icon** to flash.

### 2. Update Firmware (B4J)
Updating the application device firmware requires the B4J uploader tool, but bootloader must be flashed with MPLAB first:
1. Connect your target PIC device to your PC.
2. Open the **B4J Uploader** utility.
3. Select `Com Port` and click `Open Port`.
4. Select configuration type eg. `16F88` in the combobox
5. Click `Load Firmware` the `BootLoaderApp16F88.X.production.hex` file generated in the previous step.
6. Execute the `Flash` command to flash the device.  It will Erase, Flash and Verify in that order.

---

## 📱 Supported Families

| Project | PIC Family | Target Architecture |
| :--- | :--- | :--- |
| `PIC16_Project` | PIC16 | 8-bit |
| `PIC18_Project` | PIC18 | 8-bit |
| `PIC24_Project` | PIC24 | 16-bit |

---

bootloader.c and application.c share the same function name calls across all 8 PIC projects. Adding support for additional PIC devices requires only minor modifications to the C codes, config.h and .map configuration.

---

> **Note:** Ensure your hardware connections are secure and the correct COM port is selected in the B4J uploader before attempting to flash the device.

---

