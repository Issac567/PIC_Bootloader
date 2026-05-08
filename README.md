# PIC Microcontroller Projects & B4J Uploader & ESP32 S3 Uploader

This repository contains a collection of firmware projects for various **PIC microcontroller families**. It includes both the source code for the microcontrollers and the necessary PC-side utilities for firmware deployment.

---

## 📂 Repository Structure

* **`MPLAB_Projects/`**: Organized by PIC family. Each directory contains the complete source code, headers, and build files.
* **`B4J_BootloaderUploader/`**: Contains the **B4J uploader** utility used to flash compiled `.hex` files to the target hardware via a PC.
* **`ESP32_BootloaderUploader/`**: Contains the **ESP32 uploader** utility used to flash compiled `.bin` files to the target hardware via a PC. Required flash.bin and config.map files must be added to SD Card.  B4J can export these files for ESP32 use. Only BLE is supported at 20 MTU only.

## 🛠 Requirements

| Requirement | Purpose |
| :--- | :--- |
| **MPLAB X IDE** | Compiling and building the firmware projects. |
| **XC Compilers** | Required for the build process (XC8, XC16, or XC32). |
| **B4J Runtime** | Necessary to run the uploader utilities on your PC. |
| **ESP32** | Another option, to run the uploader utilities on your PC. |
| **Adaptor** | Required communcation from B4J to Microchip. |

---

## 🛠 Setup & Installation

* [MPLAB Ecosystem – Microchip (Recommend MPLAB X 6.05 for both Pickit 3/MBLAP Snap)](https://www.microchip.com/en-us/tools-resources/archives/mplab-ecosystem)
* [MPLAB Compiler – Microchip](https://www.microchip.com/en-us/tools-resources/develop/mplab-xc-compilers)
* [B4J – B4X](https://www.b4x.com/b4j.html)
* [Visual Studio Code with Platformio extension](https://code.visualstudio.com/download)

## 🔌 Adaptors Required (Choose any one)
`Aliexpress can be a hit or miss.  I had few defects with them.  Try Amazon by DSD Tech Bluetooth. I tested HC-05 and HM-10 ok.  They are better.`

* [Amazon - DSD Tech Bluetooth](https://www.amazon.com/stores/page/F410FA30-270A-401B-89BD-0241C6D26886?ingress=2&lp_context_asin=B06WGZB2N4&lp_context_query=dsd%20tech%20bluetooth&visitId=e3431c51-7aae-4b37-84ba-2ab95bdd8037&store_ref=bl_ast_dp_brandlogo_sto&ref_=ast_bln)
* [Aliexpress - USB-to-TTL serial adapter](https://www.aliexpress.us/w/wholesale-USB%2525252dto%2525252dTTL-serial-adapter.html?spm=a2g0o.productlist.search.0)
* [Aliexpress - HC-05 Bluetooth SSP adapter ](https://www.aliexpress.us/w/wholesale-hc05-bluetooth-module.html?osf=auto_suggest&spm=a2g0n.productlist.header.0)
* [Aliexpress - HM-10 Bluetooth BLE adapter ](https://www.aliexpress.us/w/wholesale-hM10-bluetooth-module.html?spm=a2g0o.productlist.auto_suggest.2.bf20493bPXGW1g)
* [Aliexpress - DT-06 WIFI TCP/IP adapter (Not Tested!)](https://www.aliexpress.us/w/wholesale-dt-06-wifi-module.html?spm=a2g0o.productlist.auto_suggest.2.459d36cbgTXSe1)

### ⚡ Flashing Speed Comparison
The firmware flashing duration varies significantly based on the hardware interface used. For the fastest experience, use a wired connection.

| Interface | Tech | MTU / Bandwidth | Speed Rank |
| :--- | :--- | :--- | :--- |
| **TTL USB** | Wired | No MTU Limit (High Baud) | 🚀 **Fastest** |
| **DT-06** | WiFi | High Throughput | 🟢 Fast |
| **HC-05** | BT Classic | Continuous Stream | 🟡 Moderate |
| **HM-10** | BLE 4.0 | **20-byte MTU Limit** | 🔴 **Very Slow** |
| **ESP32 S3 UI** | BLE 5.0 | **20-byte MTU Limit** | 🔴 **Very Slow** |

> **Note:** BLE flashing is throttled by the mandatory 20-byte packet limit, making it the least efficient method for large binary transfers.

---

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
7. **B4J Uploader** or **ESP32 Uploader** is what you will use to upload the **Application** Intex Hex file to your **16F88** eg.  File location `..\BootloaderApp16F88.X\dist\default\production\BootLoaderApp16F88.X.production.hex`.

---

## 🚀 Usage Instructions

### 1. Build the Project (MPLAB) `Required`
1. Open the specific project folder in **MPLAB X IDE**.
2. Select your target configuration.  Under properties make sure your programmer is selected.
4. Run **Clean and Build**. 
5. Make sure minimal bootloader is flashed before `B4J` or `ESP32` usage.  **Chip with download icon** to flash.

### 2. Update Application Firmware (B4J) `Option 1`
e.g. using **TTL Serial Com**.  Updating the application device firmware requires the B4J uploader tool, but bootloader must be flashed with MPLAB first:
1. Connect your target PIC device to your PC.
2. Open the **B4J Uploader** utility.
3. Select `Com Port` and click `Open Port`.
4. Select configuration type eg. `16F88` in the combobox
5. Click `Load Firmware` the `BootLoaderApp16F88.X.production.hex` file generated in the previous step.
6. Execute the `Flash` command to flash the device.  It will Erase, Flash and Verify in that order.
7. Goto `B4J_BootloaderUploader` directory for more information on various connection device.

### 3. Update Application Firmware (ESP32) `Option 2`
Updating the application device firmware requires the ESP32 uploader tool, but bootloader must be flashed with MPLAB first:
1.  Build circuit according to schematics.
2.  SD Card requires `flash.bin` and `config.map` exported by B4J Uploader.  In B4J, select **PIC Name** from list, click **Tools->Export for ESP32**.
3.  Copy files to SD Card.
4.  With HM-10 connected to your choice of PIC.
5.  Execute the `Flash` command to flash the device.  It will Erase, Flash and Verify in that order.
6. Goto `ESP32_BootloaderUploader` directory for more information.

---

## 📱 Supported Families

| Project | PIC Family | Target Architecture |
| :--- | :--- | :--- |
| `PIC16_Project` | PIC16 | 8-bit |
| `PIC18_Project` | PIC18 | 8-bit |
| `PIC24_Project` | PIC24 | 16-bit |

---

> bootloader.c and application.c share the same function name calls across all 8 PIC projects. Adding support for additional PIC devices requires only minor modifications to the C codes, config.h and .map configuration.

---

> **Note:** Ensure your hardware connections are secure and the correct COM port is selected in the B4J uploader before attempting to flash the device.

---

> **Note:** **(ESP32 Only)** When canceling a flash write, you must wait at least 10 seconds for the PIC to reach its internal timeout before proceeding. If you cancel during a flash verify, please be aware that there is currently no mechanism to stop the PIC from sending bytes to the host; you must wait until the green LED begins flashing to confirm the device is ready. A cancel command for the verification phase is planned for a future update, which will require a bootloader patch to implement.

---

