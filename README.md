# PIC Microcontroller Projects & B4J Uploader

This repository contains a collection of firmware projects for various **PIC microcontroller families**. It includes both the source code for the microcontrollers and the necessary PC-side utilities for firmware deployment.

---

## 📂 Repository Structure

* **`MPLAB Projects/`**: Organized by PIC family. Each directory contains the complete source code, headers, and build files.
* **`B4JTools/`**: Contains the **B4J uploader** utility used to flash compiled `.hex` files to the target hardware via a PC.

## 🛠 Requirements

| Requirement | Purpose |
| :--- | :--- |
| **MPLAB X IDE** | Compiling and building the firmware projects. |
| **XC Compilers** | Required for the build process (XC8, XC16, or XC32). |
| **B4J Runtime** | Necessary to run the uploader utilities on your PC. |

---

## 🛠 Setup & Installation

Follow these steps to correctly link the Application and Bootloader projects:

1. **Download the Repository**: Click the green **Code** button and select **Download ZIP**, then extract it to your local machine.
2. **Open MPLAB X**: Launch the IDE and open both the **Bootloader** project and its corresponding **App** project (e.g., `BootLoader16F88.X` and `BootLoaderApp16F88.X`).
3. **Import Loadables**:
   * In the Projects pane, locate your **BootLoader16F88.X** project.
   * Right-click on the **Loadables** folder.
   * Select **Add Loadable Project...**
   * Select the corresponding project folder that has the **BootLoaderApp16F88.X** suffix.
   * Right click **BootLoader16F88** folder and **Set as Main Project**
4. Click **Production** --> **Clean and Build Project**: Do not use **Build Project**, this only builds bootloader Intel Hex file. This will automatically compile the bootloader/application and also merge them into a single `.hex` file.  **Bootloader** has `BootLoader16F88.X.production.hex`, **Application** has `BootLoaderApp16F88.X.production.hex` and **Bootloader** also has both merged into `BootLoader16F88.X.production.unified.hex`.
5. **B4J Uploader** is what you use to upload the Intex Hex file to your **16F88** eg.  File location `..\BootloaderApp16F88.X\dist\default\production\BootLoaderApp16F88.X.production.hex`

---

## 🚀 Usage Instructions

### 1. Build the Project
1. Open the specific project folder in **MPLAB X IDE**.
2. Select your target configuration.
3. Run **Clean and Build**. 
4. Locate the output `.hex` file in the `dist/` directory of the project.

### 2. Update Firmware
Updating the device firmware requires the B4J uploader tool:
1. Connect your target PIC device to your PC.
2. Open the **B4J Uploader** utility.
3. Select `com port` and click `Open Port`
4. Select configuration type eg. `16F88` in the combobox
5. Click `Load Firmware` the `BootLoaderApp16F88.X.production.hex` file generated in the previous step.
6. Execute the `flash` command to flash the device.  It will Erase, Flash and Verify in that order

---

## 📱 Supported Families

| Folder Name | PIC Family | Target Architecture |
| :--- | :--- | :--- |
| `PIC16_Project` | PIC16 | 8-bit |
| `PIC18_Project` | PIC18 | 8-bit |
| `PIC24_Project` coming soon | PIC24 | 16-bit |

---

> **Note:** Ensure your hardware connections are secure and the correct COM port is selected in the B4J uploader before attempting to flash the device.

---

