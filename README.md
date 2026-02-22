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
3. Load the `.hex` file generated in the previous step.
4. Execute the update command to flash the device.

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

