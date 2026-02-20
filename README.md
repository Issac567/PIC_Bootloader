# PIC Multi-Project Repository

This repository contains several PIC microcontroller projects. Each MPLAB project targets a specific PIC family.  

A **PC with the B4J uploader** is required to update application firmware on the devices.  

- **MPLAB Projects:** Each folder contains source code and build files for a specific PIC family.  
- **B4JTools:** Utilities for uploading `.hex` files to the devices.  

---

## Usage

1. Build the desired MPLAB project to produce a `.hex` file.  
2. Use the B4J uploader to flash or update firmware on the target device.
3. The application codes can change the program codes in bootloader startup memory, therefore making application useless for entry. I will find solution for this, hopefully? Seperate projects can resolve this issue, but ISR and function calls to bootloader is inaccessible.

---

