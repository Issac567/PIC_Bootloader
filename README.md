# PIC Multi-Project Repository

This repository contains several PIC microcontroller projects. Each MPLAB project targets a specific PIC family.  

A **PC with the B4J uploader** is required to update application firmware on the devices.  

- **MPLAB Projects:** Each folder contains source code and build files for a specific PIC family.  
- **B4JTools:** Utilities for uploading `.hex` files to the devices.  

---

## Usage

1. Build the desired MPLAB project to produce a `.hex` file.  
2. Use the B4J uploader to flash or update firmware on the target device.    

---

## Notes

- ISRs cannot be shared across projects; each project handles its own interrupts.  
- Ordinary functions ccnnit be shared. 
-Future project will combine both Bootloader and App project as one.  Therefore, sharing will work across both areas.
