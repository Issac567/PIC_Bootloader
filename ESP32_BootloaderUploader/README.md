# 📺 ESP32-S3 + TFT + Touch + SD + BLE Project

---

## 🛠 Hardware Configuration

* **MCU:** ESP32-S3
* **Display:** ILI9488 3.5" TFT
* **Touch:** XPT2046 (Resistive)
* **Storage:** MicroSD via SPI (Optional)
* **BLE Device:** HM-10 BLE Adaptor

---

## 🔌 Pin Mapping (ESP32-S3)

| Function   | Pin | Bus       |
| ---------- | --- | --------- |
| TFT_MOSI   | 13  | SPI Bus 1 |
| TFT_SCLK   | 14  | SPI Bus 1 |
| TFT_MISO   | -1  | Not Used  |
| TFT_CS     | 10  | SPI Bus 1 |
| TFT_DC     | 11  | SPI Bus 1 |
| TFT_RST     | 12  | SPI Bus 1 |
| TOUCH_CS   | 15  | SPI Bus 2 |
| TOUCH_SCLK | 5   | SPI Bus 2 |
| TOUCH_MISO | 6   | SPI Bus 2 |
| TOUCH_MOSI | 4   | SPI Bus 2 |
| TOUCH_IRQ  | 1   | Interrupt |
| SD_CS      | 21  | SPI Bus 2 |

![Image](DeviceImage.jpg)

## 🔌 Pin Mapping (HM-10)

| Function   | Pin |
| ---------- | --- |
| Power 3.3v   | VCC  |
| Ground  | GND  |

---

## 💻 Software Stack

* **Framework:** Arduino / PlatformIO
* **Graphics:** https://github.com/Bodmer/TFT_eSPI
* **Touch:** https://github.com/PaulStoffregen/XPT2046_Touchscreen
* **IDE:** VS Code + PlatformIO

---

## ⚙️ Installation & Build

1. Clone the repository
2. Open in VS Code with PlatformIO

---

## Note
* You can experiment with BLE by powering on and off the HM-10 and checking the status.  Also, removing and inserting the SD Card and checking with **FLASH** button.  In the future i will include a working flash uploader for microchip.  

