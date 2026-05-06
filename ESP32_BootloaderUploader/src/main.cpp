/* * =================================================================
 * PROJECT: Demo Uploader
 * HARDWARE: ESP32 + XPT2046 Touch + TFT Display + SD Card(Optional)
 * CONFIGURATION: Dual SPI (Separate Buses)
 * TFT9488 Seperate Bus. Touch2046/SD Card Shared Bus.
 * =================================================================
* * --- TFT DISPLAY (Bus 1: FSPI) ---
 * MOSI/SDI:   13                       -> Master Out Slave In
 * SCLK:       14                       -> Serial Clock
 * MISO/SDO:   -1                       -> (Not used by ILI9488)
 * CS:         10                       -> Chip Select
 * DC:          2                       -> Data/Command
 * RST:        12                       -> Reset 
 * * --- TOUCH SENSOR OR/AND SD Card (Shared Bus 2: HSPI) ---
 * TOUCH_DIN:  4 (MOSI)                 -> Data In to XPT2046
 * TOUCH_CLK:  5 (SCLK)                 -> Clock from ESP32 to XPT2046
 * TOUCH_DO:   6 (MISO)                 -> Data Out from XPT2046
 * TOUCH_CS:   15                       -> Touch Chip Select
 * TOUCH_IRQ:   1                       -> Touch Interrupt (Active LOW)
 * SD_MOSI      4                       -> Data In to SD Card
 * SD_MISO      6                       -> Data Out from SD Card
 * SD_SCK       5                       -> Clock from ESP32 to SD Card
 * SD_CS:      21                       -> SD Card Chip Select
 * * --- POWER & GROUND ---
 * VCC:    3.3V or 5V (Check Screen Regulator)
 * GND:    Common Ground
 * =================================================================
 */

#include <Arduino.h>
#include "globals.h"
#include "display_logic.h"
#include "ble.h"
#include "sdcard.h"

// TFT and Touch Pins defined in Platformio.ini!

// Define the actual objects here (Allocation)
TFT_eSPI tft = TFT_eSPI();
XPT2046_Touchscreen touch(TOUCH_CS, TOUCH_IRQ);
SPIClass touchSPI(HSPI); 

void setup() 
{
    Serial.begin(115200);
    delay(2500); // Good delay for the S3 USB Serial to catch up. Can remove if debugging not needed!

    initDisplaySystem();

    initSDSystem();

    drawUI();
    
    Serial.println("Issac Engineer Go-Box Ready.");
}

void loop() 
{
  

    //-------------------------------------------------------------------------------------------------
    // Check for touch
    //-------------------------------------------------------------------------------------------------
    handleTouch(); 

    //-------------------------------------------------------------------------------------------------
    // Do connecting to server
    //-------------------------------------------------------------------------------------------------
    handleConnection();

    //-------------------------------------------------------------------------------------------------
    // Do BLE scanning and find the device
    //-------------------------------------------------------------------------------------------------
    handleBleScan();
 
    delay(10);
}

