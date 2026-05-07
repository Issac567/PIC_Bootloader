#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <SD.h>
#include <SPI.h>

#define SD_CS 21

// .map configuration file
struct ConfigMap {
    uint32_t intStartAddrFlash;
    uint32_t intEndAddrFlash;
    uint8_t  intEmptyFlashValue;
    uint16_t intInstructionPacket;
    uint16_t intPacketDelayMS;
    uint16_t intHandShakeDelayMS;
    uint8_t  intStopBit;
    uint32_t intExpectedFirmwareBytes;
    bool     blnUseWriteBurst;
    bool     blnUseDoubleHexAddr;
    bool     blnUse4Padding;
    String   strPicName;
    String   strNotes;
};

// microchip status
struct PicStatus {
    volatile int  cntVerify = false;
    volatile bool blnHandShakeSuccess = false;
    volatile bool blnConfigOK = false;
    volatile bool blnWriteACK = false;
    volatile bool blnISRTimeOut = false;
    volatile bool blnTimeoutError = false;
    volatile bool blnStartFlashVerify = false;
    volatile bool blnUserCancel = false;
    volatile bool blnEndFlashErase = false;
    volatile bool blnEndFlashVerify = false;
};

// External references so other files can see them (main.cpp)
extern TFT_eSPI tft;
extern XPT2046_Touchscreen touch;
extern SPIClass touchSPI;


extern ConfigMap myConfig;          // (sdcard.cpp)
extern PicStatus myPicStatus;       // (flash.cpp)

#endif