#ifndef BLE_H
#define BLE_H

#include <Arduino.h>
#include <NimBLEDevice.h>   // This replaces ALL old BLE headers

// Functions to be called from main.cpp
void initBLESystem();
bool bleconnectToServer();
bool bleIsConnected();
void bleDisconnect();
void bleDoScan();
void handleConnection();
void handleBleScan();
void handleMessage(String msg, uint8_t* rawBytes, size_t len);

extern NimBLEAdvertisedDevice* myDevice;                    // display_logic.cpp needs access to info
extern NimBLERemoteCharacteristic* pRemoteCharacteristic;   // flash.cpp needs access to write
extern uint16_t intMTUSize;

#endif
