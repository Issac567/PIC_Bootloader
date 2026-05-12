
/* 
 * File:   bootloader.h
 * Revision history: 1.01
 * Device: Universal
 */

#ifndef BOOTLOADER_H            // This is a "Header Guard"
#define BOOTLOADER_H            // It prevents errors if you include this file twice

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    bool isVerify_Checksum;     // Checksum or Regular byte for byte comparison
    uint8_t WhichDevice;        // Which Connection Device Selected
    uint16_t BLE_MTU_Size;      // BLE MTU Size (B4J sends value from config)
    uint16_t BLE_MTU_Delay;     // Min delay for each packet sent (ms)
    uint8_t WhichFlashRequest;  // Type of flash request (0x00 = default)
} FlashConfig_t;

#endif
