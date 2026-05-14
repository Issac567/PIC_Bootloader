/*
 * File:   bootloader.c
 * Version: 4.04
 * Created on January 19, 2026, 2:50 PM
 * Family: 18F26Q24
 * USE 1.30.487
 */

// Note: Both Bootloader.c and application.c should be flashed together due to 0x0008 Goto 0xE000 is owned by application.c!
// B4J Uploader does not flash bootloader range

#include <xc.h>
#include <stdint.h>                         // for standard integer types 
#include <stdbool.h>                        // for bool, true, false
#include "bootloader.h"
#include "config.h"
#include "uart.h"

// Note: B4J Expected bytes = 0xFFFF - 0x00C00 = 0xF400 = 62,464 BYTES!

#define FLASH_START         0x00C00         // Recommended for Q24 (Page aligned)
#define FLASH_END           0x0FFFF         // 64KB Flash limit
#define FLASH_ERASE_BLOCK   128             // Q24 erases in 128-word (256-byte) pages
#define FLASH_WRITE_BLOCK   128             // Q24 writes in 128-word (256-byte) pages        

#define MSG_MS_DELAY        150              // (min 150 for BT latency) Standard pacing delay 

#define LED_PIN   LATBbits.LATB4            // Use LAT for Output / Bootloader Led Status 
#define LED_TRIS  TRISBbits.TRISB4          // Output PortB.4 pin

#define DEVICE_BLE 0x01
#define DEVICE_CLASSIC_BT 0x02
#define DEVICE_WIFI 0x03
#define DEVICE_TTLSERIAL 0x04

// HARD-RESERVE THE MEMORY (0x1500 - 0x15FF)
// The 'volatile' ensures the compiler doesn't optimize away reads/writes.
// Verified: @1500 = Bank 21 for 64KB Q24 chips
volatile uint16_t nvm_hardware_buffer[FLASH_WRITE_BLOCK] __section("nvm_ram_area");

uint16_t flash_packet[FLASH_WRITE_BLOCK];   // 128 words, 256 bytes total
FlashConfig_t flashSettings = {             // Initialize the struct with your default values
   .isVerify_Checksum = false,
   .WhichDevice = 0x01,
   .BLE_MTU_Size = 20,
   .BLE_MTU_Delay = 20,
   .WhichFlashRequest = 0x00
};  

//-------------------------------------------------------
// INTERNAL OSCILLATOR CLK CONFIG
//-------------------------------------------------------
void INTOSC_Init(void)
{
    // 1. Set Nominal Frequency to 64 MHz
    // Q24 OSCFRQ: 1000 = 64 MHz
    OSCFRQ = 0x08; 

    // 2. Select HFINTOSC as system clock, NDIV = 1
    // OSCCON1: NOSC = 110 (HFINTOSC), NDIV = 0000 (1:1)
    // This is identical to K42
    OSCCON1 = 0b01100000; 

    #ifndef __DEBUG
        // 3. Wait for the clock switch to complete
        // OSCCON2 tracks OSCCON1. When they match, the switch is done.
        while (OSCCON2 != OSCCON1); 

        // 4. Wait for HFINTOSC to be stable
        // CHANGE: On Q24, the bit is OSCSTATbits.HFOR (HFINTOSC Oscillator Ready)
        while (!OSCSTATbits.HFOR); 
    #endif
}


//-------------------------------------------------------
// READ AND WRITE PROGRAM CODE ROUTINE + READ EEPROM DATA
//-------------------------------------------------------
// READ FLASH DATA (1 Word)
uint16_t Flash_ReadInstruction(uint32_t address)
{
    NVMADRU = (uint8_t)(address >> 16);
    NVMADRH = (uint8_t)(address >> 8);
    NVMADRL = (uint8_t)(address);

    NVMCON1bits.CMD = 0x00; // Word Read Command
    NVMCON0bits.GO = 1;
    while (NVMCON0bits.GO);

    // Combine the hardware result registers
    return (uint16_t)(NVMDATL | ((uint16_t)NVMDATH << 8));
}

/**
 * @brief Writes a 128-word page to Program Flash Memory on Q24
 * @param address The 24-bit target Flash address (must be page-aligned)
 * @param data Pointer to the global 'Input' array containing 128 words
 * The "Buffer RAM" Method (Modern / Q24)
 */
void Flash_WriteBlock(uint32_t address, uint16_t *data) 
{
    // 1. FORCE ALIGNMENT: Ensure we start at the beginning of a 256-byte page
    address &= (uint32_t)(~0xFF);
    
    // 2. Set the destination address in NVMADR
    // The NVMADR must point to the start of the page you want to program
    NVMADRU = (uint8_t)((address >> 16) & 0xFF);
    NVMADRH = (uint8_t)((address >> 8) & 0xFF);
    NVMADRL = (uint8_t)(address & 0xFF);

    // 4. Load the 256-byte Hardware Buffer
    // We copy directly from your global 'Input' array to the Buffer RAM
    for (uint8_t i = 0; i < FLASH_WRITE_BLOCK; i++) 
    {
        nvm_hardware_buffer[i] = data[i];
    }

    // 5. Set Command to "Page Write" (0x05)
    // Note: Ensure any previous Erase (0x06) is complete before calling this
    NVMCON1bits.CMD = 0x05; 

    // 6. Critical Unlock Sequence
    // We save the interrupt state to ensure 5V stability during the burn
    INTCON0bits.GIE = 0; 
    
    NVMLOCK = 0x55;
    NVMLOCK = 0xAA;
    NVMCON0bits.GO = 1; // Start the internal Flash charge pump
    
    // 7. Wait for the hardware to finish (CPU stalls here)
    while(NVMCON0bits.GO);

    // 8. Restore Interrupts and Clear Command
    INTCON0bits.GIE = 1;
    NVMCON1bits.CMD = 0x00;
}


//-------------------------------------------------------
// VERIFY FLASH DATA
//-------------------------------------------------------
void Flash_Verify(void)
{
    uint32_t addr;
    uint8_t ble_counter = 0;
    uint8_t b;
    uint8_t totalChecksum = 0;
    
    UART_TxString("<StartFlashVerify>");
    __delay_ms(MSG_MS_DELAY);

    // Q24 Caution: FLASH_ERASE_BLOCK is 128 words (256 bytes)
    // addr increases by the byte-count (128 * 2)
    //for (addr = FLASH_START; addr < FLASH_END; addr += (FLASH_ERASE_BLOCK * 2))
    for (addr = FLASH_START; addr + (FLASH_ERASE_BLOCK * 2) - 1 <= FLASH_END; addr += FLASH_ERASE_BLOCK * 2)
    {
        // --- NEW CANCEL CHECK ---
        // Check if data is available in the hardware UART buffer
        if (PIR4bits.U1RXIF)
        {
            b = UART_Rx();
            if (b == 0xCA) // Read the byte to clear the flag
            {
                __delay_ms(MSG_MS_DELAY); 
                UART_TxString("<VerifyCancelled>");
                __delay_ms(MSG_MS_DELAY); 
                return; 
            }
        }
        // -------------------------
        
        // On Q24, this buffer needs to be 128 words long
        uint16_t packet[FLASH_WRITE_BLOCK];
        for (uint8_t i = 0; i < FLASH_WRITE_BLOCK; i++)
        {
            packet[i] = Flash_ReadInstruction(addr + (uint32_t)(i * 2));
            totalChecksum += (packet[i] & 0xFF);
            totalChecksum += (packet[i] >> 8);
        }
              
        // Byte for Byte comparison
        if (flashSettings.isVerify_Checksum == false)
        {
            for (uint8_t i = 0; i < FLASH_WRITE_BLOCK; i++)
            {
                UART_Tx(packet[i] & 0xFF);   
                UART_Tx(packet[i] >> 8); 

                // BLE can only send MTU Limits usually 20 bytes per session
                if (flashSettings.WhichDevice == DEVICE_BLE) 
                {
                    ble_counter += 2;

                    // 2. Every xx bytes, we must pause for the HM-10 radio
                    if (ble_counter >= flashSettings.BLE_MTU_Size) 
                    {
                        // Wait for the HM-10 to clear its internal UART-to-BLE buffer
                       // Loop until value of MTU_Delay has achieved.
                        uint16_t temp = flashSettings.BLE_MTU_Delay;
                        while (temp--)
                        {
                            __delay_ms(1);
                        }   
                        ble_counter = 0;
                    }
                }
            }
        }
        if (flashSettings.isVerify_Checksum == false)
        {
            __delay_ms(1);
        // No Delay for checksum result.
        }  
    }

    // Checksum comparison only
    if (flashSettings.isVerify_Checksum == true)
    {
        UART_Tx(totalChecksum);
        __delay_ms(MSG_MS_DELAY); 
    }
    
    __delay_ms(MSG_MS_DELAY);
    UART_TxString("<EndFlashVerify>");
    __delay_ms(MSG_MS_DELAY);
}


//-------------------------------------------------------
// ERASE FLASH PROGRAM CODE DATA
//-------------------------------------------------------
void Flash_EraseApplication(void)
{
    uint32_t addr;

    UART_TxString("<StartFlashErase>");
    __delay_ms(MSG_MS_DELAY);
    
    // Iterate through Flash memory 
    // Q24: FLASH_ERASE_BLOCK is 128 (words), so 256 bytes per step
    for (addr = FLASH_START; addr + (FLASH_ERASE_BLOCK * 2) - 1 <= FLASH_END; addr += FLASH_ERASE_BLOCK * 2)
    {
        // 1. Load the 24-bit target address into NVMADR registers
        NVMADRU = (uint8_t)((addr >> 16) & 0xFF);
        NVMADRH = (uint8_t)((addr >> 8) & 0xFF);
        NVMADRL = (uint8_t)(addr & 0xFF);

        // 2. Set the Command for Page Erase (0x06)
        NVMCON1bits.CMD = 0x06;            // Set the page erase command

        // 3. CRITICAL SECTION
        INTCON0bits.GIE = 0;     

        // 4. HARDWARE UNLOCK SEQUENCE (Same as K42, but to NVMLOCK)
        NVMLOCK = 0x55;
        NVMLOCK = 0xAA;

        // 5. EXECUTION
        // On Q24, we set the GO bit. The CPU will stall.
        NVMCON0bits.GO = 1;

        // Wait for hardware to clear the GO bit
        while(NVMCON0bits.GO);        
        
        INTCON0bits.GIE = 1;     

        // 6. POST-OPERATION CLEANUP
        // Clear command to NOP to prevent accidental execution
        NVMCON1bits.CMD = 0x00;            // Disable writes to memory
    }

    UART_TxString("<EndFlashErase>");
    __delay_ms(MSG_MS_DELAY);
}


//-------------------------------------------------------
// FIRWARE UPDATE ROUTINE
//-------------------------------------------------------
bool ReceivePacket(void)
{
    // Q24 Warning: FLASH_WRITE_BLOCK is now 128 (words), so temp needs to be 256 bytes.
    // Ensure your stack size is large enough or make this 'static'.
    uint8_t temp[FLASH_WRITE_BLOCK * 2];  
    uint16_t byteCount = 0;
    uint32_t timeoutCounter = 0;
    const uint32_t TIMEOUT_3SEC = 2400000;
    
    while (byteCount < (FLASH_WRITE_BLOCK * 2))
    {
        // --- DATA RETRIEVAL ---  
        if (PIR4bits.U1RXIF) 
        {
            temp[byteCount] = UART_Rx();  
            byteCount++;
            timeoutCounter = 0; // Reset 3s window on every byte
        }
        else
        {
            timeoutCounter++;
            
            if (timeoutCounter > TIMEOUT_3SEC)
            {
                UART_TxString("<ISR Timeout>");
                __delay_ms(MSG_MS_DELAY);
                return false;
            }
        }
    }

    for (uint16_t i = 0; i < FLASH_WRITE_BLOCK; i++)
    {
        flash_packet[i] = ((uint16_t)temp[i*2 + 1] << 8) | temp[i*2];
    }

    return true; 
}

// When ready will use this in future
void DoFirmwareUpdate(void)
{   
    UART_TxString("<StartFlashWrite>");
    __delay_ms(MSG_MS_DELAY);

    uint32_t flashAddr = FLASH_START;
    uint8_t timeoutCount = 0;

    while (1)
    {
        UART_TxString("<ACK>"); 
            
        // 1. DATA ACQUISITION
        // IMPORTANT: ReceivePacket must now collect 256 bytes (128 words) 
        // to fill the larger Q24 Page Buffer.
        if (ReceivePacket())  
        {
            timeoutCount = 0; 

            // 2. FLASH PROGRAMMING
            // Commits 128 words to the Q24 NVM Page.
            Flash_WriteBlock(flashAddr, flash_packet);

            // 3. ADDRESS POINTER ADVANCEMENT
            // On Q24: 128 words * 2 = 256 bytes.
            flashAddr += (uint32_t)(FLASH_WRITE_BLOCK * 2);  

            // 4. BOUNDARY CHECK
            if (flashAddr + (FLASH_WRITE_BLOCK * 2) - 1 > FLASH_END)
            {
                UART_TxString("<EndFlashWrite>");
                __delay_ms(MSG_MS_DELAY);

                Flash_Verify(); // This will also need to walk in 256-byte steps
                return;
            }
        }
        // 5. ERROR / TIMEOUT HANDLING
        else 
        {
            timeoutCount++;

            if (timeoutCount >= 3)
            {
                UART_TxString("<ErrorTimeout>");
                __delay_ms(MSG_MS_DELAY);
                return;
            }
        }
    }
}


void ReceiveConfig(void)
{
    // First Byte = 0x01 = BLE, 0x00 <> BLE
    // Second and Third Byte = MTU Size
    // Fourth Byte = 0x00 = Flash and Verify (Byte for Byte): 0x01 = Flash and Verify Checksum))
    // Fourth byte = In future! 0x02 = Verify (Byte for Byte only): 0x03 = Verify Checksum only))
    
    uint8_t temp[4];  
    uint16_t byteCount = 0;
    uint32_t timeoutCounter = 0;
    
    const uint32_t TIMEOUT_MAX = 2400000; 
    
   // ----- FLush -----
    uint8_t dummy;
    while (PIR4bits.U1RXIF) 
    {
        dummy = U1RXB;         // discard
    }
    
    // Wait for all 4 configuration bytes from HOST.
    while (byteCount < 4)
    {
        if(PIR4bits.U1RXIF)  
        {
            temp[byteCount] = UART_Rx();  
            byteCount++;
            timeoutCounter = 0; 
        }
        else
        {
            timeoutCounter++;
            
            // CRITICAL: Do not proceed to erase/write flash if config failed
            if (timeoutCounter > TIMEOUT_MAX)
            {
                UART_TxString("<ConfigTimeout>");
                __delay_ms(MSG_MS_DELAY);
                return; 
            }
        }
    }

    // --- PROCESS CONFIG BYTES ---
    // BYTE 0: 0x01 = BLE: 0x02 = BT CLassic: 0x03 = WIFI: 0x04 = TTL Serial
    flashSettings.WhichDevice = (temp[0]);

    // BYTE 1 & 2: Set the MTU Size and MTU_Delay
    flashSettings.BLE_MTU_Size = ((uint16_t)temp[1] << 8) | temp[2];
    // Linear mapping:
    // 20 bytes  -> 20 ms
    // 400 bytes -> 100 ms
    flashSettings.BLE_MTU_Delay = (uint16_t)((flashSettings.BLE_MTU_Size * 211UL) / 1000UL + 16);
    
    // BYTE 3: Which Flash Process?
    flashSettings.WhichFlashRequest = (temp[3]);
    
    // Is it Checksum or not!
    if (flashSettings.WhichFlashRequest == 0x01 || flashSettings.WhichFlashRequest == 0x03)
    {
        flashSettings.isVerify_Checksum = true;        
    }
    else
    {
        flashSettings.isVerify_Checksum = false;
    }

   // Configuration received acknowledge         
    __delay_ms(MSG_MS_DELAY);
    UART_TxString("<ConfigOK>");
    __delay_ms(MSG_MS_DELAY);
          
    // Which type of flash request?  Flash + Verify or Verify alone.
    if (flashSettings.WhichFlashRequest == 0x00 || flashSettings.WhichFlashRequest == 0x01)   
    {
        Flash_EraseApplication();  
        DoFirmwareUpdate();     
    } else {
        Flash_Verify();
    }
 
}

//-------------------------------------------------------
// WAIT HANDSHAKE
//-------------------------------------------------------
// 0x55 and 0XAA handshake expected from Host to start firmware update
void WaitHandshake(void) 
{
    uint8_t prev = 0;
    uint8_t curr = 0;
    uint32_t handshakeCounter = 0;
    const uint32_t TIMEOUT_3SEC = 2400000;
    
    while (handshakeCounter < TIMEOUT_3SEC)
    {
        // CHANGE: PIR3bits.U1RXIF moved to PIR6bits.U1RXIF on the Q24
        if (PIR4bits.U1RXIF) 
        {
            // Note: UART_Rx() must return U1RXB for the Q24
            curr = UART_Rx();     

            // Check for BLE, SSP, TTL USB and WIFI uses this!
            if (prev == 0x55 && curr == 0xAA)
            {         
                UART_TxString("<InitReceived>");
                __delay_ms(MSG_MS_DELAY);
                
                ReceiveConfig();
                
                return;        
            }              
            prev = curr;
            //handshakeCounter = 0; // Optional: reset timeout if we see traffic
        }
        else 
        {
            handshakeCounter++;
        }
    }
    
    UART_TxString("<HandShakeTimeout>");
    __delay_ms(MSG_MS_DELAY);
}


//-------------------------------------------------------
// MAIN ENTRY FOR BOOTLOADER
//-------------------------------------------------------
void main(void) {    
    ANSELA = 0x00;                  // Port A all digital
    ANSELB = 0x00;                  // Port B all digital
    ANSELC = 0x00;                  // Port C all digital

    LED_TRIS = 0;                   // LED Output
    LED_PIN  = 1;                   // LED On (bootloader led))
       
    INTOSC_Init();                  // Must set internal Oscillator
    UART_Init();                    // Init Hardware UART   
     
    WaitHandshake();                // wait for 0x55 0xAA (3s timeout then goto app))
    
    LED_PIN  = 0;                   // LED Off (bootloader led))
    
    asm("goto 0xC00");              // If bootloader is not init from PC, then continue to application
    
    
    // Good news is when bootloader goes to 0x0C00 and is invalid, causes pic to reset and main repeated over and over till handshake and flash success!
}
