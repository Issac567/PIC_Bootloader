/*
 * File:   bootloader.c
 * Version: 3.10
 * Author: Issac
 * Created on January 19, 2026, 2:50 PM
 * Family: 18F26Q24
 * USE 1.30.487
 */

#include <xc.h>
#include <stdint.h>                         // for standard integer types 
#include <stdbool.h>                        // for bool, true, false
#include "config.h"
#include "uart.h"

// Note: B4J Expected bytes = 0xFFFF - 0x00800 = 0xF800 = 63,488 BYTES!

#define FLASH_START         0x00800         // Recommended for Q24 (Page aligned)
#define FLASH_END           0x0FFFF         // 64KB Flash limit
#define FLASH_ERASE_BLOCK   128             // Q24 erases in 128-word (256-byte) pages
#define FLASH_WRITE_BLOCK   128             // Q24 writes in 128-word (256-byte) pages        

#define MSG_MS_DELAY 150                      // (min 150 for BT latency) Standard pacing delay 

#define LED_PIN   LATBbits.LATB4            // Use LAT for Output / Bootloader Led Status 
#define LED_TRIS  TRISBbits.TRISB4          // Output PortB.4 pin

// HARD-RESERVE THE MEMORY (0x1500 - 0x15FF)
// The 'volatile' ensures the compiler doesn't optimize away reads/writes.
// The '__at()' forces the compiler to pin this buffer to your hardware address.
// Verified: Bank 21 for 64KB Q24 chips
volatile uint16_t nvm_hardware_buffer[FLASH_WRITE_BLOCK] __section("nvm_ram_area");

uint16_t flash_packet[FLASH_WRITE_BLOCK];   // 128 words, 256 bytes total

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
void Verify_Flash(void)
{
    uint32_t addr;

    UART_TxString("<StartFlashVerify>");
    __delay_ms(MSG_MS_DELAY);

    // Q24 Caution: FLASH_ERASE_BLOCK is 128 words (256 bytes)
    // addr increases by the byte-count (128 * 2)
    //for (addr = FLASH_START; addr < FLASH_END; addr += (FLASH_ERASE_BLOCK * 2))
    for (addr = FLASH_START; addr + (FLASH_ERASE_BLOCK * 2) - 1 <= FLASH_END; addr += FLASH_ERASE_BLOCK * 2)
    {
        // On Q24, this buffer needs to be 128 words long
        uint16_t packet[FLASH_WRITE_BLOCK];

        for (uint8_t i = 0; i < FLASH_WRITE_BLOCK; i++)
        {
            packet[i] = Flash_ReadInstruction(addr + (uint32_t)(i * 2));
        }

        for (uint8_t i = 0; i < FLASH_WRITE_BLOCK; i++)
        {
            UART_Tx(packet[i] & 0xFF);   
            UART_Tx(packet[i] >> 8);     
        }

        __delay_ms(1);  
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

                Verify_Flash(); // This will also need to walk in 256-byte steps
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
            
            if (prev == 0x55 && curr == 0xAA) 
            {                                               
                UART_TxString("<InitReceived>");
                __delay_ms(MSG_MS_DELAY);
                
                Flash_EraseApplication();  
                DoFirmwareUpdate();        
                
                return; 
            }
            
            prev = curr;
            
            // OPTIONAL: If you want the 3-second window to "refresh" 
            // every time a character is typed, uncomment the line below:
            // handshakeCounter = 0; 
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
    
    asm("goto 0x800");              // If bootloader is not init from PC, then continue to application
    
    
    // Good news is when bootloader goes to 0x0800 and is invalid, causes pic to reset and main repeated over and over till handshake and flash success!
}
