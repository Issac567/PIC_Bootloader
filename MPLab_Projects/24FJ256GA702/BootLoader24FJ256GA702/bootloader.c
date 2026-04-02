/*
 * File:   bootloader.c
 * Version: 3.06
 * Author: Issac
 * Family: 24F256GA702
 * Created on January 19, 2026, 2:50 PM
 */


/**
 * BOOTLOADER MEMORY CONFIGURATION:
 * ----------------------------------------------------------------------------
 * 1. This modified .gld MUST be included in the 'Linker Files' project tab.
 * 2. This defines the dedicated footprint for the Bootloader firmware.
 * * [BOOTLOADER RANGE]
 * Start Address (ORIGIN): 0x200  (Offset for Vector Tables)
 * End Address:           0x7FF
 * Length:                0x600 
 * * COORDINATION: The Application range begins at 0x800.
 * ----------------------------------------------------------------------------
 */

#include <xc.h>
#include <stdint.h>                         // for standard integer types 
#include <stdbool.h>                        // for bool, true, false
#include "config.h"
#include "uart.h"

// Note: B4J Expected bytes = 0x2A7FE - 0x0800 = 344,064 BYTES with Phantom!

// Adjusted for PIC24FJ64GA102 based on your .gld configuration
#define FLASH_START          0x00800            // Matches your Application ORIGIN
#define FLASH_END            0x2A7FE            // Matches your Application END (Last address) 0x2AEFE
// IMPORTANT: Do not set FLASH_END above 0x2A800. 
// The PIC24FJ256GA702 erases in 1024-instruction "Pages" (2048 or 0x800 addresses).  Not using Row erase 256 or 0x100 addresses)
// If the erase loop hits 0x2A800, it wipes the entire block up to 0x2AFFF.
// Since Configuration Words live at 0x2AF00+, a Page Erase at 0x2A800 will 
// brick the device by clearing FOSC, FWDT, and FSEC.

// PIC24FJ256GA702 Specific Flash geometry
#define FLASH_ERASE_BLOCK    1024                // Erase page size is 1024 instructions (standard for GA702)
#define FLASH_WRITE_BLOCK    128                 // Write row size is 128 instructions (standard for GA702)

#define TIMER2_COUNT        186                 // 3s 
#define MSG_MS_DELAY        50                  // Standard pacing delay 

#define LED_PIN   LATBbits.LATB4                // Use LAT for Output / Bootloader Led Status 
#define LED_TRIS  TRISBbits.TRISB4              // Output PortB.4 pin

uint16_t flash_packet[FLASH_WRITE_BLOCK * 2];   // Array of 128 words

//-------------------------------------------------------
// INTERNAL OSCILLATOR CLK CONFIG
//-------------------------------------------------------
void INTOSC_Init(void)
{
    CLKDIVbits.RCDIV = 0; 
    CLKDIVbits.CPDIV = 0; 

    __builtin_write_OSCCONH(0x01); 
    __builtin_write_OSCCONL(OSCCON | 0x01); 

    #ifndef __DEBUG
        while (OSCCONbits.COSC != 0b001); 
    #endif
}


/* In Bootloader.c */
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void)
{
    // High-speed assembly jump to the Application's entry point
    __asm__("goto 0x908"); 
}


//-------------------------------------------------------
// READ AND WRITE PROGRAM CODE ROUTINE + READ EEPROM DATA
//-------------------------------------------------------
// READ FLASH DATA (1 Word)
int32_t Flash_ReadInstruction(uint32_t addr)
{
    uint16_t low16, high16;
    int32_t instr32 = 0;

    // Set table page
    TBLPAG = (uint16_t)((addr >> 16) & 0xFF);

    // Read low (MID+LSB)
    low16  = __builtin_tblrdl((uint16_t)(addr & 0xFFFF));

    // Read high (MSB+phantom)
    high16 = __builtin_tblrdh((uint16_t)(addr & 0xFFFF));

    // Assemble [Phantom][MSB][MID][LSB]
    instr32  = ((int32_t)((high16 >> 8) & 0xFF)) << 24;     // Phantom (high byte of high16)
    instr32 |= ((int32_t)(high16 & 0xFF)) << 16;            // MSB (low byte of high16)
    instr32 |= ((int32_t)((low16 >> 8) & 0xFF)) << 8;       // MID
    instr32 |= (low16 & 0xFF);                              // LSB

    return instr32;
}

void Flash_WriteBlock(uint32_t address, uint16_t *data)
{
    // 1. Setup NVMCON (0x4002 = Row Write)
    NVMCON = 0x4002;  

    // 2. Set TBLPAG to Write Latches
    TBLPAG = 0xFA;

    // 3 & 4. Load the latches
    // The guide says target the latches starting at offset 0
    uint16_t latch_offset = 0; 
    
    for (uint16_t i = 0; i < FLASH_WRITE_BLOCK; i++) 
    {
        // data[i*2] is the Low Word (LSW)
        // data[i*2 + 1] is the High Byte (MSB)
        __builtin_tblwtl(latch_offset, data[i * 2]);
        __builtin_tblwth(latch_offset, data[i * 2 + 1]);
        
        latch_offset += 2; // Increment to next instruction slot in latches
    }

    // 6. Set destination in Flash
    NVMADR  = (uint16_t)(address & 0xFFFF);
    NVMADRU = (uint16_t)((address >> 16) & 0x7F);

    // 7. Unlock and Write
    __builtin_disi(5);
    NVMKEY = 0x55;
    NVMKEY = 0xAA;
    NVMCONbits.WR = 1;

    __builtin_nop(); // Guide asks for 3 NOPs, your code had 2. 
    __builtin_nop(); // Adding a 3rd for safety.
    __builtin_nop();

    while (NVMCONbits.WR);
}

//-------------------------------------------------------
// VERIFY FLASH DATA
//-------------------------------------------------------
void Verify_Flash(void)
{
    uint32_t addr;

    UART_TxString("<StartFlashVerify>");
    __delay_ms(MSG_MS_DELAY);

    for (addr = FLASH_START; addr <= FLASH_END; addr += (FLASH_WRITE_BLOCK * 2))
    {
        uint32_t packet[FLASH_WRITE_BLOCK];

        // Read block (24-bit instructions)
        for (uint8_t i = 0; i < FLASH_WRITE_BLOCK; i++)
        {
            packet[i] = Flash_ReadInstruction(addr + (uint32_t)(i * 2));
        }

        // Send block
        for (uint8_t i = 0; i < FLASH_WRITE_BLOCK; i++)
        {
            UART_Tx((packet[i] >>  0) & 0xFF);
            UART_Tx((packet[i] >>  8) & 0xFF);
            UART_Tx((packet[i] >> 16) & 0xFF);
            UART_Tx((packet[i] >> 24) & 0xFF);
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
    unsigned long progAddr;   // Address of page to erase

    UART_TxString("<StartFlashErase>");
    __delay_ms(MSG_MS_DELAY);

    for (progAddr = FLASH_START; progAddr < FLASH_END; progAddr += FLASH_ERASE_BLOCK * 2)
    {
        // Set up pointer to the first program memory location to erase
        NVMADRU = progAddr >> 16;        // Upper address
        NVMADR  = progAddr & 0xFFFF;     // Lower address

        // Initialize NVMCON for page erase
        NVMCON = 0x4003;

        // Required unlock sequence
        asm("DISI #5");                  // Disable interrupts
        __builtin_write_NVM();           // Start erase operation

        // Wait for operation to complete
        while (NVMCONbits.WR);

    }

    UART_TxString("<EndFlashErase>");
    __delay_ms(MSG_MS_DELAY);
}


//-------------------------------------------------------
// WAIT HANDSHAKE AND FIRWARE UPDATE ROUTINE
//-------------------------------------------------------
bool ReceivePacket(void)
{
    // 64 instructions * 4 bytes/instruction = 256 bytes
    uint8_t temp[FLASH_WRITE_BLOCK * 4];  
    uint16_t byteCount = 0;
    uint32_t timeoutCounter = 0;
    const uint32_t TIMEOUT_3SEC = 6000000;
    
    // 1. Collect exactly 256 bytes (matches B4J intBlockSize)
    while (byteCount < (FLASH_WRITE_BLOCK * 4))
    {
        if (U1STAbits.URXDA) 
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

    // 2. Combine bytes into 128 16-bit words (64 Low words + 64 High words)
    // Loop runs 128 times (FLASH_WRITE_BLOCK * 2)
    for (uint16_t i = 0; i < (FLASH_WRITE_BLOCK * 2); i++)
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
        // ReceivePacket handles the 256-byte (FLASH_WRITE_BLOCK * 4) collection
        if (ReceivePacket())  
        {
            timeoutCount = 0; 

            // 2. FLASH PROGRAMMING
            // Commits 128 words (64 Low, 64 High) to the PIC24 Row
            Flash_WriteBlock(flashAddr, flash_packet);

            // 3. ADDRESS POINTER ADVANCEMENT
            // PIC24: 64 instructions = 128 address units (0x80)
            flashAddr += (FLASH_WRITE_BLOCK * 2);  

            // 4. BOUNDARY CHECK
            // Stop if the next write would exceed the App limit
            if (flashAddr + (FLASH_WRITE_BLOCK * 2) - 1 > FLASH_END)
            {
                UART_TxString("<EndFlashWrite>");
                __delay_ms(MSG_MS_DELAY);
                
                Verify_Flash(); // This will also need to walk in 256-byte steps
                return;
            }
        }
        // 5. ERROR / RETRY HANDLING
        else 
        {
            timeoutCount++;

            if (timeoutCount >= 3)
            {
                UART_TxString("<ErrorTimeout>");
                __delay_ms(MSG_MS_DELAY);
                return;
            }
            // Optional: Request resend from B4J here
        }
    }
}

// 0x55 and 0XAA handshake expected from Host to start firmware update
void WaitHandshake(void) 
{
    uint8_t prev = 0;
    uint8_t curr = 0;
    uint32_t handshakeCounter = 0;
    const uint32_t TIMEOUT_3SEC = 6000000;
    
     while (handshakeCounter < TIMEOUT_3SEC)
    {
        // PIC24 uses U1STAbits.URXDA to check for data in the FIFO
        if (U1STAbits.URXDA) 
        {
            // UART_Rx() should return U1RXREG on this chip
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
int main(void) {    
     // 1. Digital/Analog Configuration
    // The GA102 uses AD1PCFG. Setting a bit to 1 makes the pin DIGITAL.
    ANSA = 0x0000;   // all PORTA digital
    ANSB = 0x0000;   // all PORTB digital

    // 2. I/O Setup
    LED_TRIS = 0;                   // LED Output
    LED_PIN  = 1;                   // LED On (bootloader led)
       
    // 3. Peripheral Init
    // Ensure your INTOSC_Init sets the clock to 32MHz (Fcy = 16MHz)
    INTOSC_Init();                  
    UART_Init();                    // Init Hardware UART (PPS happens inside here) 
     
    WaitHandshake();                // wait for 0x55 0xAA (3s timeout then goto app))
    
    LED_PIN  = 0;                   // LED Off (bootloader led))
    
    asm("goto 0x800");            
    
    // Good news is when bootloader goes to 0x0800 and is invalid, causes pic to reset and main repeated over and over till handshake and flash success!
}

