/*
 * File:   bootloader.c
 * Version: 3.10
 * Author: Issac
 * Family: 24F64GA102
 * Created on January 19, 2026, 2:50 PM
 * USE 1.10.375
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

// Note: B4J Expected bytes = 0xABF6 - 0x0800 = 83,952 BYTES with Phantom!

// Adjusted for PIC24FJ64GA102 based on your .gld configuration
#define FLASH_START          0x00800            // Matches your Application ORIGIN
#define FLASH_END            0x0ABF6            // Matches your Application END (Last address)

// PIC24FJ64GA102 Specific Flash geometry
#define FLASH_ERASE_BLOCK    512                // Erase page size is 512 instructions (standard for GA102)
#define FLASH_WRITE_BLOCK    64                 // Write row size is 64 instructions (standard for GA102)

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
    // 1. Configure the Oscillator (FRC with PLL)
    // The PIC24F internal FRC is ~8 MHz. 
    // To get 32 MHz (Max), we use the 4x PLL.
    // CLKDIV: RCDIV = 000 (8MHz), DOZE = 0 (1:1)
    CLKDIVbits.RCDIV = 0; 

    // 2. Initiate Clock Switch to FRC with PLL (NOSC = 0b001)
    // OSCCONL/H are protected by an unlock sequence in some compilers, 
    // but __builtin_write_OSCCONH is the safest method.
    __builtin_write_OSCCONH(0x01); // NOSC = 001 (Fast RC Oscillator with PLL)
    __builtin_write_OSCCONL(OSCCON | 0x01); // OSWEN = 1 (Request switch)

    #ifndef __DEBUG
        // 3. Wait for Clock Switch to complete
        // COSC (Current Oscillator) should match NOSC (New Oscillator)
        while (OSCCONbits.COSC != 0b001); 

        // 4. Wait for the PLL to lock
        // Unlike Q43's HFOR, PIC24 uses the LOCK bit for PLL stability
        while (OSCCONbits.LOCK != 1); 
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
    uint16_t addr_offset = (uint16_t)(address & 0xFFFF);
    uint16_t addr_page = (uint16_t)((address >> 16) & 0x007F);

    // 1. Setup NVMCON for row programming FIRST
    NVMCON = 0x4001; 

    // 2. Set TBLPAG to the ACTUAL destination page (not 0xFA)
    TBLPAG = addr_page;
    
    // 3. Load ALL 64 instructions
    // Using explicit offsets ensures we don't drift past the row boundary
    for (uint16_t i = 0; i < FLASH_WRITE_BLOCK; i++)
    {
        uint16_t target = addr_offset + (i * 2);
        
        __builtin_tblwtl(target, data[i * 2]);     
        __builtin_tblwth(target, data[i * 2 + 1]); 
    }

    // 4. Unlock and Trigger
    __builtin_disi(5);
    NVMKEY = 0x55;
    NVMKEY = 0xAA;
    NVMCONbits.WR = 1;

    // 5. Mandatory NOPs from your assembly example
    __builtin_nop();
    __builtin_nop();

    // 6. Wait for hardware to clear the WR bit
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
    uint32_t addr;

    UART_TxString("<StartFlashErase>");
    __delay_ms(MSG_MS_DELAY);
    
    for (addr = FLASH_START; addr < FLASH_END; addr += FLASH_ERASE_BLOCK * 2)
    {
        // 2. Set up pointer (Matches your PDF example)
        TBLPAG = (uint16_t)((addr >> 16) & 0x007F);
        uint16_t offset = (uint16_t)(addr & 0xFFFF);

        // 3. Dummy Write to set base address
        __builtin_tblwtl(offset, 0xFFFF); 

        // 4. Initialize NVMCON (Matches your 0x4042)
        NVMCON = 0x4042; 

        // 5. THE UNLOCK SEQUENCE
        // Using the built-in function is the "Correct" way to 
        // match the PDF's assembly requirements exactly.
        __builtin_disi(5);      
        __builtin_write_NVM();  

        // 6. Wait for completion
        while(NVMCONbits.WR);        
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
    AD1PCFG = 0xFFFF;               // Set all pins to Digital mode

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

