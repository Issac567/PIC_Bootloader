/*
 * File:   bootloader.c
 * Version: 3.10
 * Author: Issac
 * Created on January 19, 2026, 2:50 PM
 * Family: 16F13145
 * PACKS: USE 1.29.444
 * Flash required power off MB102 power supply then flash. Turn MB102 on then flash again for MBLAB Snap
 */


#include <xc.h>
#include <stdint.h>                         // for standard integer types 
#include <stdbool.h>                        // for bool, true, false
#include "config.h"
#include "uart.h" 

// Note: B4J Expected bytes = 0x1FFF - 0x600 = 0x19FF + 1 = 0x1A00(6,656) * 2 = 13,312 BYTES (Each address is 1 WORD!))

#define FLASH_START 0x0600                  // Flash start address
#define FLASH_END 0x1FFF                    // Flash end address for 4-word block
#define FLASH_ERASE_BLOCK 32                // Runtime can only do 32 word block erase max!
#define FLASH_WRITE_BLOCK 32                // Can only do 32 word block write max with PIC 16F13145!
#define MSG_MS_DELAY 150                      // (min 150 for BT latency) Standard pacing delay 

#define LED_PIN   LATAbits.LATA4            // Use LAT for Output / Bootloader Led Status 
#define LED_TRIS  TRISAbits.TRISA4          // Output PortB.4 pin

uint16_t flash_packet[FLASH_WRITE_BLOCK];   // 32 words, 64 bytes total


//-------------------------------------------------------
// INTERNAL OSCILLATOR CLK CONFIG
//-------------------------------------------------------
void INTOSC_Init(void)
{
    // Select HFINTOSC, no divider (NDIV = 1)
    OSCCON1 = 0x60;   // NOSC = 110 (HFINTOSC), NDIV = 0000 (÷1)

    // Set frequency to 32 MHz
    OSCFRQ = 0x06;    // HFFRQ = 110 ? 32 MHz

    // Wait for clock to stabilize
    
    #ifndef __DEBUG
        while (OSCSTATbits.HFOR == 0);   // Wait for HFINTOSC ready
    #endif
}


void __interrupt() boot_ISR(void)
{
    asm("PAGESEL 0x1000");
    asm("CALL 0x1000");
}


//-------------------------------------------------------
// READ AND WRITE PROGRAM CODE ROUTINE
//-------------------------------------------------------
// READ FLASH DATA (1 Word)
uint16_t Flash_ReadInstruction(uint16_t address)

{
    uint16_t word;

    // Set program memory address (word address)
    NVMADRL = address & 0xFF;
    NVMADRH = (address >> 8) & 0xFF;

    // Select program memory
    NVMCON1bits.NVMREGS = 0;

    // Start read
    NVMCON1bits.RD = 1;

    // Required: NOP after RD (pipeline requirement)
    NOP();
    NOP();

    // Read data
    word  = NVMDATL;
    word |= ((uint16_t)NVMDATH << 8);

    // Mask to 14-bit instruction
    word &= 0x3FFF;

    return word;
}
void Flash_WriteBlock(uint16_t address, uint16_t *data)
{
    uint8_t i;

    // Align to 32-word boundary
    address &= 0xFFE0;

    // Setup
    NVMCON1bits.NVMREGS = 0;
    NVMCON1bits.WREN = 1;
    NVMCON1bits.LWLO = 1;

    INTCONbits.GIE = 0;   // Disable once

    for (i = 0; i < FLASH_WRITE_BLOCK; i++)
    {
        uint16_t currentWordAddr = address + i;

        NVMADRL = currentWordAddr & 0xFF;
        NVMADRH = (currentWordAddr >> 8);   

        NVMDATL = data[i] & 0xFF;
        NVMDATH = (data[i] >> 8) & 0x3F;

        // Load latch
        NVMCON2 = 0x55;
        NVMCON2 = 0xAA;
        NVMCON1bits.WR = 1;
        while (NVMCON1bits.WR);
    }

    // Commit row
    NVMCON1bits.LWLO = 0;

    NVMCON2 = 0x55;
    NVMCON2 = 0xAA;
    NVMCON1bits.WR = 1;
    while (NVMCON1bits.WR);

    INTCONbits.GIE = 1;   // Re-enable

    // Disable write
    NVMCON1bits.WREN = 0;
}


//-------------------------------------------------------
// VERIFY FLASH DATA
//-------------------------------------------------------
// Verify Flash is performed after Flash Write is completed
void Verify_Flash(void)
{
    uint16_t addr;
    
    // Send to host
    UART_TxString("<StartFlashVerify>");
    __delay_ms(MSG_MS_DELAY);
    
    // Loop through all flash from start to end
    for (addr = FLASH_START; addr + FLASH_WRITE_BLOCK - 1 <= FLASH_END; addr += FLASH_WRITE_BLOCK)
    {
        // Prepare 32-word packet
        uint16_t packet[FLASH_WRITE_BLOCK];
        for (uint8_t i = 0; i < FLASH_WRITE_BLOCK; i++)
        {
            packet[i] = Flash_ReadInstruction(addr + i);  // Read word from flash
        }

        // Send packet to B4J 
        for (uint8_t i = 0; i < FLASH_WRITE_BLOCK; i++)
        {
            // eg. 0x3FFF = FF first then 3F second (B4J binary is backwards!)
            UART_Tx(packet[i] & 0xFF);              // First Byte (LSB)
            UART_Tx(packet[i] >> 8);                // Second Byte (MSB) Shift upper to lower
        }

        __delay_ms(1);                              // tested 1 ms ok!  as long below first delay is there!
    }

    // Send to host
    __delay_ms(MSG_MS_DELAY);                       // Must do this delay first helps alot.  does not interfere with last packet sent! (tested with 1 ms delay above and works flawless!)
    UART_TxString("<EndFlashVerify>");
    __delay_ms(MSG_MS_DELAY);
}


//-------------------------------------------------------
// ERASE FLASH PROGRAM CODE DATA
//-------------------------------------------------------
// ERASE FLASH BLOCK 32 word erase at each for 
void Flash_EraseApplication(void)
{
    UART_TxString("<StartFlashErase>");
    __delay_ms(MSG_MS_DELAY);

    uint16_t addr;

    // Loop through all flash blocks safely
    for (addr = FLASH_START; addr <= FLASH_END; addr += FLASH_ERASE_BLOCK)
    {
        // Load address
        NVMADRL = addr & 0xFF;
        NVMADRH = (addr >> 8) & 0xFF;  // FULL 8 bits

        // Configure for PFM erase
        NVMCON1bits.NVMREGS = 0;       // Program Flash Memory
        NVMCON1bits.FREE    = 1;       // Erase
        NVMCON1bits.WREN    = 1;       // Enable writes

        // Unlock sequence with interrupts disabled
        INTCONbits.GIE = 0;
        NVMCON2 = 0x55;
        NVMCON2 = 0xAA;
        NVMCON1bits.WR = 1;            // Start erase
        while (NVMCON1bits.WR);        // Wait for completion
        INTCONbits.GIE = 1;

        // Disable writes
        NVMCON1bits.WREN = 0;
    }

    UART_TxString("<EndFlashErase>");
    __delay_ms(MSG_MS_DELAY);
}


//-------------------------------------------------------
// FIRMWARE UPDATE ROUTINE
//-------------------------------------------------------
bool ReceivePacket(void)
{
    // Buffer for raw bytes (e.g., 64 bytes if FLASH_WRITE_BLOCK is 32)
    uint8_t temp[FLASH_WRITE_BLOCK * 2];        
    uint8_t byteCount = 0; 
    uint32_t timeout_counter = 0;

    const uint32_t THREE_SECONDS = 1000000; 
    
    while (byteCount < FLASH_WRITE_BLOCK * 2)   
    {
        // Check if UART receive flag is set
        if (PIR4bits.RC1IF)                          
        {       
            temp[byteCount] = UART_Rx();        // Capture incoming byte
            byteCount++;                                
            timeout_counter = 0;                // Reset timeout every time a byte arrives
                }
        else 
        {
            // Increment Software Timeout if no data
            timeout_counter++;
            
            // Exit if we hit 3 seconds
            if (timeout_counter >= THREE_SECONDS)
            {
                return false; // Timeout reached
            }
        }
    }

    /* * Convert bytes into 16-bit words.
     * Host sends LSB first, then MSB (Little Endian).
     * Example: Host sends 0x3FFF as [0xFF, 0x3F].
     */
    for (uint8_t i = 0; i < FLASH_WRITE_BLOCK; i++) 
    {
        // flash_packet[i] = (MSB << 8) | LSB
        flash_packet[i] = ((uint16_t)temp[i*2 + 1] << 8) | temp[i*2];
    }

    return true;
}

// When ready will use this in future
void DoFirmwareUpdate(void)
{    
    UART_TxString("<StartFlashWrite>");     // Start of Flash Write Block
    __delay_ms(MSG_MS_DELAY);
    
    uint16_t flashAddr = FLASH_START;
    uint8_t timeoutCount = 0;               // count consecutive timeouts
        
    while (1)
    {      
        // Send Acknowledge: Host sends next 64-byte packet after seeing this
        UART_TxString("<ACK>");  
    
        if (ReceivePacket())                // Check packet 64 bytes total 32 word
        {          
            // Successfully received a packet, reset timeout counter
            timeoutCount = 0;
                        
            // Write 4-word block to flash
            Flash_WriteBlock(flashAddr, flash_packet);

            // Move to next flash block
            flashAddr += FLASH_WRITE_BLOCK;     // currently +32
             
            // stop if we reach end of flash memory
            if (flashAddr + FLASH_WRITE_BLOCK - 1 > FLASH_END)
            {
                // this will trigger B4J to get in Verify Mode! Send to host
                UART_TxString("<EndFlashWrite>");
                __delay_ms(MSG_MS_DELAY);  
                 
                Verify_Flash();
                
                return;
            }
        }
        else        // Packet returned false 
        {
            // If ReceivePacket returns false, it means 3 seconds passed with no UART data
            timeoutCount++;

            UART_TxString("<ISR Timeout>");  // Simplified message for smaller flash
            __delay_ms(MSG_MS_DELAY);   
            
            // Exit after 3 consecutive timeouts
            if (timeoutCount >= 3)
            {                   
                // Send to host
                UART_TxString("<ErrorTimeout>");    
                __delay_ms(MSG_MS_DELAY);
                
                return;
            } 
            
            // Otherwise, continue waiting for next packet
        }
    }
}


//-------------------------------------------------------
// WAIT HANDSHAKE
//-------------------------------------------------------
// 0x55 and 0XAA handshake expected from Host to start firmware update
void WaitHandshake(void) {
    uint8_t prev = 0;
    uint8_t curr;
    uint32_t timeout_counter = 0;
    
    const uint32_t THREE_SECONDS = 1000000; 
    
     while (timeout_counter < THREE_SECONDS)
    {
        if(PIR4bits.RC1IF)               // UART receive interrupt flag set (data received in RCREG)
        {
            curr = UART_Rx();
                
            // Expecting 0xAA and 0x55 from PC to enter Flash mode
            if(prev == 0x55 && curr == 0xAA) 
            {                           
                // Send initialization acknowledgment before starting Erase and Flash update
                UART_TxString("<InitReceived>");
                __delay_ms(MSG_MS_DELAY);
                
                // Update the Firmware        
                Flash_EraseApplication();               // Erase Flash
                DoFirmwareUpdate();                     // Flash Write
                return;
            }
            
            // sliding window byte comparison
            prev = curr;
        }
        else 
        {
            // 2. No data received? Increment the timer
            timeout_counter++;
        }
    }
        
    // Send to host
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
    
    asm("PAGESEL 0x600");           // Ensure PCLATH is correct for the jump
    asm("goto 0x600");              // If bootloader is not init from PC, then continue to application

    // Good news is when bootloader goes to 0x0600 and is invalid, causes pic to reset and main repeated over and over till handshake and flash success!
}
