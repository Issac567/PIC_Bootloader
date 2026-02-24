/*
 * File:   bootloader.c
 * Version: 3.01
 * Author: Issac
 * Created on January 19, 2026, 2:50 PM
 * Family: 16F88
 */


#include <xc.h>
#include <stdint.h>                         // for standard integer types 
#include <stdbool.h>                        // for bool, true, false
#include "config.h"
#include "uart.h"

// Note: B4J Expected bytes = 0xFFF - 0x600 = 0x9FF + 1 = 0xA00(2560) * 2 = 5120 BYTES (Each address is 1 WORD!))

#define FLASH_START 0x0600                  // Flash start address
#define FLASH_END 0x0FFF                    // Flash end address
#define FLASH_ERASE_BLOCK 32                // Runtime can only do 32 word block erase max!
#define FLASH_WRITE_BLOCK 4                 // Can only do 4 word block write max with PIC 16F88!              
#define MSG_MS_DELAY 50                     // Delay for UART_TxString 

#define LED_PIN PORTBbits.RB4               // Bootloader Led Status    
#define LED_TRIS TRISBbits.TRISB4           // Output PortB.4 pin

uint16_t flash_packet[FLASH_WRITE_BLOCK];   // 4 words, 8 bytes total

//-------------------------------------------------------
// INTERNAL OSCILLATOR CLK CONFIG
//-------------------------------------------------------
void INTOSC_Init(void)
{
    /**
    * @brief Initializes the Internal Oscillator (INTOSC).
    * * Sets the internal oscillator frequency to 8 MHz and selects 
    * the internal oscillator block as the system clock source.
    */
    
    // Set Internal Oscillator Frequency Select bits (IRCF<2:0>)
    // 111 = 8 MHz (Note: Specific frequencies depend on the MCU datasheet)
    OSCCONbits.IRCF2 = 1;           
    OSCCONbits.IRCF1 = 1;  
    OSCCONbits.IRCF0 = 1;  

    // Set System Clock Select bits (SCS<1:0>)
    // 1x = Internal oscillator block
    // 01 = Timer1 oscillator
    // 00 = Primary oscillator (defined by FOSC configuration bits)
    OSCCONbits.SCS1  = 1;           // Select Internal Oscillator
    OSCCONbits.SCS0  = 0;   
}

void __interrupt() boot_ISR(void)
{
    asm("PAGESEL 0xF00");
    asm("CALL 0xF00");
}


//-------------------------------------------------------
// READ AND WRITE PROGRAM CODE ROUTINE
//-------------------------------------------------------
// READ FLASH DATA (1 Word)
uint16_t Flash_ReadWord(uint16_t address)
{
    uint16_t word;

    EEADR  = address & 0xFF;            // low byte of address
    EEADRH = (address >> 8) & 0xFF;     // high byte of address

    EECON1bits.EEPGD = 1;               // Select program memory type
    EECON1bits.RD    = 1;               // initiate read

    NOP();                              // required
    NOP();
           
    word  = EEDATA;                     // Low 8 bits
    word |= ((uint16_t)EEDATH << 8);    // High 6 bits
    word &= 0x3FFF;                     // Mask 14-bit instruction
    
    return word;                        // Full 14-bit instruction
}

// WRITE FLASH BLOCK Starting Address + 4 (8 Word or 4 Addresses in 1 call)
void Flash_WriteBlock(uint16_t address, uint16_t *data) 
{
    uint16_t i;

    address &= 0xFFFC;                      // align to 4-word block
    INTCONbits.GIE = 0;                     // Disable interrupts
            
    for (i = 0; i < FLASH_WRITE_BLOCK; i++) 
    {
        EECON1bits.EEPGD = 1;               // Select program memory type
        EECON1bits.WREN  = 1;               // Enable write
        EECON1bits.FREE  = 0;               // Stop erase
        
        uint16_t currentWordAddr = address + i;
        EEADR  = currentWordAddr & 0xFF;         // Extract bits <7:0>
        EEADRH = (currentWordAddr >> 8) & 0x3F;  // Extract bits <13:8>, masking out top 2 bits
        
        //EEADR  = (address + i) & 0x00FF;    // Low byte address
        //EEADRH = (address + i) >> 8;        // High byte address
        
        //data = MSB LSB order
        EEDATA = data[i] & 0xFF;            // Masking here is standard
        EEDATH = (data[i] >> 8) & 0x3F;     // Masking here ensures only 6 bits are used

        EECON2 = 0x55;                      // Unlock sequence
        EECON2 = 0xAA;

        EECON1bits.WR = 1;                  // Start write
        while (EECON1bits.WR);              // wait until this word is written
        
        NOP();                              // Short delay required
        NOP();                 
    }
    EECON1bits.WREN = 0;                    // Disable writes
    INTCONbits.GIE = 1;                     // Enable interrupts
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
        // Prepare 4-word packet
        uint16_t packet[FLASH_WRITE_BLOCK];
        for (uint8_t i = 0; i < FLASH_WRITE_BLOCK; i++)
        {
            packet[i] = Flash_ReadWord(addr + i);  // Read word from flash
        }

        // Send packet to B4J 
        for (uint8_t i = 0; i < FLASH_WRITE_BLOCK; i++)
        {
            // eg. 0x3FFF = FF first then 3F second (B4J binary is backwards!)
            UART_Tx(packet[i] & 0xFF);              // First Byte (LSB)
            UART_Tx(packet[i] >> 8);                // Second Byte (MSB) Shift upper to lower
        }

        __delay_ms(10);                             // tested 1 ms ok!  as long below first delay is there!
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
    // Send to host
    UART_TxString("<StartFlashErase>");    
    __delay_ms(MSG_MS_DELAY);
    
    uint16_t addr;
 
    // Application area: 0x0600 to 0xFFF 
    for (addr = FLASH_START; addr + FLASH_ERASE_BLOCK - 1 <= FLASH_END; addr += FLASH_ERASE_BLOCK) 
    {
        EEADR  = addr & 0xFF;       
        EEADRH = addr >> 8;

        EECON1bits.EEPGD = 1;       // Select program memory type
        EECON1bits.WREN  = 1;       // Enable write
        EECON1bits.FREE  = 1;       // Enable erase

        INTCONbits.GIE = 0;         // Disable interrupts
        
        EECON2 = 0x55;              // Unlock Sequence
        EECON2 = 0xAA;
        
        EECON1bits.WR = 1;          // Trigger erase
        while (EECON1bits.WR);      // wait until this word is written

        NOP();                      // Sequence Required
        NOP();
        
        EECON1bits.FREE = 0;        // Stop erase
        EECON1bits.WREN = 0;        // Stop write
        INTCONbits.GIE = 1;         // Enable interrupts
    }
    
    // Send to host
    UART_TxString("<EndFlashErase>");  
    __delay_ms(MSG_MS_DELAY);
}


//-------------------------------------------------------
// WAIT HANDSHAKE AND FIRWARE UPDATE ROUTINE
//-------------------------------------------------------
bool ReceivePacket(void)
{
    // Buffer for raw incoming bytes (e.g., 8 bytes)
    uint8_t temp[FLASH_WRITE_BLOCK * 2]; 
    uint8_t byteCount = 0;      
    uint32_t timeout_counter = 0;
    
    // Adjust this value based on your Fosc (Clock Speed)
    // For 8MHz, 3 seconds is roughly 600,000 to 1,000,000 iterations
    const uint32_t THREE_SECONDS = 200000; 
    
    while (byteCount < (FLASH_WRITE_BLOCK * 2)) 
    {    
        // 1. Check for UART Data
        if (PIR1bits.RCIF) 
        {
            temp[byteCount] = RCREG; 
            byteCount++;      
            timeout_counter = 0; // Reset timeout every time a byte arrives
        }
        else 
        {
            // 2. Increment Software Timeout if no data
            timeout_counter++;
            
            // 3. Exit if we hit 3 seconds
            if (timeout_counter >= THREE_SECONDS)
            {
                return false; // Timeout reached
            }
        }

    }

    // --- Byte-to-Word Reassembly ---
    for (uint8_t i = 0; i < FLASH_WRITE_BLOCK; i++) 
    {
        flash_packet[i] = ((uint16_t)temp[i*2 + 1] << 8) | temp[i*2];
    }

    return true; 
}

void DoFirmwareUpdate(void)
{   
    /**
    * @brief Main execution loop for the PIC16F88 firmware update.
    * Uses software-polled timeouts instead of Timer2 ISR.
    */
    
    // Notify Host (B4J) that the MCU is ready
    UART_TxString("<StartFlashWrite>");     
    __delay_ms(MSG_MS_DELAY);
    
    uint16_t flashAddr = FLASH_START;       
    uint8_t timeoutCount = 0;               

    // Send Acknowledge: Host sends next 8-byte packet after seeing this
    UART_TxString("<ACK>");  
    
    while (1)
    {
        // Attempt to capture a packet. 
        // NOTE: ReceivePacket now handles the 3-second software timeout internally.
        if (ReceivePacket())                
        {
            timeoutCount = 0; // Success, reset the "hate" counter
                        
            // PIC16F88 writes are typically done in blocks
            Flash_WriteBlock(flashAddr, flash_packet);

            // Increment address (Note: PIC16 addresses words, not bytes)
            flashAddr += FLASH_WRITE_BLOCK;     
             
            // Boundary Check
            if (flashAddr + FLASH_WRITE_BLOCK - 1 > FLASH_END)
            {
                UART_TxString("<EndFlashWrite>");
                __delay_ms(MSG_MS_DELAY);  
                 
                Verify_Flash(); // Final read-back
                return; 
            }
        }
        else 
        {
            // If ReceivePacket returns false, it means 3 seconds passed with no UART data
            timeoutCount++;
            
            UART_TxString("<ISR Timeout>");  // Simplified message for smaller flash
            __delay_ms(MSG_MS_DELAY);              
            
            if (timeoutCount >= 3)
            {
                UART_TxString("<ErrorTimeout>");    
                __delay_ms(MSG_MS_DELAY);
                return; // Abort
            } 
        }
    }
}


void WaitHandshake(void) 
{ 
    uint8_t prev = 0; 
    uint8_t curr; 
    uint32_t timeout_counter = 0;
    const uint32_t THREE_SECONDS = 200000; 
        
    while (timeout_counter < THREE_SECONDS)
    {
        // 1. Check for UART data
        if(PIR1bits.RCIF) 
        {
            curr = RCREG; // Read directly for speed
                
            if(prev == 0x55 && curr == 0xAA) 
            {                         
                UART_TxString("<InitReceived>");
                __delay_ms(MSG_MS_DELAY);
                
                Flash_EraseApplication(); 
                DoFirmwareUpdate();       
                
                return; 
            }
            
            prev = curr;
            // Optional: Reset timeout_counter = 0; if you want the 3s 
            // to restart every time a character is typed.
        }
        else 
        {
            // 2. No data received? Increment the timer
            timeout_counter++;
        }
    }
        
    // If loop finishes, it means timeout_counter hit THREE_SECONDS
    UART_TxString("<HandShakeTimeout>");
    __delay_ms(MSG_MS_DELAY);
}


//-------------------------------------------------------
// MAIN ENTRY FOR BOOTLOADER
//-------------------------------------------------------
void main(void) 
{
    ANSEL  = 0x00;                  // Disable analog (VERY IMPORTANT)
    
    LED_TRIS = 0;                   // Output
    LED_PIN  = 1;                   // LED On (bootloader led))
       
    INTOSC_Init();                  // Must set internal Oscillator
    UART_Init();                    // Init Hardware UART   
    
    WaitHandshake();                // wait for 0x55 0xAA (3s timeout then goto app))
    
    LED_PIN  = 0;                   // LED Off (bootloader led))
  
    asm("PAGESEL 0x600");           // Ensure PCLATH is correct for the jump
    asm("goto 0x600");              // If bootloader is not init from PC, then continue to application
    
    
    // Good news is when bootloader goes to 0x0600 and is invalid, causes PIC to reset and main repeated over and over till handshake and flash success!
}
