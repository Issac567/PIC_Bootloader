/*
 * File:   bootloader.c
 * Version: 2.01...............
 * Author: Issac
 *
 * Created on January 19, 2026, 2:50 PM
 */


#include <xc.h>
#include <stdint.h>                         // for standard integer types 
#include <stdbool.h>                        // for bool, true, false
#include "config.h"

// Note: B4J Expected bytes = 0x7FFF - 0x600 = 0x79FF + 1 = 0x7A00(31,232) * 2 = 62,464 BYTES (Each address is 1 WORD!))

#define FLASH_START 0x0600                  // Flash start address
#define FLASH_END 0x7FFF                    // Flash end address for 4-word block
#define FLASH_ERASE_BLOCK 32                // Runtime can only do 32 word block erase max!
#define FLASH_WRITE_BLOCK 32                // Can only do 32 word block write max with PIC 16F18857!
#define MSG_MS_DELAY 50                     // Delay for UART_TxString   

#define LED_PIN   LATBbits.LATB4            // Use LAT for Output / Bootloader Led Status 
#define LED_TRIS  TRISBbits.TRISB4          // Output PortB.4 pin

uint16_t flash_packet[FLASH_WRITE_BLOCK];   // 32 words, 64 bytes total


//-------------------------------------------------------
// INTERNAL OSCILLATOR CLK CONFIG
//-------------------------------------------------------
void INTOSC_Init(void)
{
  // Select HFINTOSC as system clock, NDIV = 1
    OSCCON1 = 0b01100000;  // 6-4 = 110 ? HFINTOSC, 3-0 NDIV = 0000 ? divide by 1


    // Set HFINTOSC frequency to 32 MHz
    OSCFRQ = 0b00000110;  // HFFRQ=110 ? 32 MHz (redundant if RSTOSC=000)
    
    //Note 1: When RSTOSC=110 (HFINTOSC 1 MHz), the HFFRQ bits will default to ?010? upon Reset; 
    //        When RSTOSC = 000 (HFINTOSC 32 MHz), the HFFRQ bits will default to ?110? upon Reset.
    
    /*RSTOSC<2:0>: Power-up Default Value for COSC bits This value is the Reset default value for COSC, 
    and selects the oscillator first used by user software 
    111 = EXT1X EXTOSC operating per FEXTOSC bits 
    110 = HFINT1 
    101 = LFINT 
    100 = SOSC 
    011 = Reserved 
    010 = EXT4X HFINTOSC (1 MHz) LFINTOSC SOSC EXTOSC with 4x PLL, with EXTOSC operating per FEXTOSC bits 
    001 = HFINTPLL HFINTOSC with 2x PLL, with OSCFRQ = 16 MHz and CDIV = 1:1(FOSC = 32 MHz) 
    000 = HFINT32 HFINTOSC with OSCFRQ= 32 MHz and CDIV = 1:1

    */
    
    //while(OSCCON3bits.ORDY == 0);
}


//-------------------------------------------------------
// UART ROUTINE
//-------------------------------------------------------
void UART_Init(void)
{
    // 1. Set pin directions
    TRISBbits.TRISB2 = 1; // RB2 as Input (RX)
    TRISBbits.TRISB5 = 0; // RB5 as Output (TX)

    // 2. PPS Unlock Sequence
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0; // Unlock PPS

    // 3. Map RX to RB2
    RXPPS = 0x0A; // RB2->EUSART:RX;

    // 4. Map RB5 to TX
    RB5PPS = 0x10; // RB5->EUSART:TX;

    // 5. PPS Lock Sequence
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 1; // Lock PPS

    // ----- Baud rate (57600 @ 32MHz HFINTOSC) -----
    SP1BRGL = 34;
    SP1BRGH = 0;
    BAUD1CONbits.BRG16 = 0; // 8-bit baud generator
    TX1STAbits.BRGH = 1;    // High-speed

    // ----- UART mode -----
    TX1STAbits.SYNC = 0;    // Asynchronous
    RC1STAbits.SPEN = 1;    // Enable UART pins

    // ----- Enable TX and RX -----
    TX1STAbits.TXEN = 1;
    RC1STAbits.CREN = 1;

    // ----- Clear pending RX bytes -----
    uint8_t dummy;
    while (PIR3bits.RCIF) {
        dummy = RC1REG;      // discard
    }
}

void UART_Tx(uint8_t d)
{
    while (!PIR3bits.TXIF);   // Wait until TX1 ready
    TX1REG = d;               // Send data
}

void UART_TxString(const char *s)
{        
    for (uint16_t i = 0; s[i] != '\0'; i++)     // Loop using index
    {
        UART_Tx(s[i]);
    }
}

uint8_t UART_Rx(void)
{
    // Check if a receive overrun occurred
    if (RC1STAbits.OERR)
    {
        RC1STAbits.CREN = 0;   // Reset continuous receive
        RC1STAbits.CREN = 1;   // Re-enable receive
    }

    return RC1REG;             // Return received byte
}


void __interrupt() boot_ISR(void)
//void __at(0x0004) v_isr(void) 
{
    asm("PAGESEL 0x7000");
    asm("CALL 0x7000");
}


//-------------------------------------------------------
// READ AND WRITE PROGRAM CODE ROUTINE + READ EEPROM DATA
//-------------------------------------------------------
// READ FLASH DATA (1 Word)
uint16_t Flash_ReadWord(uint16_t address)
{
    uint16_t word;

    NVMADRL  = address & 0xFF;          // low byte of address
    NVMADRH = (address >> 8) & 0xFF;    // high byte of address

    NVMCON1bits.NVMREGS = 0;            // Select program memory type
    NVMCON1bits.RD = 1;                 // initiate read

    word  = NVMDATL;                    // Low 8 bits
    word |= ((uint16_t)NVMDATH << 8);   // High 6 bits shifted to MSB 
    word &= 0x3FFF;                     // Mask 14-bit instruction
    
    return word;                        // Full 14-bit instruction
}

void Flash_WriteBlock(uint16_t address, uint16_t *data)
{
    uint8_t i;

    // -----------------------------
    // Align address to start of row
    // -----------------------------
    // Each row is 32 words (64 bytes). We mask the lower 5 bits to ensure
    // we start at the beginning of a 32-word row. This prevents the
    // "row shift" issue where data could start at 0x0920 instead of 0x0900.
    address &= 0xFFE0;

    // -----------------------------
    // Enable flash write mode
    // -----------------------------
    NVMCON1bits.NVMREGS = 0;  // Select Program Flash memory
    NVMCON1bits.WREN    = 1;  // Enable writes
    NVMCON1bits.LWLO    = 1;  // Load Latches Only (don't commit to flash yet)

    // -----------------------------
    // Load each word into its latch
    // -----------------------------
    for (i = 0; i < FLASH_WRITE_BLOCK; i++)
    {
        // Calculate the address for this specific word in the row
        // This ensures each word goes to the correct latch, avoiding
        //NVMADRH = 0x0000 - 0x7FFF	Set 0x7F
        //NVMADRH =	0x8000 - 0xFFFF	Set to 0xFF (or none)
        uint16_t currentWordAddr = address + i;
        NVMADRL = currentWordAddr & 0xFF;           // Lower 8 bits of address
        NVMADRH = (currentWordAddr >> 8) & 0x7F;    // Upper 7 bits of address (32KW range)

        // Load the word into NVMDATL/NVMDATH (LSB/MSB)
        // The 16F18857 uses 14-bit words, so upper 2 bits are masked
        NVMDATL = data[i] & 0xFF;                   // Lower 8 bits
        NVMDATH = (data[i] >> 8) & 0x3F;            // Upper 6 bits

        // -----------------------------
        // Unlock and write the word to the latch
        // -----------------------------
        // Each word requires the special unlock sequence:
        // 1. Disable interrupts (GIE = 0)
        // 2. Write 0x55 and 0xAA to NVMCON2
        // 3. Set WR = 1 to load the latch
        // 4. Wait for WR to complete
        INTCONbits.GIE = 0;     // Disable interrupts to avoid write interruption
        NVMCON2 = 0x55;         // Unlock sequence part 1
        NVMCON2 = 0xAA;         // Unlock sequence part 2
        NVMCON1bits.WR = 1;     // Start write to latch
        while (NVMCON1bits.WR); // Wait until latch write completes
        INTCONbits.GIE = 1;     // Re-enable interrupts
    }

    // -----------------------------
    // Commit the entire row to flash
    // -----------------------------
    // After all words are loaded into their latches, clear LWLO = 0
    // to indicate we want to write all latches to the actual flash row.
    NVMCON1bits.LWLO = 0;

    // Perform the unlock/write sequence for the final row commit
    INTCONbits.GIE = 0;     // Disable interrupts
    NVMCON2 = 0x55;         // Unlock part 1
    NVMCON2 = 0xAA;         // Unlock part 2
    NVMCON1bits.WR = 1;     // Start row write
    while (NVMCON1bits.WR); // Wait for completion
    INTCONbits.GIE = 1;     // Re-enable interrupts

    // -----------------------------
    // Cleanup: disable flash writes
    // -----------------------------
    NVMCON1bits.WREN = 0;   // Prevent accidental writes
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
    
    // Loop over all blocks in the application area
    for (addr = FLASH_START; addr + FLASH_ERASE_BLOCK - 1 <= FLASH_END; addr += FLASH_ERASE_BLOCK)

    {
        // Load full flash address into NVM address registers
        NVMADRL = addr & 0xFF;           // Low 8 bits
        NVMADRH = (addr >> 8) & 0x7F;    // Upper 7 bits for full 0x0000?0x7FFF

        // Program flash erase setup
        NVMCON1bits.NVMREGS = 0; // PFM (Program Flash Memory)
        NVMCON1bits.FREE    = 1; // Erase
        NVMCON1bits.WREN    = 1; // Enable writes

        // Disable interrupts during unlock sequence
        INTCONbits.GIE = 0;

        // Required unlock sequence
        NVMCON2 = 0x55;
        NVMCON2 = 0xAA;
        NVMCON1bits.WR = 1; // Start erase

        // Wait until erase completes
        while(NVMCON1bits.WR);

        // Re-enable interrupts
        INTCONbits.GIE = 1;

        // Disable writes
        NVMCON1bits.WREN = 0;
    }

    UART_TxString("<EndFlashErase>");
    __delay_ms(MSG_MS_DELAY);
}


//-------------------------------------------------------
// WAIT HANDSHAKE AND FIRWARE UPDATE ROUTINE
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
        if (PIR3bits.RCIF)                          
        {       
            temp[byteCount] = UART_Rx();            // Capture incoming byte
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
        // Send Acknowledge: Host sends next 8-byte packet after seeing this
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


// 0x55 and 0XAA handshake expected from Host to start firmware update
void WaitHandshake(void) {
    uint8_t prev = 0;
    uint8_t curr;
    uint32_t timeout_counter = 0;
    
    const uint32_t THREE_SECONDS = 1000000; 
    
     while (timeout_counter < THREE_SECONDS)
    {
        if(PIR3bits.RCIF)               // UART receive interrupt flag set (data received in RCREG)
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

