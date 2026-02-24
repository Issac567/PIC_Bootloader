/*
 * File:   bootloader.c
 * Version: 3.01
 * Author: Issac
 * Created on January 19, 2026, 2:50 PM
 * Family: 18F27K42
 */


#include <xc.h>
#include <stdint.h>                         // for standard integer types 
#include <stdbool.h>                        // for bool, true, false
#include "config.h"
#include "uart.h"

// Note: B4J Expected bytes = 0x1FFFF - 0x00900 = 0x1F700 = 128,768 BYTES! (Each Address is 1 BYTE!)

#define FLASH_START 0x00900                 // Flash start address
#define FLASH_END 0x1FFFF                   // Flash end address
#define FLASH_ERASE_BLOCK 64                // Runtime can only do 64 Word erase max!
#define FLASH_WRITE_BLOCK 64                // Can only do 64 Word block write max with PIC 18F24K47! 
#define MSG_MS_DELAY 50                     // Delay for UART_TxString   

#define LED_PIN   LATBbits.LATB4            // Use LAT for Output / Bootloader Led Status 
#define LED_TRIS  TRISBbits.TRISB4          // Output PortB.4 pin

uint16_t flash_packet[FLASH_WRITE_BLOCK];   // 64words, 128 bytes total


//-------------------------------------------------------
// INTERNAL OSCILLATOR CLK CONFIG
//-------------------------------------------------------
void INTOSC_Init(void)
{
    // 1. Set Nominal Frequency to 64 MHz
    // For K42: 1000 = 64 MHz, 0110 = 32 MHz
    OSCFRQ = 0x08;  // HFFRQ = 1000 (64 MHz)

    // 2. Select HFINTOSC as system clock, NDIV = 1
    // OSCCON1: NOSC<6:4> = 110 (HFINTOSC), NDIV<3:0> = 0000 (1:1)
    OSCCON1 = 0b01100000; 

    // 3. Wait for the clock switch to complete and oscillator to be stable
    // --- THE "HANDSHAKE" (Wait for hardware to catch up) ---
    #ifndef __DEBUG
        while (OSCCON2 != OSCCON1);     // Wait for switch to complete
        while (!OSCSTATbits.HFOR);      // Wait for 64MHz to be stable
    #endif
}


//-------------------------------------------------------
// READ AND WRITE PROGRAM CODE ROUTINE + READ EEPROM DATA
//-------------------------------------------------------
// READ FLASH DATA (1 Word)
uint16_t Flash_ReadWord(uint32_t address)
{
    uint16_t word;
    uint8_t lowByte, highByte;
    
    NVMCON1bits.NVMREG = 2; // Point to Program Flash Memory (REG1=1, REG0=0)

    // 1. Load the 24-bit address into Table Pointer registers
    TBLPTRU = (uint8_t)((address >> 16) & 0xFF); // Upper byte
    TBLPTRH = (uint8_t)((address >> 8) & 0xFF);  // High byte
    TBLPTRL = (uint8_t)(address & 0xFF);         // Low byte

    // 2. Perform Table Read and Post-Increment
    asm("TBLRD*+");      // Read current byte into TABLAT, then increment pointer
    lowByte = TABLAT;    // Get low byte

    asm("TBLRD*+");      // Read next byte into TABLAT, then increment pointer
    highByte = TABLAT;   // Get high byte

    // 3. Combine into a 16-bit word
    word = (uint16_t)(lowByte | ((uint16_t)highByte << 8));

    return word;
}

void Flash_WriteBlock(uint32_t address, uint16_t *data)
{
    // 1. Align address to 64-word (128-byte) boundary
    //    PIC18F24K42 erases/writes in rows of 64 words (128 bytes)
    //    Clearing lower 7 bits ensures start of the row
    address &= (uint32_t)(~0x7F);   

    // 2. Load the 24-bit Table Pointer to the start of the row
    TBLPTRU = (uint8_t)((address >> 16) & 0xFF);
    TBLPTRH = (uint8_t)((address >> 8) & 0xFF);
    TBLPTRL = (uint8_t)(address & 0xFF);

    // 3. Fill the 128-byte latch buffer
    //    Each word is 2 bytes: low then high
    for (uint8_t i = 0; i < FLASH_WRITE_BLOCK; i++)
    {
        // Low byte first
        TABLAT = (uint8_t)(data[i] & 0xFF); 
        asm("TBLWT*+");   // Write to latch, post-increment TBLPTR

        // High byte
        TABLAT = (uint8_t)(data[i] >> 8);   
        asm("TBLWT*+");   // Write high byte to latch, post-increment TBLPTR
    }

    // 4. Reset Table Pointer to start of row
    //    Required for the actual flash write (latch -> PFM)
    TBLPTRU = (uint8_t)((address >> 16) & 0xFF);
    TBLPTRH = (uint8_t)((address >> 8) & 0xFF);
    TBLPTRL = (uint8_t)(address & 0xFF);

    // 5. Configure NVMCON1 for Program Flash write
    NVMCON1bits.NVMREG = 2;  // Select Program Flash Memory (PFM)
    NVMCON1bits.FREE   = 0;  // Normal write (not erase)
    NVMCON1bits.WREN   = 1;  // Enable write

    // 6. Unlock sequence and start write
    INTCON0bits.GIE = 0;     // Disable interrupts to prevent timing issues
    NVMCON2 = 0x55;          // Unlock step 1
    NVMCON2 = 0xAA;          // Unlock step 2
    NVMCON1bits.WR = 1;      // Start write (CPU stalls until done)

    while (NVMCON1bits.WR);  // Wait for write to complete

    INTCON0bits.GIE = 1;     // Re-enable interrupts

    // 7. Cleanup: disable writes
    NVMCON1bits.WREN = 0;
}


//-------------------------------------------------------
// VERIFY FLASH DATA
//-------------------------------------------------------
void Verify_Flash(void)
{
    uint32_t addr;

    // Marker sent to host
    UART_TxString("<StartFlashVerify>");
    __delay_ms(MSG_MS_DELAY);

    // Walk flash in erase/write-sized blocks
    //for (addr = FLASH_START; addr < FLASH_END; addr += FLASH_ERASE_BLOCK * 2)
    for (addr = FLASH_START; addr + (FLASH_ERASE_BLOCK * 2) - 1 <= FLASH_END; addr += FLASH_ERASE_BLOCK * 2)
    {
        // Buffer holds FLASH_WRITE_BLOCK words
        uint16_t packet[FLASH_WRITE_BLOCK];

        // Read flash block into buffer (word-aligned)
        for (uint8_t i = 0; i < FLASH_WRITE_BLOCK; i++)
        {
            // Each word occupies two byte addresses
            packet[i] = Flash_ReadWord(addr + (uint32_t)(i * 2));
        }

        // Transmit block to host: LSB first, then MSB
        for (uint8_t i = 0; i < FLASH_WRITE_BLOCK; i++)
        {
            UART_Tx(packet[i] & 0xFF);   // Low byte
            UART_Tx(packet[i] >> 8);     // High byte
        }

        __delay_ms(1);  // Proven safe pacing for UART/host
    }

    // Ensure last packet fully drains before end marker
    __delay_ms(MSG_MS_DELAY);
    UART_TxString("<EndFlashVerify>");
    __delay_ms(MSG_MS_DELAY);
}


//-------------------------------------------------------
// ERASE FLASH PROGRAM CODE DATA
//-------------------------------------------------------
void Flash_EraseApplication(void)
{
    // Visual/Serial feedback before entering the time-critical erase loop
    UART_TxString("<StartFlashErase>");
    __delay_ms(MSG_MS_DELAY);
    
    // Initialize Non-Volatile Memory Control Register 1 to a known safe state
    NVMCON1 = 0x00;         

    uint32_t addr;          // 24-bit byte address pointer for Program Flash Memory (PFM)
    
    // Iterate through Flash memory in physical Page increments. 
    // The PIC18F27K42 erases in 128-byte blocks; we step by (BLOCK * 2) for byte-addressing.
    //for (addr = FLASH_START; addr < FLASH_END; addr += FLASH_ERASE_BLOCK * 2)
    for (addr = FLASH_START; addr + (FLASH_ERASE_BLOCK * 2) - 1 <= FLASH_END; addr += FLASH_ERASE_BLOCK * 2)
    {
        // --- ADDRESS LOADING ---
        // Load the 24-bit target address into the Table Pointer registers.
        // TBLPTR registers are used by the NVM controller to latch the erase target.
        TBLPTRU = (uint8_t)((addr >> 16) & 0xFF); // Upper 6 bits (up to 0x3F for 128KB)
        TBLPTRH = (uint8_t)((addr >> 8) & 0xFF);  // High byte
        TBLPTRL = (uint8_t)(addr & 0xFF);         // Low byte (lower bits ignored for page alignment)

        // --- NVM CONFIGURATION ---
        // Select the Program Flash Memory (PFM) region for access
        NVMCON1bits.NVMREG = 2;  
        // Enable 'FREE' bit to specify an Erase operation rather than a Write
        NVMCON1bits.FREE   = 1;  
        // Set Write Enable (WREN) as a safety gate to allow PFM modification
        NVMCON1bits.WREN   = 1;  

        // --- CRITICAL SECTION ---
        // Globally disable interrupts to prevent a context switch during the unlock sequence.
        // The hardware unlock (55h/AAh) must occur in consecutive instruction cycles.
        INTCON0bits.GIE = 0;     

        // --- HARDWARE UNLOCK SEQUENCE ---
        // Mandatory safety sequence to prevent accidental Flash corruption.
        NVMCON2 = 0x55;
        NVMCON2 = 0xAA;

        // --- EXECUTION ---
        // Setting the WR bit initiates the internal high-voltage erase state machine.
        // On PIC18F K42, the CPU physically stalls until the erase cycle completes.
        NVMCON1bits.WR = 1;

        // Wait for the hardware to clear the WR bit, signaling the end of the erase cycle.
        // If the CPU is not stalled, this loop ensures synchronization.
        while(NVMCON1bits.WR);       
        
        // Restore Global Interrupt Enable bit to previous state
        INTCON0bits.GIE = 1;     

        // --- POST-OPERATION CLEANUP ---
        // Disable WREN to protect memory from unintended writes during normal execution.
        NVMCON1bits.WREN = 0;
        NVMCON1bits.FREE = 0;
    }

    // Indicate completion of the full application region erase
    UART_TxString("<EndFlashErase>");
    __delay_ms(MSG_MS_DELAY);
}


//-------------------------------------------------------
// WAIT HANDSHAKE AND FIRWARE UPDATE ROUTINE
//-------------------------------------------------------
bool ReceivePacket(void)
{
    uint8_t temp[FLASH_WRITE_BLOCK * 2];  
    uint16_t byteCount = 0;
    uint32_t timeoutCounter = 0;
    const uint32_t TIMEOUT_3SEC = 2400000; 

    while (byteCount < FLASH_WRITE_BLOCK * 2)
    {
        // Check for Data
        if (PIR3bits.U1RXIF) 
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

    // Reconstruct 16-bit words for Flash
    for (uint16_t i = 0; i < FLASH_WRITE_BLOCK; i++)
    {
        flash_packet[i] = ((uint16_t)temp[i*2 + 1] << 8) | temp[i*2];
    }

    return true; 
}

// When ready will use this in future
void DoFirmwareUpdate(void)
{   
    // Notify the Host (B4J tool) that the PIC is ready to receive the binary stream
    UART_TxString("<StartFlashWrite>");
    __delay_ms(MSG_MS_DELAY);

    // Initialize the starting pointer for the Application region
    uint32_t flashAddr = FLASH_START;
    uint8_t timeoutCount = 0;

    // --- MAIN UPDATE LOOP ---
    // This loop runs until the entire memory is filled or a fatal error occurs
    while (1)
    {
        UART_TxString("<ACK>"); 
            
        // 1. DATA ACQUISITION
        // ReceivePacket waits for exactly 128 bytes (64 words) to fill the RAM buffer
        if (ReceivePacket())  
        {
            timeoutCount = 0;  // Successful reception; reset error counter

            // 2. FLASH PROGRAMMING
            // Commits the current 'flash_packet' (64 words) to the NVM latches.
            // Note: On K42, the CPU stalls during the physical write cycle.
            Flash_WriteBlock(flashAddr, flash_packet);

            // 3. ADDRESS POINTER ADVANCEMENT
            // Move the address forward by the number of bytes written (Words * 2)
            flashAddr += FLASH_WRITE_BLOCK * 2;  

            // 4. BOUNDARY CHECK
            // If the current address reaches or exceeds the end of the allocated PFM region, 
            // the update is complete.
            //if (flashAddr >= FLASH_END)
            if (flashAddr + (FLASH_WRITE_BLOCK * 2) - 1 > FLASH_END)
            {
                UART_TxString("<EndFlashWrite>");
                __delay_ms(MSG_MS_DELAY);

                // Perform a CRC or checksum check to ensure data integrity
                Verify_Flash();
                return;
            }
        }
        // 5. ERROR / TIMEOUT HANDLING
        else 
        {
            timeoutCount++;

            // Allow up to 3 retry attempts before declaring a failed update
            if (timeoutCount >= 3)
            {
                UART_TxString("<ErrorTimeout>");
                __delay_ms(MSG_MS_DELAY);
                return;
            }
        }
    }
}


// 0x55 and 0XAA handshake expected from Host to start firmware update
void WaitHandshake(void) 
{
    uint8_t prev = 0;
    uint8_t curr = 0;
    uint32_t handshakeCounter = 0;
    const uint32_t TIMEOUT_3SEC = 2400000; 

    // --- LISTENING LOOP ---
    // Instead of checking Timer2_Timeout, we check our software counter
    while (handshakeCounter < TIMEOUT_3SEC)
    {
        // 1. Check for UART data
        if (PIR3bits.U1RXIF) 
        {
            curr = UART_Rx();     
            
            // PATTERN MATCHING: 0x55 then 0xAA
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
            // 2. Increment counter when no data is present
            handshakeCounter++;
        }
    }
    
    // --- EXIT LOGIC ---
    UART_TxString("<HandShakeTimeout>");
    __delay_ms(MSG_MS_DELAY);
}


//-------------------------------------------------------
// MAIN ENTRY FOR BOOTLOADER
//-------------------------------------------------------
void main(void) 
{        
    ANSELA = 0x00;                  // Port A all digital
    ANSELB = 0x00;                  // Port B all digital
    ANSELC = 0x00;                  // Port C all digital

    LED_TRIS = 0;                   // LED Output
    LED_PIN  = 1;                   // LED On (bootloader led))
       
    INTOSC_Init();                  // Must set internal Oscillator
    UART_Init();                    // Init Hardware UART   
     
    WaitHandshake();                // wait for 0x55 0xAA (3s timeout then goto app))

    LED_PIN  = 0;                   // LED Off (bootloader led))
    
    asm("goto 0x900");              // If bootloader is not init from PC, then continue to application
    
    
    // Good news is when bootloader goes to 0x0900 and is invalid, causes pic to reset and main repeated over and over till handshake and flash success!
}
