/*
 * File:   bootloader.c
 * Version: 4.02
 * Author: Issac
 * Created on January 19, 2026, 2:50 PM
 * Family: 18F27Q43
 * USE 1.29.481
 */

#include <xc.h>
#include <stdint.h>                         // for standard integer types 
#include <stdbool.h>                        // for bool, true, false
#include "config.h"
#include "uart.h"

// Note: B4J Expected bytes = 0x1FFFF - 0x00A00 = 0x1F600 = 128,512 BYTES!

#define FLASH_START         0x00A00         // Recommended for Q43 (Page aligned)
#define FLASH_END           0x1FFFF         // 128KB Flash limit
#define FLASH_ERASE_BLOCK   128             // Q43 erases in 128-word (256-byte) pages
#define FLASH_WRITE_BLOCK   128             // Q43 writes in 128-word (256-byte) pages

#define MSG_MS_DELAY 150                    // (min 150 for BT latency) Standard pacing delay 

#define LED_PIN   LATBbits.LATB4            // Use LAT for Output / Bootloader Led Status 
#define LED_TRIS  TRISBbits.TRISB4          // Output PortB.4 pin

uint16_t flash_packet[FLASH_WRITE_BLOCK];   // 128 words, 256 bytes total
bool isBLE = false;                         // BLE Detection uses different verify_flash process
uint16_t BLE_MTU_Size = 20;                 // BLE MTU Size (B4J sends the value from configuration function)

//-------------------------------------------------------
// INTERNAL OSCILLATOR CLK CONFIG
//-------------------------------------------------------
void INTOSC_Init(void)
{
    // 1. Set Nominal Frequency to 64 MHz
    // Q43 OSCFRQ: 1000 = 64 MHz
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
        // CHANGE: On Q43, the bit is OSCSTATbits.HFOR (HFINTOSC Oscillator Ready)
        while (!OSCSTATbits.HFOR); 
    #endif
}


//-------------------------------------------------------
// READ AND WRITE PROGRAM CODE ROUTINE + READ EEPROM DATA
//-------------------------------------------------------
// READ FLASH DATA (1 Word)
uint16_t Flash_ReadInstruction(uint32_t address)
{
    uint16_t word;
    uint8_t lowByte, highByte;
    
    // NVMCON1bits.NVMREG = 2; // REMOVED: Not required on Q43 for Table Reads

    // 1. Load the 24-bit address into Table Pointer registers
    TBLPTRU = (uint8_t)((address >> 16) & 0xFF); 
    TBLPTRH = (uint8_t)((address >> 8) & 0xFF);  
    TBLPTRL = (uint8_t)(address & 0xFF);         

    // 2. Perform Table Read and Post-Increment
    asm("TBLRD*+");      // Read low byte into TABLAT, increment pointer
    lowByte = TABLAT;    

    asm("TBLRD*+");      // Read high byte into TABLAT, increment pointer
    highByte = TABLAT;   

    // 3. Combine into a 16-bit word
    word = (uint16_t)(lowByte | ((uint16_t)highByte << 8));

    return word;
}

// The "Table Latch" Method (Legacy / Bridge)
void Flash_WriteBlock(uint32_t address, uint16_t *data)
{
    // 1. Align address to 128-word (256-byte) boundary for Q43
    address &= (uint32_t)(~0xFF);   

    // 2. Load the 24-bit Table Pointer to start of the Page Buffer
    // TBLPTR is used specifically by the TBLWT instruction to fill the buffer
    TBLPTRU = (uint8_t)((address >> 16) & 0xFF);
    TBLPTRH = (uint8_t)((address >> 8) & 0xFF);
    TBLPTRL = (uint8_t)(address & 0xFF);

    // 3. Fill the 256-byte Page Buffer
    // NOTE: FLASH_WRITE_BLOCK must be 128 for the Q43
    for (uint8_t i = 0; i < FLASH_WRITE_BLOCK; i++)
    {
        TABLAT = (uint8_t)(data[i] & 0xFF); 
        asm("TBLWT*+");   

        TABLAT = (uint8_t)(data[i] >> 8);   
        asm("TBLWT*+");   
    }

    // 4. Load the target Flash address into NVMADR
    // This is the actual destination in Flash memory
    NVMADRU = (uint8_t)((address >> 16) & 0xFF);
    NVMADRH = (uint8_t)((address >> 8) & 0xFF);
    NVMADRL = (uint8_t)(address & 0xFF);

    // 5. Configure NVMCON1 for "Write Page" command (0x05)
    NVMCON1bits.CMD = 0x05;            // Set the page write command

    // 6. Unlock sequence and start write
    INTCON0bits.GIE = 0;     
    
    // Q43 uses NVMLOCK instead of NVMCON2
    NVMLOCK = 0x55;          
    NVMLOCK = 0xAA;          
    
    // Start the operation using the GO bit
    NVMCON0bits.GO = 1;      

    // CPU stalls here until the 256-byte page is written
    while (NVMCON0bits.GO);  

    INTCON0bits.GIE = 1;     

    // 7. Cleanup: Clear command to NOP
    NVMCON1bits.CMD = 0x00;            // Disable writes to memory
}


//-------------------------------------------------------
// VERIFY FLASH DATA
//-------------------------------------------------------
void Flash_Verify(void)
{
    uint32_t addr;
    uint8_t ble_counter = 0;
    
    UART_TxString("<StartFlashVerify>");
    __delay_ms(MSG_MS_DELAY);

    // Q43 Caution: FLASH_ERASE_BLOCK is 128 words (256 bytes)
    // addr increases by the byte-count (128 * 2)
    //for (addr = FLASH_START; addr < FLASH_END; addr += (FLASH_ERASE_BLOCK * 2))
    for (addr = FLASH_START; addr + (FLASH_ERASE_BLOCK * 2) - 1 <= FLASH_END; addr += FLASH_ERASE_BLOCK * 2)
    {
        // On Q43, this buffer needs to be 128 words long
        uint16_t packet[FLASH_WRITE_BLOCK];

        for (uint8_t i = 0; i < FLASH_WRITE_BLOCK; i++)
        {
            packet[i] = Flash_ReadInstruction(addr + (uint32_t)(i * 2));
        }

        for (uint8_t i = 0; i < FLASH_WRITE_BLOCK; i++)
        {
            UART_Tx(packet[i] & 0xFF);   
            UART_Tx(packet[i] >> 8); 
            
            // BLE can only send MTU Limits usually 20 bytes per session
            if (isBLE) 
            {
                ble_counter += 2;
        
                // 2. Every 20 bytes, we must pause for the HM-10 radio
                if (ble_counter >= BLE_MTU_Size) 
                {
                    // Wait for the HM-10 to clear its internal UART-to-BLE buffer
                    __delay_ms(20);         // Important to keep it min 20 ms! BLE Cant handle less then that!
                    ble_counter = 0;
                }
            }
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
    // Q43: FLASH_ERASE_BLOCK is 128 (words), so 256 bytes per step
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
        // On Q43, we set the GO bit. The CPU will stall.
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
    // Q43 Warning: FLASH_WRITE_BLOCK is now 128 (words), so temp needs to be 256 bytes.
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
        // to fill the larger Q43 Page Buffer.
        if (ReceivePacket())  
        {
            timeoutCount = 0; 

            // 2. FLASH PROGRAMMING
            // Commits 128 words to the Q43 NVM Page.
            Flash_WriteBlock(flashAddr, flash_packet);

            // 3. ADDRESS POINTER ADVANCEMENT
            // On Q43: 128 words * 2 = 256 bytes.
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
    
    uint8_t temp[3];  
    uint16_t byteCount = 0;
    uint32_t timeoutCounter = 0;
    
    const uint32_t TIMEOUT_MAX = 2400000; 

        //-----FLush-----
    uint8_t dummy;
    while (PIR4bits.U1RXIF) {
        dummy = U1RXB;        
    }
    
    while (byteCount < 3)
    {
        if (PIR4bits.U1RXIF) 
        {
            temp[byteCount] = UART_Rx();  
            byteCount++;
            timeoutCounter = 0; 
        }
        else
        {
            timeoutCounter++;
            
            if (timeoutCounter > TIMEOUT_MAX)
            {
                UART_TxString("<ConfigTimeout>");
                __delay_ms(MSG_MS_DELAY);
                
                // CRITICAL: Do not proceed to erase/write flash if config failed
                return; 
            }
        }
    }

    // --- PROCESS CONFIG BYTES ---
    // Byte 0: BLE Toggle (1 = True, 0 = False)
    isBLE = (temp[0] != 0); 

    // Bytes 1 & 2: Set the MTU Size
    BLE_MTU_Size = ((uint16_t)temp[1] << 8) | temp[2];
    
    __delay_ms(MSG_MS_DELAY);
    UART_TxString("<ConfigOK>");
    __delay_ms(MSG_MS_DELAY);
                
    Flash_EraseApplication();  
    DoFirmwareUpdate();    
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
        // CHANGE: PIR3bits.U1RXIF moved to PIR6bits.U1RXIF on the Q43
        if (PIR4bits.U1RXIF) 
        {
            // Note: UART_Rx() must return U1RXB for the Q43
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
    
    asm("goto 0xA00");              // If bootloader is not init from PC, then continue to application
    
    
    // Good news is when bootloader goes to 0x0A00 and is invalid, causes pic to reset and main repeated over and over till handshake and flash success!
}
