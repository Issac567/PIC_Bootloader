/*
 * File:   bootloader.c
 * Version: 4.50
 * Created on January 19, 2026, 2:50 PM
 * Family: 16F18857
 * PACKS: USE 1.29.444
 */


#include <xc.h>
#include <stdint.h>                         // for standard integer types 
#include <stdbool.h>                        // for bool, true, false
#include "bootloader.h"
#include "config.h"
#include "uart.h" 

// Note: B4J Expected bytes = 0x7FFF - 0x700 = 0x78FF + 1 = 0x7A00(30,976) * 2 = 61,952 BYTES (Each address is 1 WORD!))
// @0x600 remaining less then 20 address code space available.  

#define FLASH_START 0x0700                  // Flash start address
#define FLASH_END 0x7FFF                    // Flash end address for 4-word block
#define FLASH_ERASE_BLOCK 32                // Runtime can only do 32 word block erase max!
#define FLASH_WRITE_BLOCK 32                // Can only do 32 word block write max with PIC 16F18857!
#define MSG_MS_DELAY 150                    // (min 150 for BT latency) Standard pacing delay 

#define LED_PIN   LATBbits.LATB4            // Use LAT for Output / Bootloader Led Status 
#define LED_TRIS  TRISBbits.TRISB4          // Output PortB.4 pin

#define DEVICE_BLE 0x01
#define DEVICE_CLASSIC_BT 0x02
#define DEVICE_WIFI 0x03
#define DEVICE_TTLSERIAL 0x04
            
uint16_t flash_packet[FLASH_WRITE_BLOCK];   // 32 words, 64 bytes total

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
    
    #ifndef __DEBUG
        //while(OSCCON3bits.ORDY == 0);
    #endif
}

void __interrupt() boot_ISR(void)
{
    asm("PAGESEL 0x7000");
    asm("CALL 0x7000");
}


//-------------------------------------------------------
// READ AND WRITE PROGRAM CODE ROUTINE + READ EEPROM DATA
//-------------------------------------------------------
// READ FLASH DATA (1 Word)
uint16_t Flash_ReadInstruction(uint16_t address)
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
    INTCONbits.GIE = 0;         // Disable interrupts
    NVMCON2 = 0x55;             // Unlock part 1
    NVMCON2 = 0xAA;             // Unlock part 2
    NVMCON1bits.WR = 1;         // Start row write
    while (NVMCON1bits.WR);     // Wait for completion
    INTCONbits.GIE = 1;         // Re-enable interrupts

    // -----------------------------
    // Cleanup: disable flash writes
    // -----------------------------
    NVMCON1bits.WREN = 0;   // Prevent accidental writes
}


//-------------------------------------------------------
// VERIFY FLASH DATA
//-------------------------------------------------------
// Verify Flash is performed after Flash Write is completed
void Flash_Verify(void)
{
    uint16_t addr;
    uint8_t ble_counter = 0;
    uint8_t b;  
    uint8_t totalChecksum = 0;
     
    // Send to host
    UART_TxString("<StartFlashVerify>");
    __delay_ms(MSG_MS_DELAY);
    
    // Loop through all flash from start to end
    for (addr = FLASH_START; addr + FLASH_WRITE_BLOCK - 1 <= FLASH_END; addr += FLASH_WRITE_BLOCK)
    {
        // Check if data is available in the hardware UART buffer for cancel operation
        if (PIR3bits.RCIF)
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
        
        // Prepare 32-word packet
        uint16_t packet[FLASH_WRITE_BLOCK];
        for (uint8_t i = 0; i < FLASH_WRITE_BLOCK; i++)
        {
            packet[i] = Flash_ReadInstruction(addr + i);  // Read word from flash
            totalChecksum += (packet[i] & 0xFF);
            totalChecksum += (packet[i] >> 8);
        }
        
        // Byte for Byte comparison
        if (flashSettings.isVerify_Checksum == false)
        {
            // Send packet to B4J 
            for (uint8_t i = 0; i < FLASH_WRITE_BLOCK; i++)
            {
                // eg. 0x3FFF = FF first then 3F second (B4J binary is backwards!)
                UART_Tx(packet[i] & 0xFF);              // First Byte (LSB)
                UART_Tx(packet[i] >> 8);                // Second Byte (MSB) Shift upper to lower
                // BLE can only send MTU Limits usually 20 bytes per session
                /**/
                if (flashSettings.WhichDevice == DEVICE_BLE) 
                {
                    ble_counter += 2;

                    // 2. Every xx bytes, we must pause for the HM-10 radio
                    if (ble_counter >= flashSettings.BLE_MTU_Size) 
                    {
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
            // WIFI DT06 requires higher delay or will cause issues!
            if (flashSettings.WhichDevice == DEVICE_WIFI) 
            {
                __delay_ms(100);     // tested min 100ms ok.  Disappointing! Maybe improve the custom DT06 rom?   
            } else {
                __delay_ms(1);
            }  
        // No Delay for checksum result.
        }
    }
    
    // Checksum comparison only
    if (flashSettings.isVerify_Checksum == true)
    {
        UART_Tx(totalChecksum);
        __delay_ms(MSG_MS_DELAY); 
    }

    // Send to host
    __delay_ms(MSG_MS_DELAY);                       // Must do this delay first. helps alot.  does not interfere with last packet sent!
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
        NVMADRL = addr & 0xFF;              // Low 8 bits
        NVMADRH = (addr >> 8) & 0x7F;       // Upper 7 bits for full 0x0000?0x7FFF

        // Program flash erase setup
        NVMCON1bits.NVMREGS = 0;            // PFM (Program Flash Memory)
        NVMCON1bits.FREE    = 1;            // Erase
        NVMCON1bits.WREN    = 1;            // Enable writes

        // Disable interrupts during unlock sequence
        INTCONbits.GIE = 0;

        // Required unlock sequence
        NVMCON2 = 0x55;
        NVMCON2 = 0xAA;
        NVMCON1bits.WR = 1;                 // Start erase

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
        if (PIR3bits.RCIF)                          
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
                 
                Flash_Verify();
                
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


void ReceiveConfig(void)
{
    // First Byte = 0x01 = BLE, 0x00 <> BLE
    // Second and Third Byte = MTU Size
    // Fourth Byte = 0x00 = Flash and Verify (Byte for Byte): 0x01 = Flash and Verify Checksum))
    // Fourth byte = In future! 0x02 = Verify (Byte for Byte only): 0x03 = Verify Checksum only))
    
    uint8_t temp[4];  
    uint16_t byteCount = 0;
    uint32_t timeoutCounter = 0;
    
    const uint32_t TIMEOUT_MAX = 1000000; 
    
   // ----- FLush -----
    uint8_t dummy;
    while (PIR3bits.RCIF) 
    {
        dummy = RC1REG;         // discard
    }
    
    // Wait for all 4 configuration bytes from HOST.
    while (byteCount < 4)
    {
        if(PIR3bits.RCIF)  
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
                
            // Check for BLE, SSP, TTL USB and WIFI uses this!
            if (prev == 0x55 && curr == 0xAA)
            {         
                // Send initialization acknowledgment before starting Erase and Flash update
                UART_TxString("<InitReceived>");
                __delay_ms(MSG_MS_DELAY);
                
                ReceiveConfig();
                
                return;         
            }              
            // sliding window byte comparison
            prev = curr;
            //handshakeCounter = 0; // Optional: reset timeout if we see traffic
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
    
    asm("PAGESEL 0x700");           // Ensure PCLATH is correct for the jump
    asm("goto 0x700");              // If bootloader is not init from PC, then continue to application
    
    
    // Good news is when bootloader goes to 0x0700 and is invalid, causes pic to reset and main repeated over and over till handshake and flash success!
}
