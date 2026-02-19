/*
 * File:   bootloader.c
 * Version: 2.01
 * Author: Issac
 *
 * Created on January 19, 2026, 2:50 PM
 */


#include <xc.h>
#include <stdint.h>                         // for standard integer types 
#include <stdbool.h>                        // for bool, true, false
#include "config.h"

// Note: B4J Expected bytes = 0xFFF - 0x600 = 0x9FF + 1 = 0xA00(2560) * 2 = 5120 BYTES (Each address is 1 WORD!))

#define FLASH_START 0x0600                  // Flash start address
#define FLASH_END 0x0FFF                    // Flash end address
#define FLASH_ERASE_BLOCK 32                // Runtime can only do 32 word block erase max!
#define FLASH_WRITE_BLOCK 4                 // Can only do 4 word block write max with PIC 16F88!              
#define TIMER2_COUNT 93                     // 32ms(ISR Trigger) x 93(t2_counter) = 3000 ms Timout
#define MSG_MS_DELAY 50                     // Delay for UART_TxString 

#define LED_PIN4 PORTBbits.RB4              // Bootloader Led Status    
#define LED_TRIS4 TRISBbits.TRISB4          // Output PortB.4 pin
#define LED_PIN3 PORTBbits.RB3              // Application LED
#define LED_TRIS3 TRISBbits.TRISB3

uint16_t flash_packet[FLASH_WRITE_BLOCK];   // 4 words, 8 bytes total
bool Timer2_Timeout = false;                // For Timer 2 Timeout Detection
uint8_t t2_counter = 0;                     // For Timer 2 ISR counter (32 ms trigger ISR function)


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

//-------------------------------------------------------
// UART ROUTINE
//-------------------------------------------------------
void UART_Init(void)
{
    // Configure pins
    TRISBbits.TRISB2 = 1;   // RX as input
    TRISBbits.TRISB5 = 0;   // TX as output

    // Baud rate calculation (Asynchronous, BRGH = 0)
    // Formula: Baud Rate = FOSC / (64 * (SPBRG + 1))
    // For FOSC = 8 MHz, desired baud = 9600:
    // SPBRG = (FOSC / (64 * Baud)) - 1
    // SPBRG = (8,000,000 / (64 * 9600)) - 1 ? 12
    // SPBRG = 12;                      // Set SPBRG for 9600 baud
    // TXSTAbits.BRGH  = 0;              // Low-speed baud
 
    // Baud rate calculation (Asynchronous, BRGH = 1, High-speed)
    // Formula: Baud Rate = FOSC / (16 * (SPBRG + 1))
    // For FOSC = 8 MHz, desired baud = 57600:
    // SPBRG = (FOSC / (16 * Baud)) - 1
    // SPBRG = (8,000,000 / (16 * 57600)) - 1 ? 8
    SPBRG = 8;              // Set SPBRG for 57600 baud
    TXSTAbits.BRGH  = 1;    // High-Speed baud

    // Serial port enable
    TXSTAbits.SYNC = 0;     // Asynchronous mode
    RCSTAbits.SPEN = 1;     // Enables UART pins (RB2/RB5)

    // Transmission enable
    TXSTAbits.TXEN = 1;     // Transmit enable
    RCSTAbits.CREN = 1;     // Continuous receive enable
    
    // Read all pending bytes
    uint8_t dummy;
    while (PIR1bits.RCIF) {
        dummy = RCREG;      // discard byte
    }
}

void UART_Tx(uint8_t d)
{
    while (!PIR1bits.TXIF);     // Wait until TX ready
    TXREG = d;                  // Send data
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
    if (RCSTAbits.OERR)         // If overrun error (receiver full, unread data lost)
    {
        RCSTAbits.CREN = 0;     // Reset continuous receive
        RCSTAbits.CREN = 1;     // Re-enable receive
    }

    return RCREG;               // Read received byte
}


//-------------------------------------------------------
// TIMER2 ROUTINE
//-------------------------------------------------------
void TIMER2_Init(void)
{
    /**
    * @brief Initializes Timer2 for a 32ms periodic interrupt.
    * Settings: Fosc = 8MHz, Prescaler 1:16, Postscaler 1:16, PR2 = 249.
    */
    
    T2CONbits.TMR2ON = 0;         // Ensure timer is OFF during configuration
    
    // Interrupt Configuration
    PIR1bits.TMR2IF = 0;          // Clear the interrupt flag to avoid immediate firing
    PIE1bits.TMR2IE = 1;          // Enable Timer2 interrupts (usually 1 if you want 32ms ISR)
    TMR2 = 0;                     // Reset counter to 0

    // Period Register
    // Formula: (PR2 + 1) * Tcy * Prescaler * Postscaler
    PR2 = 249;                    // Sets period to 32ms at 8MHz Fosc

    // Clock Control
    T2CONbits.T2CKPS = 0b10;      // Prescaler = 1:16 (Sets T2CKPS1=1, T2CKPS0=0)
    
    // Postscaler Configuration
    // 0b1111 = 1:16 Postscaler
    T2CONbits.TOUTPS3 = 1;
    T2CONbits.TOUTPS2 = 1;
    T2CONbits.TOUTPS1 = 1;
    T2CONbits.TOUTPS0 = 1;

    // Global Interrupt Enables
    INTCONbits.PEIE = 1;          // Enable Peripheral Interrupts
    INTCONbits.GIE  = 1;          // Enable Global Interrupts
}

void Timer2_Start(void)
{
   PIR1bits.TMR2IF = 0;        // clear pending interrupt FIRST
   t2_counter = 0;             // reset ISR counter
   TMR2 = 0;                   // reset File Register counter
   Timer2_Timeout = false;     // Clear flag boolean
   
   PIE1bits.TMR2IE = 1;        // enable Timer2 interrupt flag
   T2CONbits.TMR2ON = 1;       // start Timer2
}

void Timer2_Stop(void)
{
    t2_counter = 0;             // reset ISR counter
    TMR2 = 0;                   // reset counter
    Timer2_Timeout = false;     // Clear flag boolean
    
    PIE1bits.TMR2IE = 0;        // Disable Timer2 interrupt flag
    T2CONbits.TMR2ON = 0;       // Stop Timer2

}

void __interrupt() ISR(void)
{
    if (PIR1bits.TMR2IF)
    {
        PIR1bits.TMR2IF = 0;                        // Clear flag

        t2_counter++;                               // counter to compare
        if (t2_counter >= TIMER2_COUNT)      
        {
            t2_counter = 0;                         // Reset it
            Timer2_Timeout = true;                   // Set true for looper to detect flag
        }
    }
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
    
    // Timer Interrupt not needed.  B4J will receive continous data from Program Memory
    Timer2_Stop();                  

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
    // Timer Interrupt not needed.  
    Timer2_Stop();   
                
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
        
        EECON1bits.FREE = 0;       // Stop erase
        EECON1bits.WREN = 0;       // Stop write
        INTCONbits.GIE = 1;        // Enable interrupts
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
    /**
    * @brief Receives a block of data via UART and assembles it into 16-bit words.
    * @return true if a full packet was received, false if a Timer2 timeout occurred.
    * * Note: Expected data size is FLASH_WRITE_BLOCK * 2 (e.g., 8 bytes for 4 words).
    */
    
    // Buffer for raw incoming bytes (e.g., 8 bytes)
    uint8_t temp[FLASH_WRITE_BLOCK * 2]; 
    uint8_t byteCount = 0;      
     
    // --- Data Acquisition Phase ---
    while (byteCount < (FLASH_WRITE_BLOCK * 2)) 
    {    
        // Exit if the global Timer2_Timeout flag is set by the ISR
        if (Timer2_Timeout)
        {
            return false;
        }

        // Check if UART Receive Interrupt Flag is set (byte waiting in RCREG)
        if (PIR1bits.RCIF) 
        {
            temp[byteCount] = UART_Rx(); // Read byte and clear RCIF
            byteCount++;                 
        }
    }

    // --- Byte-to-Word Reassembly Phase ---
    // Converts Little-Endian stream (LSB first) into 16-bit words
    // Example: Host sends [0xAA, 0xBB] -> flash_packet[i] becomes 0xBBAA
    for (uint8_t i = 0; i < FLASH_WRITE_BLOCK; i++) 
    {
        /* * Assembly Logic:
         * 1. Take the second byte (MSB), shift it left by 8 bits
         * 2. OR it with the first byte (LSB)
         * 3. Result: A 16-bit Word ready for Flash writing
         */
        flash_packet[i] = ((uint16_t)temp[i*2 + 1] << 8) | temp[i*2];
    }

    return true; // Packet successfully reconstructed
}

void DoFirmwareUpdate(void)
{   
    /**
    * @brief Main execution loop for the over-the-air/serial firmware update.
    * Manages the handshake, flash writing, and timeout-based error recovery.
    */
    
    // Notify Host (B4J) that the MCU is ready to receive flash data
    UART_TxString("<StartFlashWrite>");     
    __delay_ms(MSG_MS_DELAY);
    
    uint16_t flashAddr = FLASH_START;       // Start pointer for program memory
    uint8_t timeoutCount = 0;               // Counter for consecutive failed attempts
        
    while (1)
    {
        // Preparation: Reset and start the watchdog timer (Timer2)
        Timer2_Stop();                      
        Timer2_Start();

        // Send Acknowledge: Host waits for this before sending the next 8-byte packet
        UART_TxString("<ACK>");    
    
        // Attempt to capture a full packet from the UART buffer
        if (ReceivePacket())                
        {
            // SUCCESS: Reset failure counter
            timeoutCount = 0;
                        
            // Commit the 4-word (8-byte) packet to the physical Flash memory
            Flash_WriteBlock(flashAddr, flash_packet);

            // Increment address pointer (PIC16F88 writes in 4-word blocks)
            flashAddr += FLASH_WRITE_BLOCK;     
             
            // Boundary Check: Have we filled the designated flash area?
            if (flashAddr + FLASH_WRITE_BLOCK - 1 > FLASH_END)
            {
                // Notify Host to switch from "Send Mode" to "Verify Mode"
                UART_TxString("<EndFlashWrite>");
                __delay_ms(MSG_MS_DELAY);  
                 
                // Read back flash and stream it to Host
                Verify_Flash();
                
                return; // Firmware update successful
            }
        }
        else // FAILURE: ReceivePacket returned false (Timer2 timed out)
        {
            timeoutCount++;
            Timer2_Stop();
            
            // Inform Host of the specific communication lapse
            UART_TxString("<ISR Timeout>");        
            __delay_ms(MSG_MS_DELAY);              
            
            // Critical Failure: Exit if we lose connection 3 times in a row
            if (timeoutCount >= 3)
            {
                UART_TxString("<ErrorTimeout>");    
                __delay_ms(MSG_MS_DELAY);
                
                return; // Abort update and return to main/bootloader menu
            } 
            
            // If < 3 timeouts, the loop continues and sends another <ACK>
        }
    }
}


void WaitHandshake(void) 
{ 
    /**
    * @brief Waits for a specific 16-bit sequence (0x55, 0xAA) via UART.
    * If detected within the Timer2 timeout period, it triggers the firmware update process.
    */
    
    uint8_t prev = 0; // Stores the previous byte for sequence matching
    uint8_t curr;     // Stores the current byte being read
    
    Timer2_Start();   // Initialize the watchdog timeout
    
    // Continue polling until Timer2_Timeout is set by the ISR
    while (!Timer2_Timeout)
    {
        // Check if UART has received data (Receive Interrupt Flag)
        if(PIR1bits.RCIF) 
        {
            curr = UART_Rx(); // Fetch byte from RCREG
                
            // Security Check: Look for the specific "Magic Sequence"
            // The Host must send 0x55 followed immediately by 0xAA
            if(prev == 0x55 && curr == 0xAA) 
            {                         
                // Step 1: Confirm handshake to Host (B4J)
                UART_TxString("<InitReceived>");
                __delay_ms(MSG_MS_DELAY);
                
                // Step 2: Clear old application code to make room for new firmware
                Flash_EraseApplication(); 
                
                // Step 3: Enter the main writing loop
                DoFirmwareUpdate();       
                
                return; // Exit after successful update
            }
            
            // Sliding Window: Move current byte to previous to check next pair
            prev = curr;
        }
    }
    
    // If the loop exits, it means Timer2 timed out without seeing 0x55 0xAA
    Timer2_Stop();
    
    // Notify the host that the connection window has closed
    UART_TxString("<HandShakeTimeout>");
    __delay_ms(MSG_MS_DELAY);
}


//-------------------------------------------------
// APPLICATION ROUTINE IN HERE
//-------------------------------------------------
// Each function must define address location.  If you do not specify location,
// it will add codes below 0x600 into bootloader section!
// YOU MUST adjust address location if more codes are added or compile error will result!
void __at(0x650)EEPROM_WriteByte(uint8_t address, uint8_t data) 
{
    // Not used just keeping a template of it!
    
    while (EECON1bits.WR);              // Wait until previous write finishes
    EEADR = address;                    // Address to write
    EEDATA = data;                      // Data to write
    EECON1bits.EEPGD = 0;               // Select DATA EEPROM memory
    EECON1bits.WREN = 1;                // Enable write
    
    // Required sequence to unlock
    INTCONbits.GIE = 0;                 // Disable interrupts
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;                  // Start write
    while (EECON1bits.WR);              // Wait until previous write finishes
    
    INTCONbits.GIE = 1;                 // Enable interrupts
    EECON1bits.WREN = 0;                // Disable write
}

void __at(0x600)Application(void) {
    // Add application code here......
    
    uint8_t b;                          // Application monitor 0x55 for bootloading
    LED_TRIS3 = 0;                       // Output
    UART_Init();                        // Init UART
    
    //EEPROM_WriteByte(0x00, 0x55);
            
    while(1) 
    {
        // Flash Led for application.  Use different pin then bootloader.
        LED_PIN3 = 1;                    // LED ON
        __delay_ms(500);                // Wait 500 ms
        LED_PIN3 = 0;                    // LED OFF
        __delay_ms(500);                // Wait 500 ms
            
        if (PIR1bits.RCIF)              // PIR1bits.RCIF = 1 ? at least one byte in RCREG
        {
            b = UART_Rx();
            if (b == 0x55)              // This is Handshake byte. In application 0xAA is not needed.  It will reboot then 0x55 and 0xAA will be detected
            {
                // Send to Host Handshake received at app location
                UART_TxString("<InitFromApp>");
                
                asm ("goto 0x0000");    // Restart to bootloader in preparation for flash
            } 
        }
         
    }
}


//-------------------------------------------------------
// MAIN ENTRY FOR BOOTLOADER
//-------------------------------------------------------
void main(void) {
    ANSEL  = 0x00;                  // Disable analog (VERY IMPORTANT)
    
    LED_TRIS4 = 0;                   // Output
    LED_PIN4  = 1;                   // LED On (bootloader led))
       
    INTOSC_Init();                  // Must set internal Oscillator
    TIMER2_Init();                  // Init Timer2
    UART_Init();                    // Init Hardware UART   
    
    WaitHandshake();                // wait for 0x55 0xAA (3s timeout then goto app))
    
    LED_PIN4  = 0;                   // LED Off (bootloader led))
    
    Application();              // If bootloader is not init from PC, then continue to application
    
    
    // Good news is when bootloader goes to 0x0600 and is invalid, causes PIC to reset and main repeated over and over till handshake and flash success!
}
