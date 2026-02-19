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

// Note: B4J Expected bytes = 0x1FFFF - 0x00900 = 0x1F700 = 128,768 BYTES! (Each Address is 1 BYTE!)

#define FLASH_START 0x00900                 // Flash start address
#define FLASH_END 0x1FFFF                   // Flash end address
#define FLASH_ERASE_BLOCK 64                // Runtime can only do 64 Word erase max!
#define FLASH_WRITE_BLOCK 64                // Can only do 64 Word block write max with PIC 18F24K47!
#define TIMER2_COUNT 186                    // 3s (Use this!! better result!)  
#define MSG_MS_DELAY 50                     // Delay for UART_TxString   

#define LED_PIN4   LATBbits.LATB4           // Use LAT for Output / Bootloader Led Status 
#define LED_TRIS4  TRISBbits.TRISB4         // Output PortB.4 pin
#define LED_PIN3   LATBbits.LATB3           // Use LAT for Output
#define LED_TRIS3  TRISBbits.TRISB3

uint16_t flash_packet[FLASH_WRITE_BLOCK];   // 64words, 128 bytes total
bool Timer2_Timout = false;                 // For Timer 2 Timeout Detection
uint8_t t2_counter = 0;                     // For Timer 2 ISR counter (16 ms trigger ISR function)

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
// UART ROUTINE
//-------------------------------------------------------
void UART_Init(void)
{
    // 1. Set pin directions
    TRISBbits.TRISB2 = 1; // RB2 as Input (RX)
    TRISBbits.TRISB5 = 0; // RB5 as Output (TX)

    // 2. PPS Unlock Sequence (Critical for PIC18)
    // We disable interrupts briefly to ensure the 0x55/0xAA timing is perfect
    GIE = 0;
    
    asm("MOVLW 0x55");
    asm("MOVWF PPSLOCK");
    asm("MOVLW 0xAA");
    asm("MOVWF PPSLOCK");
    PPSLOCKbits.PPSLOCKED = 0; // Unlock PPS

    // 3. Map RX to RB2
    // On K42, RB2 input code is 0x0A
    U1RXPPS = 0x0A; 

    // 4. Map RB5 to TX
    // On K42, U1TX output function code is 0x13
    RB5PPS = 0x13; 

    // 5. PPS Lock Sequence
    asm("MOVLW 0x55");
    asm("MOVWF PPSLOCK");
    asm("MOVLW 0xAA");
    asm("MOVWF PPSLOCK");
    PPSLOCKbits.PPSLOCKED = 1; // Lock PPS
    
    GIE = 1; // Restore interrupt state

    // 6. Baud rate (57600 @ 64MHz HFINTOSC)
    // Formula: Baud = Fosc / (4 * (n + 1))
    // n = (64,000,000 / (4 * 57600)) - 1 = 276.7 -> 277 (0x0115)
    U1BRGL = 0x15; 
    U1BRGH = 0x01; 

    // 7. UART Configuration
    U1CON0bits.U1BRGS = 1;      // High-speed (4 clocks per bit)
    U1CON0bits.U1MODE = 0b0000; // Asynchronous 8-bit mode
    
    // 8. Enable UART and Pins
    U1CON1bits.U1ON = 1;        // Serial port enabled
    U1CON0bits.U1TXEN = 1;      // Transmit enabled
    U1CON0bits.U1RXEN = 1;      // Receive enabled

    // 9. Clear pending RX bytes
    uint8_t dummy;
    while (PIR3bits.U1RXIF) {
        dummy = U1RXB;          // Discard any data in the FIFO
    }
}

void UART_Tx(uint8_t d)
{
    // Wait until the Transmit Buffer is empty
    // On K42, the flag is U1TXIF in the PIR3 register
    while (!PIR3bits.U1TXIF);  

    // Load the data into the Transmit Buffer
    // TX1REG is replaced by U1TXB
    U1TXB = d;                 
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
    // 1. Check for errors in the U1ERRIR (UART1 Error Interrupt Flag) register
    // OERR is now U1RXFOIF (Receive FIFO Overflow Interrupt Flag)
    if (U1ERRIRbits.U1RXFOIF)
    {
        // To clear an overflow on the K42, you simply clear the flag bit.
        // There's no need to toggle CREN (U1RXEN).
        U1ERRIRbits.U1RXFOIF = 0; 
    }

    // 2. Wait for data to be available in the FIFO
    // We check the U1RXIF flag in PIR3
    //while (!PIR3bits.U1RXIF);

    // 3. Return the byte from the buffer
    // RC1REG is now U1RXB
    return U1RXB;
}


//-------------------------------------------------------
// TIMER2 ROUTINE
//-------------------------------------------------------
void TIMER2_Init(void) {
    // 1. Set Clock Source to Fosc/4 (0001)
    // On K42, this remains T2CLKCON
    T2CLKCONbits.CS = 0b0001; 

    // 2. Hardware Limit Timer - Standard Timer Mode
    // T2HLT controls the "mode". 0x00 is software control / standard.
    T2HLT = 0x00; 

    // 3. Configuration
    // Prescaler 1:128 (111), Postscaler 1:8 (0111)
    T2CONbits.CKPS = 0b111;
    T2CONbits.OUTPS = 0b0111;
    
    T2PR = 249;   // Period match
    T2TMR = 0;    // Reset count

    // 4. Clear Flag and Enable Interrupts
    // On K42, Timer 2 IF/IE are in PIR4/PIE4. 
    // Note: The bit name is TMR2IF, same as before, but verify register mapping.
    PIR4bits.TMR2IF = 0;
    PIE4bits.TMR2IE = 0; 

    // 5. Global Interrupts
    // On PIC18, we usually use INTCON0
    INTCON0bits.GIEL = 1; // Low priority (or Peripheral) enable
    INTCON0bits.GIEH = 1; // High priority (or Global) enable 
}

void Timer2_Start(void)
{
    Timer2_Timout = false;     // Clear timeout flag
    t2_counter = 0;            // Reset ISR counter
    T2TMR = 0;                 // Clear timer                
 
    PIR4bits.TMR2IF = 0;       // Clear pending interrupt
    PIE4bits.TMR2IE = 1;       // Enable Timer2 interrupt
    
    // On K42, the bit name is 'ON', not 'TMR2ON'
    T2CONbits.ON = 1;          // Start Timer2
}

void Timer2_Stop(void)
{
    Timer2_Timout = false;      // Clear flag boolean
    t2_counter = 0;             // Reset ISR counter
    T2TMR = 0;                  // Clear timer  

    PIE4bits.TMR2IE = 0;        // Disable Timer2 interrupt
    
    // On K42, the bit name is 'ON', not 'TMR2ON'
    T2CONbits.ON = 0;           // Stop Timer2             
}

void __interrupt(high_priority) ISR(void)
{
    // Best practice: Always check both the Flag (IF) AND the Enable (IE) bit
    // This prevents the ISR from executing code for a disabled peripheral
    if (PIE4bits.TMR2IE && PIR4bits.TMR2IF)
    {
        PIR4bits.TMR2IF = 0;                        // Clear flag

        t2_counter++;                               
        
        // --- IMPORTANT: Timing Change ---
        // At 64MHz, Timer2 ticks twice as fast as at 32MHz.
        // If TIMER2_COUNT was 93 for 3s at 32MHz, 
        // it now needs to be 186 for 3s at 64MHz.
        
        if (t2_counter >= TIMER2_COUNT)      
        {
            t2_counter = 0;                         // Reset it
            Timer2_Timout = true;                   // Set true for looper
        }
    }
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

    // Timer not needed. B4J will receive continuous blocks with pacing delays
    Timer2_Stop();

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
    // Ensure deterministic timing by stopping peripheral timers that could trigger interrupts
    Timer2_Stop(); 

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
    // Local buffer to store the raw byte stream (128 bytes for a 64-word block)
    uint8_t temp[FLASH_WRITE_BLOCK * 2];  
    uint16_t byteCount = 0;

    // Handshake: Signal the host (B4J Uploader) that the PIC is ready for the next stream
    UART_TxString("<ACK>"); 

    // Blocking loop: Continues until the buffer is full or a timeout occurs
    while (byteCount < FLASH_WRITE_BLOCK * 2)
    {
        // --- WATCHDOG TIMEOUT ---
        // Prevents the bootloader from hanging forever if the PC stops sending data.
        // Timer2 acts as a communication watchdog.
        if (Timer2_Timout)
        {
            UART_TxString("<ISR Timeout>");
            __delay_ms(MSG_MS_DELAY);
            return false;
        }

        // --- DATA RETRIEVAL ---
        // PIR3bits.U1RXIF is high as long as there is at least one unread byte in the FIFO.
        if (PIR3bits.U1RXIF) 
        {
            // Pull byte from the hardware FIFO into our RAM array
            temp[byteCount] = UART_Rx();  
            byteCount++;

            // Heartbeat: Reset the timeout timer every time a valid byte is received.
            // This ensures we only timeout if the 'gap' between bytes is too large.
            Timer2_Stop();
            Timer2_Start();
        }
    }

    // --- DATA RECONSTRUCTION ---
    // Intel HEX and most serial protocols send data byte-by-byte.
    // This loop reconstructs 16-bit words from the 8-bit stream.
    for (uint16_t i = 0; i < FLASH_WRITE_BLOCK; i++)
    {
        // Little-Endian Assembly: Host sends Low Byte [i*2] then High Byte [i*2 + 1].
        // This matches the PIC18 internal word structure for direct Flash writing.
        flash_packet[i] = ((uint16_t)temp[i*2 + 1] << 8) | temp[i*2];
    }

    return true; // Successfully filled the packet
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
        // Reset the communication watchdog timer before attempting to receive a packet
        Timer2_Stop();
        Timer2_Start();

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
            Timer2_Stop();

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
    
    // Initialize the communication watchdog timer.
    // This defines the window of time the user has to initiate an update after reset.
    Timer2_Start(); 
    
    // --- LISTENING LOOP ---
    // Stay in this loop until the Timer2 timeout flag is set by hardware.
    while (!Timer2_Timout)
    {
        // Check the UART1 Receive Interrupt Flag (RXIF).
        // This bit is set by hardware when the FIFO contains at least one byte.
        if (PIR3bits.U1RXIF) 
        {
            // Pull the oldest byte from the 4-level deep hardware FIFO.
            curr = UART_Rx();     
            
            // --- PATTERN MATCHING (SLIDING WINDOW) ---
            // The bootloader looks for the signature 0x55 followed by 0xAA.
            // Using a specific sequence prevents noise on the RX line from accidentally
            // triggering a full Flash Erase.
            if (prev == 0x55 && curr == 0xAA) 
            {                               
                // Acknowledge the connection to the B4J Host.
                UART_TxString("<InitReceived>");
                __delay_ms(MSG_MS_DELAY);
                
                // --- CRITICAL FLASH OPERATIONS ---
                // Step 1: Wipe the existing application region (PFM).
                Flash_EraseApplication();  
                
                // Step 2: Begin the multi-packet write process.
                DoFirmwareUpdate();        
                
                return; // Exit bootloader mode upon successful update completion.
            }
            
            // Update the window: the current byte becomes the previous byte for the next iteration.
            prev = curr;
        }
    }
    
    // --- EXIT LOGIC ---
    // If the loop exits, it means no valid handshake was received within the time limit.
    Timer2_Stop();
    
    // Notify the host that the bootloader is timing out and will now hand over control.
    UART_TxString("<HandShakeTimeout>");
    __delay_ms(MSG_MS_DELAY);
}


//-------------------------------------------------
// APPLICATION ROUTINE IN HERE
//-------------------------------------------------
// Each function must define address location.  If you do not specify location,
// it will add codes below 0x900 into bootloader section!
// YOU MUST adjust address location if more codes are added or compile error will result!
void __at(0x970)EEPROM_WriteByte(uint16_t address, uint8_t data)
{
    NVMCON1 = 0x00;                 // Setup for EEPROM (NVMREG = 00)
    
    // Address Setup
    NVMADRL = (uint8_t)(address & 0xFF);
    NVMADRH = (uint8_t)((address >> 8) & 0x03);

    // Data Setup
    NVMDAT = data;                  // Use NVMDAT if NVMDATL is not found

    NVMCON1bits.WREN = 1;           // Enable writes
    
    INTCON0bits.GIE = 0;            // Disable interrupts

    // Unlock Sequence
    NVMCON2 = 0x55;
    NVMCON2 = 0xAA;
    NVMCON1bits.WR = 1;             // Start Write

    while(NVMCON1bits.WR);          // Wait for hardware to clear WR bit

    INTCON0bits.GIE = 1;            // enable interrupts
    NVMCON1bits.WREN = 0;           // Disable writes for safety
}

void __at(0x900) Application(void) 
{
    uint8_t b;                          // Variable to hold the received handshake byte
    
    //EEPROM_WriteByte(0x00, 0x55);
    
    // --- PERIPHERAL INITIALIZATION ---
    // Configure the LED pin as an output. 
    // Ensure LED_TRIS is mapped to a different pin than the bootloader's status LED.
    LED_TRIS3 = 0;                       

    // --- MAIN APPLICATION LOOP ---
    while(1) 
    {  
        // 1. VISUAL HEARTBEAT
        // Standard blink pattern to confirm the application is currently running.
        LED_PIN3 = 1;                    // LED ON
        __delay_ms(500);                // Wait 500 ms
        LED_PIN3 = 0;                    // LED OFF
        __delay_ms(500);                // Wait 500 ms
                
        // 2. BOOTLOADER "SOFT-TRIGGER" MONITOR
        // The application constantly monitors the UART for a specific '0x55' trigger.
        // This allows the B4J tool to request a firmware update while the app is active.
        if (PIR3bits.U1RXIF)            // Check if at least one byte is in the RX FIFO
        {
            b = UART_Rx();
            
            // If the handshake byte (0x55) is detected:
            if (b == 0x55)              
            {
                // Acknowledge the request so the PC knows the app has "heard" the command.
                UART_TxString("<InitFromApp>");
                
                // 3. SOFTWARE RESET
                // Force a jump to address 0x0000. 
                // This executes the hardware reset vector where the Bootloader resides.
                // Upon restart, the Bootloader's 'WaitHandshake' will detect the full 55/AA sequence.
                asm ("goto 0x0000");  
            } 
        }
    }
}


//-------------------------------------------------------
// MAIN ENTRY FOR BOOTLOADER
//-------------------------------------------------------
//void __at(0x20)main(void) 
void main(void)
{    
    ANSELA = 0x00;                  // Port A all digital
    ANSELB = 0x00;                  // Port B all digital
    ANSELC = 0x00;                  // Port C all digital

    LED_TRIS4 = 0;                  // LED Output
    LED_PIN4  = 1;                  // LED On (bootloader led))
       
    INTOSC_Init();                  // Must set internal Oscillator
    TIMER2_Init();                  // Init Timer2
    UART_Init();                    // Init Hardware UART   
     
    WaitHandshake();                // wait for 0x55 0xAA (3s timeout then goto app))
    
    LED_PIN4  = 0;                  // LED Off (bootloader led))
    
    Application();                  // If bootloader is not init from PC, then continue to application
    
    
    // Good news is when bootloader goes to 0x0900 and is invalid, causes pic to reset and main repeated over and over till handshake and flash success!
}
