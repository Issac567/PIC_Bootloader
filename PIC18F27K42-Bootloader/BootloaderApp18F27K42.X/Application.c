/*
 * File:   newmain.c
 * Author: issac
 * Version: 2.01
 * Created on January 18, 2026, 12:13 PM
 */


#include <xc.h>
#include <stdint.h>
#include "config.h"
#include <stdbool.h>                        // for bool, true, false

#define LED_PIN   LATBbits.LATB3            // Use LAT for Output
#define LED_TRIS  TRISBbits.TRISB3
#define TIMER2_COUNT 124                    // 2s  

uint8_t t2_counter = 0;                     // For Timer2 ISR counter (16 ms trigger ISR function)

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
    INTCON0bits.GIE = 0;
    
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
    
    INTCON0bits.GIE = 1; // Restore interrupt state

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
    t2_counter = 0;            // Reset ISR counter
    T2TMR = 0;                 // Clear timer                
 
    PIR4bits.TMR2IF = 0;       // Clear pending interrupt
    PIE4bits.TMR2IE = 1;       // Enable Timer2 interrupt
    
    // On K42, the bit name is 'ON', not 'TMR2ON'
    T2CONbits.ON = 1;          // Start Timer2
}

void Timer2_Stop(void)
{
    t2_counter = 0;             // Reset ISR counter
    T2TMR = 0;                  // Clear timer  

    PIE4bits.TMR2IE = 0;        // Disable Timer2 interrupt
    
    // On K42, the bit name is 'ON', not 'TMR2ON'
    T2CONbits.ON = 0;           // Stop Timer2             
}

// NOTE: This ISR is dedicated to Application Firmware.
// The Bootloader remaps the hardware interrupt vector (0x0008) to this 
// specific section ("my_isr_code") located in high memory.
// 1F000 in Linker --> Additional Option --> Extra Linker Option -->   -Wl,-Pmy_isr_code=0x1F000
__attribute__((section("my_isr_code"))) 
void __interrupt(high_priority) my_high_isr(void)
{
    // Best practice: Always check both the Flag (IF) AND the Enable (IE) bit
    if (PIE4bits.TMR2IE && PIR4bits.TMR2IF)
    {
        PIR4bits.TMR2IF = 0; // Clear flag    

        t2_counter++;                                
        
        /* * --- CALCULATION FOR 2.0s TIMING ---
         * Fosc = 64MHz -> Fcy = 16MHz (Tcy = 62.5ns)
         * Timer2 Settings: Prescaler 1:128, Postscaler 1:10, PR2 = 124
         * Interrupt Time = Tcy * Prescale * (PR2 + 1) * Postscale
         * 62.5ns * 128 * 125 * 10 = 0.01s (10ms per interrupt)
         * To get 2.0s: 2.0s / 0.01s = 200 counts
         * Set TIMER2_COUNT to 200
         */
        
        if (t2_counter >= TIMER2_COUNT)      
        {
            t2_counter = 0;                         // Reset it
            //UART_TxString("<Application Timer 2S>"); // Demo 
        }
    }
}

void EEPROM_WriteByte(uint16_t address, uint8_t data)
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


// The __at(0xA00) attribute forces the linker to place this code at address 0xA00.
// This preserves the 0x0000 - 0x9FF range for the Bootloader firmware.  
// This also avoids the compiler add 1FFFC GOTO function
void __at(0xA00) main(void) 
{
    uint8_t b;                          // Variable to hold the received handshake byte
    
    // --- PERIPHERAL INITIALIZATION ---
    // Configure the LED pin as an output. 
    // Ensure LED_TRIS is mapped to a different pin than the bootloader's status LED.
    LED_TRIS = 0;                       

    UART_Init();                        // Initialize UART1 with the 64 MHz parameters (Baud, PPS, and Digital mode).                  
    //TIMER2_Init();                      // Demo Init Timer
    //Timer2_Start();                     // Demo working in Application area
    
    //EEPROM_WriteByte(0x01, 0x55);
    
    // --- MAIN APPLICATION LOOP ---
    while(1) 
    {  
        // 1. VISUAL HEARTBEAT
        // Standard blink pattern to confirm the application is currently running.
        LED_PIN = 1;                    // LED ON
        __delay_ms(500);                // Wait 500 ms
        LED_PIN = 0;                    // LED OFF
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
                // Demo Stop timer2 we dont want msg corruption
                //Timer2_Stop();
                
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