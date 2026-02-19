/*
 * File:   newmain.c
 * Author: issac
 * Version: 1.10
 * Created on January 18, 2026, 12:13 PM
 */


#include <xc.h>
#include <stdint.h>
#include "config.h"

#define LED_PIN   LATBbits.LATB3   // Use LAT for Output
#define LED_TRIS  TRISBbits.TRISB3

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

    // Initialize UART1 with the 64 MHz parameters (Baud, PPS, and Digital mode).
    // Note: Both Bootloader and App must use identical UART settings for a seamless transition.
    UART_Init();                        
        
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