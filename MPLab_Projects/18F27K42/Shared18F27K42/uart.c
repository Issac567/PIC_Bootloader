/*
 * File:   uart.c
 * Author: issac
 * Version: 3.01
 * Created on January 18, 2026, 12:13 PM
 * Family: 18F27K42
 */


#include <xc.h>        // MUST HAVE: Tells the compiler about your PIC's registers
#include <stdint.h>    // MUST HAVE: Defines "uint8_t"
#include "uart.h"      // MUST HAVE: Connects the C file to your Header file

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

