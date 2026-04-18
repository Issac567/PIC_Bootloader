/*
 * File:   uart.c
 * Author: issac
 * Version: 3.04
 * Created on January 18, 2026, 12:13 PM
 * Family: 18F27Q43
 */


#include <xc.h>        // MUST HAVE: Tells the compiler about your PIC's registers
#include <stdint.h>    // MUST HAVE: Defines "uint8_t"
#include "uart.h"      // MUST HAVE: Connects the C file to your Header file

#define _XTAL_FREQ 64000000           // HFINTOSC = 64 MHz internal oscillator

//-------------------------------------------------------
// UART ROUTINE
//-------------------------------------------------------
void UART_Init(void)
{
    // 1. Set pin directions
    TRISBbits.TRISB2 = 1; // RB2 as Input (RX)
    TRISBbits.TRISB5 = 0; // RB5 as Output (TX)

    // 2. PPS Unlock Sequence
    INTCON0bits.GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0; // Unlock PPS

    // 3. Map RX to RB2
    // CHANGE: On Q43, RB2 input code is 0x0A (Luckily same as K42)
    U1RXPPS = 0x0A; 

    // 4. Map RB5 to TX
    // CHANGE: On Q43, U1TX output function code is 0x20 (K42 was 0x13)
    RB5PPS = 0x20; 

    // 5. PPS Lock Sequence
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 1; // Lock PPS
    
    INTCON0bits.GIE = 1;

    // 6. Baud rate (57600 @ 64MHz)
    // Formula: n = (64MHz / (4 * 57600)) - 1 = 276.7 -> 277
    U1BRGL = 0x15; 
    U1BRGH = 0x01; 

    // 7. UART Configuration
    U1CON0bits.BRGS = 1;      // High-speed (4 clocks per bit)
    U1CON0bits.MODE = 0b0000; // Asynchronous 8-bit mode
    
    // 8. Enable UART and Pins
    U1CON1bits.ON = 1;        // Serial port enabled
    U1CON0bits.TXEN = 1;      // Transmit enabled
    U1CON0bits.RXEN = 1;      // Receive enabled

    // 9. Clear pending RX bytes
    // CHANGE: UART1 RX Flag moved to PIR6
    uint8_t dummy;
    while (PIR4bits.U1RXIF) {
        dummy = U1RXB;        
    }
}

void UART_Tx(uint8_t d)
{
    // CHANGE: On Q43, UART1 TX flag is in PIR7
    // This flag is HIGH when the transmit FIFO has room for at least one more byte.
    while (!PIR4bits.U1TXIF);  

    // Load the data into the Transmit Buffer
    // Q43 uses U1TXB (Universal UART Transmit Buffer)
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

