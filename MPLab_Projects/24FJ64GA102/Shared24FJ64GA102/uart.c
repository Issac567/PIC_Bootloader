/*
 * File:   uart.c
 * Author: issac
 * Version: 3.03
 * Created on January 18, 2026, 12:13 PM
 * Family: 24FJ64GA102
 */


#include <xc.h>        // MUST HAVE: Tells the compiler about your PIC's registers
#include <stdint.h>    // MUST HAVE: Defines "uint8_t"
#include "uart.h"      // MUST HAVE: Connects the C file to your Header file

#define FCY 16000000UL
#include <libpic30.h>

//-------------------------------------------------------
// UART ROUTINE
//-------------------------------------------------------
void UART_Init(void)
{
    // 1. Set pin directions
    TRISBbits.TRISB2 = 1; // RB2/RP2 as Input (RX)
    TRISBbits.TRISB5 = 0; // RB5/RP5 as Output (TX)

    // 2. PPS Unlock Sequence (Using XC16 built-in for safety)
    __builtin_write_OSCCONL(OSCCON & ~0x40); // Clear bit 6 to unlock PPS

    // 3. Map RX to RP2 (RB2)
    // On PIC24F, RPINR18 handles UART1 RX
    RPINR18bits.U1RXR = 2; 

    // 4. Map RP5 (RB5) to UART1 TX
    // Function code for U1TX on this device is 3
    RPOR2bits.RP5R = 3; 

    // 5. PPS Lock Sequence
    __builtin_write_OSCCONL(OSCCON | 0x40); // Set bit 6 to lock PPS

    // 6. Baud rate (57600 @ 32MHz Fosc)
    // PIC24F Formula: U1BRG = (Fcy / (16 * Baud)) - 1
    // Fcy = Fosc / 2 = 16,000,000
    // BRG = (16,000,000 / (16 * 57600)) - 1 = 16.36 -> 16
    U1BRG = 16; 

    // 7. UART Configuration
    U1MODE = 0;              // clear all bits
    U1MODEbits.UARTEN = 1;   // Enable UART
    U1MODEbits.BRGH = 0;     // Standard Speed (16 clocks per bit)
    
    // 8. Enable Transmission and Reception
    U1STA = 0;               // clear status
    U1STAbits.UTXEN = 1;     // Transmit enabled
    // Note: URXEN is not a separate bit on this older GA102; 
    // UARTEN handles the receiver.

    // 9. Clear pending RX bytes
    //uint16_t dummy;
    while (U1STAbits.URXDA) { // While UART1 Receive Data Available
        //dummy = U1RXREG; 
        (void)U1RXREG; // Read and intentionally discard the result
    }
}

void UART_Tx(uint8_t d)
{
    // 1. Check if the Transmit Buffer is full
    // UTXBF == 1 means the 4-level FIFO is completely full.
    // We wait while it is 1 (Full).
    while (U1STAbits.UTXBF);  

    // 2. Load the data into the Transmit Register
    // PIC24 uses U1TXREG (16-bit register, but we write 8-bit data)
    U1TXREG = d;                 
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
    // 1. Check for Receive Overflow Error (OERR)
    // On PIC24F, if OERR is set, the receiver is DISABLED until cleared.
    if (U1STAbits.OERR)
    {
        // To clear the overflow and "unfreeze" the receiver:
        U1STAbits.OERR = 0; 
    }

    // 2. Optional: Check for Framing Errors (FERR)
    if (U1STAbits.FERR)
    {
        // Read the bad data to clear the error state from the FIFO
        //uint16_t dummy = U1RXREG;
        (void)U1RXREG; // Read and intentionally discard the result
    }

    // 3. Wait for data to be available in the FIFO
    // URXDA (Receive Data Available) is 1 when there is at least 1 byte in the FIFO.
    //while (!U1STAbits.URXDA);  // not needed.  its done elsewhere

    // 4. Return the byte from the 16-bit register
    return (uint8_t)U1RXREG;
}
