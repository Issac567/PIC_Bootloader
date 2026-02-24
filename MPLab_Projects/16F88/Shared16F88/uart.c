/*
 * File:   uart.c
 * Author: issac
 * Version: 3.01
 * Created on January 18, 2026, 12:13 PM
 * Family: 16F88
 */


#include <xc.h>        // MUST HAVE: Tells the compiler about your PIC's registers
#include <stdint.h>    // MUST HAVE: Defines "uint8_t"
#include "uart.h"      // MUST HAVE: Connects the C file to your Header file

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
