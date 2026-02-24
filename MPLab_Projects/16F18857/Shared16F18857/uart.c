/*
 * File:   uart.c
 * Author: issac
 * Version: 3.01
 * Created on January 18, 2026, 12:13 PM
 * Family: 1618857
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
    TRISBbits.TRISB2 = 1;       // RB2 as Input (RX)
    TRISBbits.TRISB5 = 0;       // RB5 as Output (TX)

    // 2. PPS Unlock Sequence
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0;  // Unlock PPS

    // 3. Map RX to RB2
    RXPPS = 0x0A;               // RB2->EUSART:RX;

    // 4. Map RB5 to TX
    RB5PPS = 0x10;              // RB5->EUSART:TX;

    // 5. PPS Lock Sequence
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 1;  // Lock PPS

    // ----- Baud rate (57600 @ 32MHz HFINTOSC) -----
    SP1BRGL = 34;
    SP1BRGH = 0;
    BAUD1CONbits.BRG16 = 0;     // 8-bit baud generator
    TX1STAbits.BRGH = 1;        // High-speed

    // ----- UART mode -----
    TX1STAbits.SYNC = 0;        // Asynchronous
    RC1STAbits.SPEN = 1;        // Enable UART pins

    // ----- Enable TX and RX -----
    TX1STAbits.TXEN = 1;
    RC1STAbits.CREN = 1;

    // ----- Clear pending RX bytes -----
    uint8_t dummy;
    while (PIR3bits.RCIF) {
        dummy = RC1REG;         // discard
    }
}

void UART_Tx(uint8_t d)
{
    while (!PIR3bits.TXIF);   // Wait until TX1 ready
    TX1REG = d;               // Send data
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
    // Check if a receive overrun occurred
    if (RC1STAbits.OERR)
    {
        RC1STAbits.CREN = 0;   // Reset continuous receive
        RC1STAbits.CREN = 1;   // Re-enable receive
    }

    return RC1REG;             // Return received byte
}
