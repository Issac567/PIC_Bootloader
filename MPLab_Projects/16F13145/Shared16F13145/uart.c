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
    // ----- 1. Set pin directions -----
    TRISCbits.TRISC2 = 1;   // RC2 as input (RX) - Updated from RC4
    TRISCbits.TRISC5 = 0;   // RC5 as output (TX)

    // ----- 2. PPS Unlock Sequence -----
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0;  // Unlock PPS

    // ----- 3. Map PPS -----
    RX1PPS = 0x12;  // RC2 -> EUSART RX (Updated from 0x0C)
    RC5PPS = 0x13;  // EUSART TX -> RC5 (Remains the same)

    // ----- 4. PPS Lock Sequence -----
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 1;  // Lock PPS

    // ----- 5. Baud rate (57600 @ 32MHz HFINTOSC) -----
    SP1BRGL = 34;
    SP1BRGH = 0;
    BAUD1CONbits.BRG16 = 0;  // 8-bit baud generator
    TX1STAbits.BRGH = 1;     // High-speed

    // ----- 6. UART mode -----
    TX1STAbits.SYNC = 0;     // Asynchronous
    RC1STAbits.SPEN = 1;     // Enable UART pins

    // ----- 7. Enable TX and RX -----
    TX1STAbits.TXEN = 1;
    RC1STAbits.CREN = 1;

    // ----- 8. Clear any pending RX bytes -----
    uint8_t dummy;
    while (PIR4bits.RC1IF) {  
        dummy = RC1REG;        
    }
}

void UART_Tx(uint8_t d)
{
    while (!PIR4bits.TX1IF);   // Wait until TX1 ready
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
