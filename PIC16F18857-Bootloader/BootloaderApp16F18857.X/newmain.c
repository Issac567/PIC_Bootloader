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
    // ----- Configure pins -----
    TRISCbits.TRISC7 = 1;   // RX as input (RC7)
    TRISBbits.TRISB5 = 0;   // TX as output

    // ----- PPS mapping -----
    RXPPS  = 0x17;          // RC7 = EUSART1 RX input (check datasheet PPS table)
    RB5PPS = 0x14;          // RB5 = EUSART1 TX output (from PPS output table)

    // ----- Baud rate (57600 @ 32MHz HFINTOSC) -----
    SP1BRGL = 34;
    SP1BRGH = 0;
    BAUD1CONbits.BRG16 = 0; // 8-bit baud generator
    TX1STAbits.BRGH = 1;    // High-speed

    // ----- UART mode -----
    TX1STAbits.SYNC = 0;    // Asynchronous
    RC1STAbits.SPEN = 1;    // Enable UART pins

    // ----- Enable TX and RX -----
    TX1STAbits.TXEN = 1;
    RC1STAbits.CREN = 1;
}

void UART_Tx(uint8_t d)
{
    while (!PIR3bits.TXIF);   // Wait until TX1 ready
    TX1REG = d;                 // Send data
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

    // Wait for a byte to be available (optional, polling)
    //while (!PIR3bits.RCIF);

    return RC1REG;             // Return received byte
}


void EEPROM_WriteByte(uint8_t address, uint8_t data)
{
    // Make sure address is valid (0x00?0xFF)
    uint16_t physAddr = 0xF000 | address;  // EEPROM address space is F000h?F0FFh

    // Load address
    NVMADRL = physAddr & 0xFF;
    NVMADRH = (physAddr >> 8) & 0xFF;

    // Load data into NVMDATL
    NVMDATL = data;     // Write data byte
    NVMDATH = 0;        // High byte unused for EEPROM

    // Select EEPROM region and enable write
    NVMCON1bits.NVMREGS = 1; // 1 = EEPROM access
    NVMCON1bits.WREN    = 1; // Enable writes

    // Unlock sequence (datasheet)
    INTCONbits.GIE = 0;
    NVMCON2 = 0x55;
    NVMCON2 = 0xAA;
    NVMCON1bits.WR = 1;      // Start write
    while (NVMCON1bits.WR);  // Wait until done
    INTCONbits.GIE = 1;

    // Disable write
    NVMCON1bits.WREN = 0;
}


void main(void) {
    // Add application code here......
    
    uint8_t b;                          // Application monitor 0x55 for bootloading
    LED_TRIS = 0;                       // Output
    UART_Init();                        // Init UART
        
    
    //EEPROM_WriteByte(0x00, 0x55);     // Write 0x55 to address 0x00
    //EEPROM_WriteByte(0x01, 0x55);     // Write 0x55 to address 0x01
    
    while(1) 
    {
        
        // Flash Led for application.  Use different pin then bootloader.
        LED_PIN = 1;                    // LED ON
        __delay_ms(500);                // Wait 500 ms
        LED_PIN = 0;                    // LED OFF
        __delay_ms(500);                // Wait 500 ms
        
        if (PIR3bits.RCIF)              // PIR1bits.RCIF = 1 ? at least one byte in RCREG
        {
            b = UART_Rx();
            if (b == 0x55)              // This is Handshake byte. In application 0xAA is not needed.  It will reboot then 0x55 and 0xAA will be detected
            {
                UART_TxString("<InitFromApp>");
                
                asm ("goto 0x0000");    // Restart to bootloader in preparation for flash
            } 
        }
         
    }
}
