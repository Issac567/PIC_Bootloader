/*
 * File:   application.c
 * Author: issac
 * Version: 3.01
 * Created on January 18, 2026, 12:13 PM
 * Family: 1618857
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>                        // for bool, true, false
#include "config.h"
#include "uart.h" 

#define LED_PIN   LATBbits.LATB3            // Use LAT for Output
#define LED_TRIS  TRISBbits.TRISB3
#define TIMER2_COUNT 93                     // 32ms(ISR Trigger) x 93(t2_counter) = 3000 ms or 3S Timout

uint8_t t2_counter = 0;                     // For Timer 2 ISR counter (32 ms trigger ISR function)

//-------------------------------------------------------
// TIMER2 ROUTINE
//-------------------------------------------------------
void TIMER2_Init(void) {
    // 1. Set Clock Source to Fosc/4
    T2CLKCONbits.CS = 0b0001; 

    // 2. Hardware Limit Timer - MUST be 0 for standard timer mode
    T2HLT = 0x00; 

    // 3. Configuration
    // Prescaler 1:128 (111), Postscaler 1:8 (0111)
    T2CONbits.CKPS = 0b111;
    T2CONbits.OUTPS = 0b0111;
    
    T2PR = 249;   // Period match
    T2TMR = 0;    // Reset count

    // 4. Clear Flag and Enable Interrupts
    PIR4bits.TMR2IF = 0;
    PIE4bits.TMR2IE = 0;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1; 
}

void Timer2_Start(void)
{
    t2_counter = 0;            // Reset ISR counter
    T2TMR = 0;                 // Clear timer                
 
    PIR4bits.TMR2IF = 0;       // Clear pending interrupt first
    PIE4bits.TMR2IE = 1;       // Enable Timer2 interrupt
    T2CONbits.TMR2ON = 1;      // Start Timer2
}

void Timer2_Stop(void)
{
    t2_counter = 0;             // reset ISR counter
    T2TMR = 0;                  // Clear timer  

    PIE4bits.TMR2IE = 0;        // Disable Timer2 interrupt flag
    T2CONbits.TMR2ON = 0;       // Stop Timer2             
}


void  __at(0x7000)App_ISR(void)
{
    if (PIR4bits.TMR2IF)
    {
        PIR4bits.TMR2IF = 0;                        // Clear flag

        t2_counter++;                               // counter to compare
        // Example: If you wanted a 3-second timeout
        // 32ms(ISR interrupt) * 93(TIMER2_COUNT_FOR_1S) = 2976 ms (Close enough for most tasks)
        if (t2_counter >= TIMER2_COUNT)      
        {
            t2_counter = 0;                         // Reset it
            
            //UART_TxString("<From ISR DEMO>");     // Enable Demo For ISR Test
        }
    }
}


void EEPROM_WriteByte(uint8_t address, uint8_t data)
{
    // Make sure address is valid (0x00?0xFF)
    uint16_t physAddr = 0xF000 | address;  // EEPROM address space is F000h?F0FFh

    // Load address
    NVMADRL = physAddr & 0xFF;
    NVMADRH = (physAddr >> 8) & 0xFF;

    // Load data into NVMDATL
    NVMDATL = data;                         // Write data byte
    NVMDATH = 0;                            // High byte unused for EEPROM

    // Select EEPROM region and enable write
    NVMCON1bits.NVMREGS = 1;                // 1 = EEPROM access
    NVMCON1bits.WREN    = 1;                // Enable writes

    // Unlock sequence (datasheet)
    INTCONbits.GIE = 0;
    NVMCON2 = 0x55;
    NVMCON2 = 0xAA;
    NVMCON1bits.WR = 1;                     // Start write
    while (NVMCON1bits.WR);                 // Wait until done
    INTCONbits.GIE = 1;

    // Disable write
    NVMCON1bits.WREN = 0;
}


void main(void) {
    // Add application code here......
    App_ISR();
    
    uint8_t b;                          // Application monitor 0x55 for bootloading
    LED_TRIS = 0;                       // Output
    UART_Init();                        // Init UART
        
    
    // Enable Demo For ISR Test
    //TIMER2_Init();
    //Timer2_Start();
    
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
                //Timer2_Stop();        // Enable Demo For ISR Test
                
                UART_TxString("<InitFromApp>");
                
                asm ("goto 0x0000");    // Restart to bootloader in preparation for flash
            } 
        }
         
    }
}
