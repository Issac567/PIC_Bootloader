/*
 * File:   application.c
 * Author: issac
 * Version: 3.06
 * Family: 24F64GA102
 * Created on January 18, 2026, 12:13 PM
 */

/**
 * PROJECT MEMORY CONFIGURATION NOTES:
 * ----------------------------------------------------------------------------
 * 1. This project uses a custom Linker Script (added via the 'Linker Files' tab).
 * 2. The default p24FJ64GA102.gld was modified to define fixed memory ranges.
 * 3. This prevents memory collisions between the Bootloader and Application.
 * * [APPLICATION RANGE]
 * Start Address (ORIGIN): 0x900
 * End Address:           0xABF6
 * Length:                0xA2F4
 * the Unified Hex generator to merge this App with the Bootloader.
 * ----------------------------------------------------------------------------
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>                        // for bool, true, false
#include "config.h"
#include "uart.h"

#define LED_PIN   LATBbits.LATB3                // Use LAT for Output
#define LED_TRIS  TRISBbits.TRISB3
#define TIMER2_COUNT        186                 // 3s 

uint8_t t2_counter = 0;                         // For Timer 2 ISR counter (16 ms trigger ISR function max)

void TIMER2_Init(void) 
{
    // 1. Clock Source
    // PIC24 uses Fcy (Fosc/2). No T2CLKCON register on this chip.
    T2CONbits.TCS = 0;      // 0 = Internal clock (Fcy)

    // 2. Mode
    // PIC24F does not have Hardware Limit Timer (HLT). Standard only.

    // 3. Configuration
    // No Postscaler on PIC24. Using 1:256 Prescaler.
    T2CONbits.TCKPS = 0b11; // 1:256 Prescaler
    
    // 4. Period Match (PR2)
    // For 16ms @ 32MHz Fosc (16 MIPS)
    PR2 = 1000;             
    TMR2 = 0;               // Reset count

    // 5. Interrupts
    // IFS0 is the flag register, IEC0 is the enable register.
    IFS0bits.T2IF = 0;      
    IEC0bits.T2IE = 0;      
    IPC1bits.T2IP = 4;      // Set Priority (4 is middle/default)

    // 6. Start
    T2CONbits.TON = 0;      // 'TON' is the PIC24 version of 'OFF'
}

void Timer2_Start(void)
{
    t2_counter = 0;         // Reset ISR counter
    TMR2 = 0;               // Clear timer (Register is TMR2, not T2TMR)

    // PIC24 Interrupt Flag (IFS0) and Enable (IEC0)
    IFS0bits.T2IF = 0;      // Clear pending interrupt
    IEC0bits.T2IE = 1;      // Enable Timer2 interrupt
    
    // In PIC24, the start bit is 'TON'
    T2CONbits.TON = 1;      // Start Timer2
}

void Timer2_Stop(void)
{
    t2_counter = 0;             // Reset ISR counter
    TMR2 = 0;                   // Clear timer (Register is TMR2)

    // PIC24 Interrupt Enable is in the IEC0 register
    IEC0bits.T2IE = 0;          // Disable Timer2 interrupt
    
    // The stop bit is 'TON'
    T2CONbits.TON = 0;          // Stop Timer2             
}

// Use the address attribute to match the Bootloader's goto
void __attribute__((address(0x908), interrupt, no_auto_psv)) App_ISR(void)
{
    // 1. Clear the flag
    // In PIC24, the flag is T2IF in the IFS0 register
    IFS0bits.T2IF = 0; 

    // 2. Increment software counter
    t2_counter++;
    
    // 3. Check for timeout
    if (t2_counter >= TIMER2_COUNT)      
    {
        t2_counter = 0;           // Reset it
        
        //UART_TxString("<From ISR DEMO>");     // Enable Demo
    }
}


// The __at(0xA00) attribute forces the linker to place this code at address 0xA00.
// This preserves the 0x0000 - 0x9FF range for the Bootloader firmware.  
// This also avoids the compiler add 1FFFC GOTO function
// In XC16, we use the 'address' attribute instead of __at()
//int __attribute__((address(0x900))) main(void) 
int main(void)
{
    uint8_t b;                          // Variable to hold the received handshake byte
    
    // --- PERIPHERAL INITIALIZATION ---
    // PIC24: Ensure all pins are digital
    AD1PCFG = 0xFFFF; 

    // Configure the LED pin as an output. 
    LED_TRIS = 0;                       

    // Initialize UART1. PPS mapping must be handled inside here for PIC24.
    UART_Init();  
    
    //TIMER2_Init();            // Enable Demo
    //Timer2_Start();           // Enable Demo

    // --- MAIN APPLICATION LOOP ---
    while(1) 
    {  
        // 1. VISUAL HEARTBEAT
        LED_PIN = 1;                    // LED ON
        __delay_ms(500);                // Wait 500 ms
        LED_PIN = 0;                    // LED OFF
        __delay_ms(500);                // Wait 500 ms
        
        // 2. BOOTLOADER "SOFT-TRIGGER" MONITOR
        // PIC24: Check URXDA (Data Available) bit
        if (U1STAbits.URXDA)            
        {
            b = UART_Rx();
            
            // If the handshake byte (0x55) is detected:
            if (b == 0x55)              
            {
                
                //Timer2_Stop(); // Enable Demo
                
                // Acknowledge the request
                UART_TxString("<InitFromApp>");
                __delay_ms(50);
                
                // 3. SOFTWARE RESET
                asm("goto 0x0000");
            } 
        }
    }
}