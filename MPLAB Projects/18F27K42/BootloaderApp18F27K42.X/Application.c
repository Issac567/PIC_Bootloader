/*
 * File:   application.c
 * Author: issac
 * Version: 3.01
 * Created on January 18, 2026, 12:13 PM
 * Family: 18F27K42
 */


#include <xc.h>
#include <stdint.h>
#include <stdbool.h>                        // for bool, true, false
#include "config.h"
#include "uart.h"

#define LED_PIN   LATBbits.LATB3            // Use LAT for Output
#define LED_TRIS  TRISBbits.TRISB3
#define TIMER2_COUNT 124                    // 2s  

uint8_t t2_counter = 0;                     // For Timer2 ISR counter (16 ms trigger ISR function)

//-------------------------------------------------------
// TIMER2 ROUTINE
//-------------------------------------------------------
void TIMER2_Init(void) {
    // 1. Set Clock Source to Fosc/4 (0001)
    // On K42, this remains T2CLKCON
    T2CLKCONbits.CS = 0b0001; 

    // 2. Hardware Limit Timer - Standard Timer Mode
    // T2HLT controls the "mode". 0x00 is software control / standard.
    T2HLT = 0x00; 

    // 3. Configuration
    // Prescaler 1:128 (111), Postscaler 1:8 (0111)
    T2CONbits.CKPS = 0b111;
    T2CONbits.OUTPS = 0b0111;
    
    T2PR = 249;   // Period match
    T2TMR = 0;    // Reset count

    // 4. Clear Flag and Enable Interrupts
    // On K42, Timer 2 IF/IE are in PIR4/PIE4. 
    // Note: The bit name is TMR2IF, same as before, but verify register mapping.
    PIR4bits.TMR2IF = 0;
    PIE4bits.TMR2IE = 0; 

    // 5. Global Interrupts
    // On PIC18, we usually use INTCON0
    INTCON0bits.GIEL = 1; // Low priority (or Peripheral) enable
    INTCON0bits.GIEH = 1; // High priority (or Global) enable 
}

void Timer2_Start(void)
{
    t2_counter = 0;            // Reset ISR counter
    T2TMR = 0;                 // Clear timer                
 
    PIR4bits.TMR2IF = 0;       // Clear pending interrupt
    PIE4bits.TMR2IE = 1;       // Enable Timer2 interrupt
    
    // On K42, the bit name is 'ON', not 'TMR2ON'
    T2CONbits.ON = 1;          // Start Timer2
}

void Timer2_Stop(void)
{
    t2_counter = 0;             // Reset ISR counter
    T2TMR = 0;                  // Clear timer  

    PIE4bits.TMR2IE = 0;        // Disable Timer2 interrupt
    
    // On K42, the bit name is 'ON', not 'TMR2ON'
    T2CONbits.ON = 0;           // Stop Timer2             
}

// NOTE: This ISR is dedicated to Application Firmware.
// The Bootloader remaps the hardware interrupt vector (0x0008) to this 
// specific section ("my_isr_code") located in high memory.
// 1F000 in Linker --> Additional Option --> Extra Linker Option -->   -Wl,-Pmy_isr_code=0x1F000
__attribute__((section("my_isr_code"))) 
void __interrupt(high_priority) App_ISR(void)
{
    // Best practice: Always check both the Flag (IF) AND the Enable (IE) bit
    if (PIE4bits.TMR2IE && PIR4bits.TMR2IF)
    {
        PIR4bits.TMR2IF = 0; // Clear flag    

        t2_counter++;                                
        
        /* * --- CALCULATION FOR 2.0s TIMING ---
         * Fosc = 64MHz -> Fcy = 16MHz (Tcy = 62.5ns)
         * Timer2 Settings: Prescaler 1:128, Postscaler 1:10, PR2 = 124
         * Interrupt Time = Tcy * Prescale * (PR2 + 1) * Postscale
         * 62.5ns * 128 * 125 * 10 = 0.01s (10ms per interrupt)
         * To get 2.0s: 2.0s / 0.01s = 200 counts
         * Set TIMER2_COUNT to 200
         */
        
        if (t2_counter >= TIMER2_COUNT)      
        {
            t2_counter = 0;                         // Reset it
            //UART_TxString("<From ISR Demo>");     // Enable Demo to Test Timer2
        }
    }
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

    UART_Init();                          // Initialize UART1 with the 64 MHz parameters (Baud, PPS, and Digital mode).                  
    //TIMER2_Init();                      // Enable Demo to Test Timer2
    //Timer2_Start();                     // Enable Demo to Test Timer2
    
    //EEPROM_WriteByte(0x01, 0x55);
    
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
                // Enable Demo to Test Timer2
                //Timer2_Stop();
                
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