/*
 * File:   application.c
 * Author: issac
 * Version: 3.10
 * Created on January 18, 2026, 12:13 PM
 * Family: 18F27Q43
 * USE 1.29.481
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>                        // for bool, true, false
#include "config.h"
#include "uart.h"

#define LED_PIN   LATBbits.LATB3            // Use LAT for Output
#define LED_TRIS  TRISBbits.TRISB3
#define TIMER2_COUNT        186             // 3s at 64MHz (Calculated correctly!)

uint8_t t2_counter = 0;                     // For Timer 2 ISR counter (16 ms trigger ISR function max)

//-------------------------------------------------------
// TIMER2 ROUTINE
//-------------------------------------------------------
void TIMER2_Init(void) 
{
    // 1. Set Clock Source to Fosc/4 (0001)
    // Same register name as K42
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
    // CHANGE: In the Q43, TMR2 interrupts moved from PIR4 to PIR3.
    PIR3bits.TMR2IF = 0;
    PIE3bits.TMR2IE = 1; // Set to 1 to actually enable the interrupt

    // 5. Global Interrupts
    // On Q43, MVECEN (Multi-Vector) is often enabled by default in Config Bits.
    // If using Legacy Mode:
    INTCON0bits.GIEL = 1; 
    INTCON0bits.GIEH = 1; 

    // 6. Start the Timer
    T2CONbits.ON = 1; 
}

void Timer2_Start(void)
{
    t2_counter = 0;            // Reset ISR counter
    T2TMR = 0;                 // Clear timer                
 
    // CHANGE: TMR2 Interrupts moved from PIR4/PIE4 to PIR3/PIE3
    PIR3bits.TMR2IF = 0;       // Clear pending interrupt
    PIE3bits.TMR2IE = 1;       // Enable Timer2 interrupt
    
    // The bit name remains 'ON' on the Q43
    T2CONbits.ON = 1;          // Start Timer2
}

void Timer2_Stop(void)
{
    t2_counter = 0;             // Reset ISR counter
    T2TMR = 0;                  // Clear timer  

    // CHANGE: TMR2 Enable bit moved from PIE4 to PIE3
    PIE3bits.TMR2IE = 0;        // Disable Timer2 interrupt
    
    // Remaining register name is consistent with K42
    T2CONbits.ON = 0;           // Stop Timer2             
}

// Note: 'high_priority' is still supported for backward compatibility, 
// but ensure MVECEN is set to OFF in your Configuration Bits if using this style.
__attribute__((section("my_isr_code"))) 
void __interrupt(high_priority) App_ISR(void)
{
    // CHANGE: TMR2 shifted to PIE3/PIR3
    if (PIE3bits.TMR2IE && PIR3bits.TMR2IF)
    {
        PIR3bits.TMR2IF = 0; // Clear flag

        t2_counter++;
        
        // At 64MHz, Fosc/4 is 16MHz. 
        // With your 1:128 Prescaler and 1:8 Postscaler, the math is:
        // 16,000,000 / (128 * 8 * 250) = 62.5 Hz (or a 16ms period)
        
        if (t2_counter >= TIMER2_COUNT)      
        {
            t2_counter = 0;           // Reset it
            //UART_TxString("<Application Timer 2S>"); // Enable DEMO
        }
    }
}


//-------------------------------------------------------
// EEPROM READ/WRITE
//-------------------------------------------------------
uint8_t EEPROM_Read(uint16_t address) {
    // 1. Point to the EEPROM Base Address (0x380000)
    // The Q43 requires the Upper byte (U) to be 0x38 for EEPROM access.
    NVMADRU = 0x38; 
    NVMADRH = (uint8_t)((address >> 8) & 0x03); // Top 2 bits of 10-bit address
    NVMADRL = (uint8_t)(address & 0xFF);        // Lower 8 bits

    // 2. Select the "Byte Read" Command
    // According to the datasheet, NVMCMD = 0x00 is for a single byte read.
    NVMCON1bits.CMD = 0x00; 

    // 3. Start the Read Operation
    // We use the 'GO' bit you found in your header file.
    NVMCON0bits.GO = 1;

    // 4. Wait for completion
    // The hardware automatically clears this bit back to 0 when the data is ready.
    while (NVMCON0bits.GO); 

    // 5. Return the data from the result register
    return NVMDATL; 
}

void EEPROM_Write(uint16_t address, uint8_t data) {
    // 1. Set the Address (EEPROM base is 0x380000)
    NVMADRU = 0x38;
    NVMADRH = (uint8_t)((address >> 8) & 0x03);
    NVMADRL = (uint8_t)(address & 0xFF);

    // 2. Load the data byte
    NVMDATL = data;

    // 3. Set command to 'Byte Write' (0x03)
    NVMCON1bits.CMD = 0x03;

    // 4. Critical Section: Unlock Sequence
    // On the Q43, the unlock sequence directly enables the next 'GO' command.
    uint8_t GIE_state = INTCON0bits.GIE; 
    INTCON0bits.GIE = 0;

    NVMLOCK = 0x55;
    NVMLOCK = 0xAA;
    NVMCON0bits.GO = 1;      // Start the write operation

    // 5. Restore Interrupts
    INTCON0bits.GIE = GIE_state;

    // 6. Wait for completion
    // The GO bit (Bit 0) clears automatically when the write is finished.
    while (NVMCON0bits.GO);
    
    // 7. Clear the command register for safety (Optional but recommended)
    NVMCON1bits.CMD = 0x00;
}


// The __at(0x900) attribute forces the linker to place this code at address 0x900.
// Can't place it in 0x800 to 0x802 will build with errors. place 0x804 or higher ok
// This also avoids the compiler add 1FFFC GOTO function
void __at(0x900) main(void)  
{
    uint8_t b;                          // Variable to hold the received handshake byte
    
    // --- PERIPHERAL INITIALIZATION ---
    // Configure the LED pin as an output. 
    // Ensure LED_TRIS is mapped to a different pin than the bootloader's status LED.
    LED_TRIS = 0;                       

    // Initialize UART1 with the 64 MHz parameters (Baud, PPS, and Digital mode).
    // Note: Both Bootloader and App must use identical UART settings for a seamless transition.
    UART_Init();  
    //TIMER2_Init();                      // Enable DEMO
    //Timer2_Start();                     // Enable DEMO
    
    // WRITING to EEPROM
    // EEPROM_Write(0x0010, newValue);
    
      
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
        if (PIR4bits.U1RXIF)            // Check if at least one byte is in the RX FIFO
        {
            b = UART_Rx();
            
            // If the handshake byte (0x55 or 0xAA) is detected:
            if (b == 0x55 || b == 0xAA)             
            {
                //Timer2_Stop(); // Enable DEMO
                
                // Acknowledge the request so the PC knows the app has "heard" the command.
                UART_TxString("<InitFromApp>");
                __delay_ms(50);
                
                // SOFTWARE RESET
                // Force a jump to address 0x0000. 
                // This executes the hardware reset vector where the Bootloader resides.
                // Upon restart, the Bootloader's 'WaitHandshake' will detect the full 55/AA sequence.
                asm ("goto 0x0000");    
            } 
        }
    }
}