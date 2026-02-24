/*
 * File:   application.c
 * Author: issac
 * Version: 3.03
 * Created on January 18, 2026, 12:13 PM
 * Family: 16F88
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>                        // for bool, true, false
#include "config.h"
#include "uart.h"

#define LED_PIN PORTBbits.RB3               // Application LED
#define LED_TRIS TRISBbits.TRISB3
#define TIMER2_COUNT 93                     // 32ms(ISR Trigger) x 93(t2_counter) = 3000 ms Timout

uint8_t t2_counter = 0;                     // For Timer 2 ISR counter (32 ms trigger ISR function)

//-------------------------------------------------------
// TIMER2 ROUTINE
//-------------------------------------------------------
void TIMER2_Init(void)
{
    /**
    * @brief Initializes Timer2 for a 32ms periodic interrupt.
    * Settings: Fosc = 8MHz, Prescaler 1:16, Postscaler 1:16, PR2 = 249.
    */
    
    T2CONbits.TMR2ON = 0;         // Ensure timer is OFF during configuration
    
    // Interrupt Configuration
    PIR1bits.TMR2IF = 0;          // Clear the interrupt flag to avoid immediate firing
    PIE1bits.TMR2IE = 1;          // Enable Timer2 interrupts (usually 1 if you want 32ms ISR)
    TMR2 = 0;                     // Reset counter to 0

    // Period Register
    // Formula: (PR2 + 1) * Tcy * Prescaler * Postscaler
    PR2 = 249;                    // Sets period to 32ms at 8MHz Fosc

    // Clock Control
    T2CONbits.T2CKPS = 0b10;      // Prescaler = 1:16 (Sets T2CKPS1=1, T2CKPS0=0)
    
    // Postscaler Configuration
    // 0b1111 = 1:16 Postscaler
    T2CONbits.TOUTPS3 = 1;
    T2CONbits.TOUTPS2 = 1;
    T2CONbits.TOUTPS1 = 1;
    T2CONbits.TOUTPS0 = 1;

    // Global Interrupt Enables
    INTCONbits.PEIE = 1;          // Enable Peripheral Interrupts
    INTCONbits.GIE  = 1;          // Enable Global Interrupts
}

void Timer2_Start(void)
{
   PIR1bits.TMR2IF = 0;        // clear pending interrupt FIRST
   t2_counter = 0;             // reset ISR counter
   TMR2 = 0;                   // reset File Register counter
   
   PIE1bits.TMR2IE = 1;        // enable Timer2 interrupt flag
   T2CONbits.TMR2ON = 1;       // start Timer2
}

void Timer2_Stop(void)
{
    t2_counter = 0;             // reset ISR counter
    TMR2 = 0;                   // reset counter
    
    PIE1bits.TMR2IE = 0;        // Disable Timer2 interrupt flag
    T2CONbits.TMR2ON = 0;       // Stop Timer2
}

void __at(0xF00) App_ISR(void)
{
    if (PIR1bits.TMR2IF)
    {
        PIR1bits.TMR2IF = 0;                        // Clear flag

        t2_counter++;                               // counter to compare
        if (t2_counter >= TIMER2_COUNT)      
        {
            t2_counter = 0;                         // Reset it
            
            //UART_TxString("<From ISR Demo>");         // Enable Demo to test timer2          
        }
    }
    
}


// Write to EEPROM just one address to confirm on real hardware. (not used just for testing!)
void EEPROM_WriteByte(uint8_t address, uint8_t data) 
{
    // Not used just keeping a template of it!
    
    while (EECON1bits.WR);              // Wait until previous write finishes
    EEADR = address;                    // Address to write
    EEDATA = data;                      // Data to write
    EECON1bits.EEPGD = 0;               // Select DATA EEPROM memory
    EECON1bits.WREN = 1;                // Enable write
    
    // Required sequence to unlock
    INTCONbits.GIE = 0;                 // Disable interrupts
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;                  // Start write
    while (EECON1bits.WR);              // Wait until previous write finishes
    
    INTCONbits.GIE = 1;                 // Enable interrupts
    EECON1bits.WREN = 0;                // Disable write
}


void  main(void) {
    // Add application code here......
    
    App_ISR();                           //Without that "dummy" call, the XC8 compiler looks at your code, sees that nothing is technically calling AppISR
    
    uint8_t b;                          // Application monitor 0x55 for bootloading
    LED_TRIS = 0;                       // Output
            
    UART_Init();                        // Init UART
    
    // Enable Demo to test timer2
    //TIMER2_Init();
    //Timer2_Start();
       
    //EEPROM_WriteByte(0x00, 0x55);
    //EEPROM_WriteByte(0x01, 0x55);
    
    while(1) 
    {
        // Flash Led for application.  Use different pin then bootloader.
        LED_PIN = 1;                    // LED ON
        __delay_ms(500);                // Wait 500 ms
        LED_PIN = 0;                    // LED OFF
        __delay_ms(500);                // Wait 500 ms
            
        if (PIR1bits.RCIF)              // PIR1bits.RCIF = 1 ? at least one byte in RCREG
        {
            b = UART_Rx();
            if (b == 0x55)              // This is Handshake byte. In application 0xAA is not needed.  It will reboot then 0x55 and 0xAA will be detected
            {
                //Timer2_Stop;            // Enable Demo to test timer2
                // Send to Host Handshake received at app location
                UART_TxString("<InitFromApp>");
                
                asm ("goto 0x0000");    // Restart to bootloader in preparation for flash
            } 
        }
         
    }
}
