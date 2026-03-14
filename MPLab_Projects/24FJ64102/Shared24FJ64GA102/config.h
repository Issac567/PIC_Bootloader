/* 
 * File:   
 * Author: Issac
 * Comments:
 * Revision history: 
 * Version 1.01
 */

#ifndef CONFIG_H
#define CONFIG_H

// CONFIG4
#pragma config DSWDTPS = DSWDTPSF
#pragma config DSWDTOSC = LPRC
#pragma config RTCOSC = SOSC
#pragma config DSBOREN = OFF       // Turned OFF to save power/stability during flash
#pragma config DSWDTEN = OFF      // DISABLE Deep Sleep Watchdog for bootloader

// CONFIG3
#pragma config WPFP = WPFP63
#pragma config SOSCSEL = IO       // Set to IO if not using a 32kHz crystal to free pins
#pragma config WUTSEL = LEG
#pragma config WPDIS = WPDIS      // Disable write protection so bootloader can work
#pragma config WPCFG = WPCFGDIS
#pragma config WPEND = WPENDMEM

// CONFIG2
#pragma config POSCMOD = NONE     // No external crystal
#pragma config I2C1SEL = PRI
#pragma config IOL1WAY = OFF      // ALLOW multiple PPS reconfigurations (very helpful)
#pragma config OSCIOFNC = ON      // Set to ON to turn OSCO pin into a digital I/O pin
#pragma config FCKSM = CSECMD     // Clock Switching ENABLED (required to switch to PLL)
#pragma config FNOSC = FRCPLL     // Fast RC with PLL (This gives you 32MHz) FRCPLL
#pragma config IESO = OFF         // Two-Speed Start-up disabled

// CONFIG1
#pragma config WDTPS = PS32768
#pragma config FWPSA = PR128
#pragma config WINDIS = OFF
#pragma config FWDTEN = OFF       // DISABLE Watchdog Timer (Crucial for bootloaders!)
#pragma config ICS = PGx1         // Adjust this if you use PGC2/PGD2 for programming
#pragma config GWRP = OFF
#pragma config GCP = OFF
#pragma config JTAGEN = OFF       // DISABLE JTAG to free up RB10, RB11, RA4, RA5!

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
// ===============================================
// SYSTEM DEFINES
// ===============================================
#define FCY 16000000UL        // 16 MHz Instruction Clock (32MHz Osc / 2)
#include <libpic30.h>         // Required for __delay_ms to work on PIC24

#endif
