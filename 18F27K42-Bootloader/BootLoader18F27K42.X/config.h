/* 
 * File:   
 * Author: Issac
 * Comments:
 * Revision history: 
 * Version 1.01
 */
#ifndef CONFIG_H
#define CONFIG_H

// ===============================================
// CONFIGURATION BITS FOR PIC18F27K42 BOOTLOADER
// ===============================================

// ---------------- CONFIG1L ----------------
// External Oscillator Selection
#pragma config FEXTOSC = OFF          // Oscillator not enabled, use internal
#pragma config RSTOSC  = HFINTOSC_64MHZ // Reset uses HFINTOSC @ 64 MHz

// ---------------- CONFIG1H ----------------
#pragma config CLKOUTEN = ON         // CLKOUT function disabled
#pragma config PR1WAY   = OFF         // PRLOCK bit can be cleared and set more than once
#pragma config CSWEN    = ON          // Clock switching allowed
#pragma config FCMEN    = ON          // Fail-Safe Clock Monitor enabled

// ---------------- CONFIG2L ----------------
#pragma config MCLRE    = EXTMCLR     // MCLR enabled (pin RE3 functions as MCLR)
#pragma config PWRTS    = PWRT_1      // Power-up Timer disabled
#pragma config MVECEN   = OFF         // Multi-vector interrupts disabled
#pragma config IVT1WAY  = OFF         // IVTLOCK can be cleared/set more than once
#pragma config LPBOREN  = OFF         // Low-power BOR disabled
#pragma config BOREN    = SBORDIS     // Brown-out Reset enabled

// ---------------- CONFIG2H ----------------
#pragma config BORV     = VBOR_2P45   // Brown-out voltage = 2.45 V
#pragma config ZCD      = OFF         // Zero-cross detect disabled
#pragma config PPS1WAY  = OFF         // PPSLOCK can be cleared/set more than once
#pragma config STVREN   = ON          // Stack overflow/underflow reset enabled
#pragma config DEBUG    = OFF         // Background debugger disabled
#pragma config XINST    = OFF         // Extended instruction set disabled

// ---------------- CONFIG3L ----------------
#pragma config WDTCPS   = WDTCPS_31   // WDT period selection (1:65536)
#pragma config WDTE     = OFF         // WDT disabled

// ---------------- CONFIG3H ----------------
#pragma config WDTCWS   = WDTCWS_7    // WDT window always open
#pragma config WDTCCS   = SC          // WDT clock = software control

// ---------------- CONFIG4L ----------------
#pragma config BBSIZE   = BBSIZE_512  // Boot block size = 512 words (1 KB)
#pragma config BBEN     = OFF         // Boot block not disabled
#pragma config SAFEN    = OFF         // SAF area disabled
#pragma config WRTAPP   = OFF         // Application block write protection disabled

// ---------------- CONFIG4H ----------------
#pragma config WRTB     = OFF         // Boot block write protection disabled
#pragma config WRTC     = OFF         // Config registers write protection disabled
#pragma config WRTD     = OFF         // Data EEPROM write protection disabled
#pragma config WRTSAF   = OFF         // SAF write protection disabled
#pragma config LVP      = OFF         // Low-voltage programming disabled

// ---------------- CONFIG5L ----------------
#pragma config CP       = OFF         // Code protection disabled

// ===============================================
// SYSTEM DEFINES
// ===============================================
#define _XTAL_FREQ 64000000           // HFINTOSC = 64 MHz internal oscillator

#endif
