/*
 * File:   uart.h
 * Version: 1.01
 * Author: Issac
 * Created on January 19, 2026, 2:50 PM
 * Family: 18F27K42
 */

#ifndef UART_H    // This is a "Header Guard"
#define UART_H    // It prevents errors if you include this file twice

#include <xc.h>   // Needed so the header understands PIC types
#include <stdint.h> // Needed for uint8_t

// These are "Prototypes" - they tell the main program what functions exist
void UART_Init(void);
void UART_Tx(uint8_t d);
void UART_TxString(const char *s);
uint8_t UART_Rx(void);

#endif // UART_H
