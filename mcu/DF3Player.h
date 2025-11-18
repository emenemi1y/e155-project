// STM32F401RE_USART.h
// Header for USART functions


#ifndef DF3Player_H
#define DF3Player_H

#include <stdint.h>
#include <stm32l432xx.h>
#include "mcu/STM32L432KC_UART.h"

///////////////////////////////////////////////////////////////////////////////
// Function prototypes
///////////////////////////////////////////////////////////////////////////////

void initializeDF3();
void selectSong();

#endif