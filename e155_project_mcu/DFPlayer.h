// STM32F401RE_USART.h
// Header for USART functions


#ifndef DF3Player_H
#define DF3Player_H

#include <stdint.h>
#include <stm32l432xx.h>
#include "STM32L432KC_UART.h"
#include "STM32L432KC_TIM.h"

///////////////////////////////////////////////////////////////////////////////
// Function prototypes
///////////////////////////////////////////////////////////////////////////////

void initializeDF3(USART_TypeDef * USART);
void selectSong();
void pauseSong(USART_TypeDef * USART);
void playSong (USART_TypeDef * USART, uint8_t song);
void setVolume(USART_TypeDef * USART, uint8_t volume);

#endif