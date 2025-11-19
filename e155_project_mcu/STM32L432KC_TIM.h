// STM32F401RE_TIM.h
// Header for TIM functions

#ifndef STM32L4_TIM_H
#define STM32L4_TIM_H


#include "STM32L432KC_RCC.h"
#include <stdint.h>
#include <stm32l432xx.h>

#include <stdio.h>
#include <stdlib.h>

#include <stdint.h> // Include stdint header
#include <stm32l432xx.h>  // CMSIS device library include
#include "STM32L432KC_GPIO.h"


///////////////////////////////////////////////////////////////////////////////
// Function prototypes
///////////////////////////////////////////////////////////////////////////////

void initTIM(TIM_TypeDef * TIMx, uint32_t base);
void delay_units(TIM_TypeDef * TIMx, uint32_t us);
void start_timer(TIM_TypeDef *TIMx, uint32_t us);
uint8_t check_timer(TIM_TypeDef *TIMx);
void initPWM(TIM_TypeDef * TIMx, uint32_t base);
void set_PWM_freq(TIM_TypeDef * TIMx, uint32_t f, uint64_t base, uint32_t dc);

#endif