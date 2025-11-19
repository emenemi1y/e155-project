// WS2812B.h
// Header for WS2812 functions
// Emily Kendrick, ekendrick@hmc.edu, 11/18/2025

#ifndef WS2812B_H

#include <stdint.h> // Include stdint header
#include <stm32l432xx.h>
#include "STM32L432KC_GPIO.h"
#include "STM32L432KC_TIM.h"


// All of the variables defined below are in units of 50 ns (so T0H = 7 * 50 ns = 0.35 us)
#define T0H 7 
#define T1H 14 
#define T0L 16 
#define T1L 12 
#define RES 1 

#define LED_PIN PB0

void initLED(void);
void send1Pulse(void);
void send0Pulse(void);

#endif