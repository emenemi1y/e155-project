// WS2812B.c
// Source code  for WS2812 functions
// Emily Kendrick, ekendrick@hmc.edu, 11/18/2025

#include "WS2812B.h"

void initLED(void){
  pinMode(LED_PIN, GPIO_OUTPUT); 
  digitalWrite(LED_PIN, PIO_LOW);
}

void send1Pulse(void){
  digitalWrite(LED_PIN, PIO_HIGH);
  delay_units(TIM16, T1H/2);
  digitalWrite(LED_PIN, PIO_LOW);
  delay_units(TIM16, T1L/2);
}

void send0Pulse(void){
  digitalWrite(LED_PIN, PIO_HIGH);
  delay_units(TIM16, T0H/2);
  digitalWrite(LED_PIN, PIO_LOW);
  delay_units(TIM16, T0L/2);
}