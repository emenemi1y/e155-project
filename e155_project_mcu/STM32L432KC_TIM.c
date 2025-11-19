// STM32F401RE_TIM.c
// TIM functions
#include "STM32L432KC_TIM.h"


void initTIM(TIM_TypeDef * TIMx, uint32_t base){
  // Set prescaler to give 1 us time base
  uint32_t psc_div = (uint32_t) ((SystemCoreClock/base) - 1);
  printf("psc: %d", (uint32_t) ((SystemCoreClock/base)));

  // Set prescaler division factor
  TIMx->PSC = (psc_div - 1);
  // Generate an update event to update prescaler value
  TIMx->EGR |= 1;
  // Enable counter
  TIMx->CR1 |= 1; // Set CEN = 1
}

void start_timer(TIM_TypeDef *TIMx, uint32_t us){
  TIMx->ARR = us;// Set timer max count
  TIMx->EGR |= 1;     // Force update
  TIMx->SR &= ~(0x1); // Clear UIF
  TIMx->CNT = 0;      // Reset count
}

uint8_t check_timer(TIM_TypeDef *TIMx){
  return TIMx->SR & 1;
}

void delay_units(TIM_TypeDef * TIMx, uint32_t us){
  TIMx->ARR = us;// Set timer max count
  TIMx->EGR |= 1;     // Force update
  TIMx->SR &= ~(0x1); // Clear UIF
  TIMx->CNT = 0;      // Reset count

  while(!(TIMx->SR & 1)); // Wait for UIF to go high
}

void initPWM(TIM_TypeDef * TIMx, uint32_t base){

  // Set prescaler to give 1 MHz
  TIMx->PSC = ((uint32_t) ((SystemCoreClock/base)) - 1);

  // Enable PWM output 
  TIMx->CCMR1 |= (0b110 << 4); // OC1M
  TIMx->CCMR1 |= (0b1 << 3); // OC1PE
  TIMx->CR1 |= (0b1 << 7); // ARPE
  
  // Set all required PWM bits
  TIMx->BDTR |= (1 << 15); // Set MOE 
  TIMx->BDTR |= (1 << 11); // OSSR bit in BDTR
  TIMx->BDTR |= (1 << 10); // OSSI bit in BDTR
  
  // Enable output
  TIMx->CCER |= (1 << 0); // CC1 output enable
  TIMx->CCER |= (1 << 1); // Set polarity to 1
  
  // Reset and enable
  TIMx->EGR |= (1 << 0); // Reset registers by setting update bit
  TIMx->CNT &= ~(0b1111111111111111 << 0); // Reset count
  TIMx->CR1 |= (1 << 0); // Enable counter (CEN)


}


// Set the frequency of the PWM on a timer
// Takes duty cycle on a scale from 1-100
void set_PWM_freq(TIM_TypeDef * TIMx, uint32_t f, uint64_t base, uint32_t on) {
  // Calculate values to get correct frequency
  uint32_t arr_cnt = (uint32_t) ((base/f));
  uint32_t ccr1_cnt = (uint32_t) ((uint64_t) ((on*base)/1e9));
  //printf("\n on: %d", on);
 // printf("\n base: %d", base);
  //printf("\n math: %llu", );
  printf("\n arr_cnt: %d", arr_cnt);
  printf("\n ccr1_cnt: %d", ccr1_cnt);
      
  // Set values for ARR and CCR1
  TIMx->ARR = ((arr_cnt - 1) << 0);
  TIMx->CCR1 = ((ccr1_cnt - 1) << 0);

  TIMx->EGR |= (1 << 0); // Reset registers 
  TIMx->CNT &= ~(0b1111111111111111 << 0); // Reset count

}


