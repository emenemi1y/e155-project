// E155 project main file

#include "STM32L432KC.h"
#include "stm32l4xx_hal.h"



  int main(void) {
  configureFlash();
  configureClock();

  // Enable GPIO
  gpioEnable(GPIO_PORT_A);
  gpioEnable(GPIO_PORT_B);
  gpioEnable(GPIO_PORT_C);

  // Enable TIM15
  RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
  initTIM(TIM15, 1e6);

  // Enable TIM16 for LED strip delays 
  RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
  initTIM(TIM16, 8e7); // Keep 80 MHz clock
  initPWM(TIM16, 8e7);

  // Enable TIM2 for short delays 
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
  initTIM(TIM2, 2e6);

  initSPI(SPI1, 6, CPOL, CPHA); // br of 6 = 80 MHz / 128 = 625 kHz

  PCD_Init();
  delay_units(TIM15, 40);

  initLED();
  printf("Clock frequency: %d", SystemCoreClock);
  
  while(1) {
    digitalWrite(LED_PIN, PIO_HIGH);
    delay_units(TIM2, 16);
    digitalWrite(LED_PIN, PIO_LOW);
    delay_units(TIM2, 9);

    
  }
  


  /*
  PCD_DumpVersionToSerial();
  printf("Scan PICC to see UID, SAK, type, and data blocks...");

  while(1){
    if (PICC_IsNewCardPresent()){
      if(PICC_ReadCardSerial()){
        PICC_DumpToSerial(&(uid));
      }
    
    }

  }
  */

}