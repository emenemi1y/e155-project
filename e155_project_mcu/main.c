// E155 project main file

#include "STM32L432KC.h"


// TODO: Initialize TIM15 for delays 



  int main(void) {
  configureFlash();
  configureClock();

  // Enable GPIO
  gpioEnable(GPIO_PORT_A);
  gpioEnable(GPIO_PORT_B);
  gpioEnable(GPIO_PORT_C);

  // Enable TIM15
  RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
  initTIM(TIM15);

  initSPI(SPI1, 3, CPOL, CPHA);

  PCD_Init();
  delay_micros(TIM15, 40);
  PCD_DumpVersionToSerial();
  printf("Scan PICC to see UID, SAK, type, and data blocks...");

  while(1){
    if (PICC_IsNewCardPresent()){
      if(PICC_ReadCardSerial()){

        PICC_DumpToSerial(&(uid));

      }
    
    }

  }

}