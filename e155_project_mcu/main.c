// E155 project main file

#include "STM32L432KC.h"



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
  initTIM(TIM16, 2e7); // base unit of 50 ns

  initSPI(SPI1, 6, CPOL, CPHA); // br of 6 = 80 MHz / 128 = 625 kHz

  PCD_Init();
  delay_units(TIM15, 40);

  initLED();

  while(1) {
    send1Pulse();
    send0Pulse();

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