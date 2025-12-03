// E155 project main file

#include "STM32L432KC.h"

#define TX PA9
#define RX PA10
#define USART_ID USART1_ID



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
  initTIM(TIM16, 1e3);

  initSPI(SPI1, 3, CPOL, CPHA);
  PCD_Init();
  delay_units(TIM15, 40);


  // Initialize USART
  USART_TypeDef * USART = initUSART(USART_ID, 9600);


  initializeDF3(USART);
  //setVolume(USART, 30);
  pauseSong(USART);
  
  
  uint64_t card_id, prev_id;
  
  PCD_DumpVersionToSerial();
  printf("\nScan PICC to see UID, SAK, type, and data blocks...");
  
  while(1){
    if (PICC_IsNewCardPresent()){
      if(PICC_ReadCardSerial()){
        prev_id = card_id;
        card_id = PICC_DumpToSerial(&(uid));
      }   
      pauseSong(USART);
      playSong(USART, getSongID(card_id));
      //if (card_id == 0x7945dc11) playSong(USART, 0x01);
      //else if (card_id == 0x1c2ae62e) playSong(USART, 0x02);
    }
  }
  

}
