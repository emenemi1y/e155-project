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
  initTIM(TIM15, 1e3);

  //initSPI(SPI1, 3, CPOL, CPHA);
  //PCD_Init();
  delay_units(TIM15, 40);


  // Initialize USART
  USART_TypeDef * USART = initUSART(USART_ID, 9600);


  //initializeDF3(USART);
  //while(1){
  
  sendChar(USART, 0x7E);
  sendChar(USART, 0xFF); // Version
  sendChar(USART, 0x06); // Length
  sendChar(USART, 0x09); // Command: Select device
  sendChar(USART, 0x00); // Feedback
  sendChar(USART, 0x00); // Parameter: TF card
  sendChar(USART, 0x02); // Checksum high byte
  sendChar(USART, 0xEF); // End byte
  delay_units(TIM15, 2000);
 // }
  while(1) {
  sendChar(USART, 0x7E);
  sendChar(USART, 0xFF); // Version
  sendChar(USART, 0x06); // Length
  sendChar(USART, 0x12); // Command: Select device
  sendChar(USART, 0x00); // Feedback
  sendChar(USART, 0x00); // Parameter: TF card
  sendChar(USART, 0x01); // Checksum high byte
  sendChar(USART, 0xEF); // End byte
  delay_units(TIM15, 2000);
  }
  
  /*
  uint64_t card_id, prev_id;
  
  PCD_DumpVersionToSerial();
  printf("\nScan PICC to see UID, SAK, type, and data blocks...");
  
  while(1){
    if (PICC_IsNewCardPresent()){
      if(PICC_ReadCardSerial()){
        card_id = PICC_DumpToSerial(&(uid));
      }   
      //pauseSong(USART);
      //if (card_id == 0x7945dc11) playSong(USART, 0x01);
      //else if (card_id == 0x1c2ae62e) playSong(USART, 0x02);
    }
    playSong(USART, 0x01);
    //printf("\ncurrent ID: %x", (int) card_id);
    
  }
  */

}
