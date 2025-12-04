// E155 project main file

#include "STM32L432KC.h"
#define USART_ID USART1_ID

int main(void) {
  configureFlash();
  configureClock();

  // Enable GPIO
  gpioEnable(GPIO_PORT_A);
  gpioEnable(GPIO_PORT_B);
  gpioEnable(GPIO_PORT_C);

  // GPIO pin for telling FPGA to start 
  pinMode(FPGA_PIN, GPIO_OUTPUT);
  digitalWrite(FPGA_PIN, PIO_LOW);
  pinMode(BUSY_PIN, GPIO_INPUT);

  // Enable TIM15
  RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
  RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
  initTIM(TIM15, 1e6);
  initTIM(TIM16, 1e3);

  initSPI(SPI1, 3, CPOL, CPHA);
  PCD_Init();
  delay_units(TIM15, 40);


  // Initialize USART
  USART_TypeDef * USART = initUSART(USART_ID, 9600);


  initializeDF3(USART);
  setVolume(USART, 20);
  pauseSong(USART);
  
  
  uint64_t card_id, prev_id;
  
  PCD_DumpVersionToSerial();
  printf("\nScan PICC to see UID, SAK, type, and data blocks...");
  
  while(1){
    if (PICC_IsNewCardPresent()){
      
      if(PICC_ReadCardSerial()){
        prev_id = card_id;
        card_id = PICC_DumpToSerial(&(uid));
        //digitalWrite(FPGA_PIN, PIO_HIGH);
        //delay_units(TIM15, 30);
        //digitalWrite(FPGA_PIN, PIO_LOW);
      } 
      
      if (getSongID(card_id) != 0x00){
        digitalWrite(FPGA_PIN, PIO_HIGH);
        delay_units(TIM15, 30);
        digitalWrite(FPGA_PIN, PIO_LOW);
      }
      pauseSong(USART);
      playSong(USART, 0x1e);
      delay_units(TIM16, 12000);
      playSong(USART, getSongID(card_id));
      
      
    }
  }
  

}