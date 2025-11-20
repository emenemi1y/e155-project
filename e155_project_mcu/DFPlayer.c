// STM32L432KC_FLASH.c
// Source code for FLASH functions
// Taken from the tutorial interrupt code 
// and then expanded upon by Nina Jobanputra
// 11/10/25
// njobanputra@g.hmc.edu

// References, they haven't really helped so far
// https://github.com/controllerstech/STM32/blob/master/LVGL/Simple_F446/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c#L433
// https://github.com/aloebs29/flash_management


#include "DFPlayer.h"

#define USART_ID USART2_ID

// card 1: 33 b2 e5 2e
// card 2: e7 50 89 14

void initializeDF3 (USART_TypeDef * USART){
     // Initialization:
    sendChar(USART, 0x7E);
    sendChar(USART, 0xFF); // Version
    sendChar(USART, 0x06); // Length
    sendChar(USART, 0x09); // Command: Select device
    sendChar(USART, 0x01); // Feedback
    sendChar(USART, 0x00); // Parameter: TF card
    sendChar(USART, 0x02); // Checksum high byte
    sendChar(USART, 0xEF); // End byte
    delay_units(TIM15, 50000);
      
}

void playSong (USART_TypeDef * USART, uint8_t song){
      sendChar(USART, 0x7E);
      sendChar(USART, 0xFF); // Version
      sendChar(USART, 0x06); // Length
      sendChar(USART, 0x12); // Command: Select device
      sendChar(USART, 0x00); // Feedback
      sendChar(USART, 0x00); // Parameter: TF card
      sendChar(USART, song); // Checksum high byte
      sendChar(USART, 0xEF); // End byte
      delay_units(TIM15, 50000);
}

void pauseSong(USART_TypeDef * USART) {
      sendChar(USART, 0x7E);
      sendChar(USART, 0xFF); // Version
      sendChar(USART, 0x06); // Length
      sendChar(USART, 0x0E); // Command: Select device
      sendChar(USART, 0x00); // Feedback
      sendChar(USART, 0x00); // Parameter: TF card
      sendChar(USART, 0x00); // Checksum high byte
      sendChar(USART, 0xEF); // End byte
      delay_units(TIM15, 50000);
}

void setVolume(USART_TypeDef * USART, uint8_t volume) {

      sendChar(USART, 0x7E);
      sendChar(USART, 0xFF); // Version
      sendChar(USART, 0x06); // Length
      sendChar(USART, 0x06); // Command: Select device
      sendChar(USART, 0x00); // Feedback
      sendChar(USART, 0x00); // Parameter: TF card
      sendChar(USART, volume); // Checksum high byte
      sendChar(USART, 0xEF); // End byte
      delay_units(TIM15, 50000);

}