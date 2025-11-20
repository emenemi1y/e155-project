// STM32L432KC_FLASH.c
// Source code for FLASH functions
// Taken from the tutorial interrupt code 
// and then expanded upon by Nina Jobanputra
// 11/10/25
// njobanputra@g.hmc.edu

// References, they haven't really helped so far
// https://github.com/controllerstech/STM32/blob/master/LVGL/Simple_F446/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c#L433
// https://github.com/aloebs29/flash_management


#define USART_ID USART2_ID

// card 1: 33 b2 e5 2e
// card 2: e7 50 89 14

//initUSART(USART_ID, 9600);