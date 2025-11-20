/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------
 
File    : main.c
Purpose : Generic application start

*/

#include <stdio.h>
#include "STM32L432KC.h"
#include <stm32l432xx.h>
#define USART_ID USART1_ID
#define TIM TIM15

//#define PAGE255 (0x0807F800)

/*********************************************************************
*
*       main()
*
*  Function description
*   Application entry point.
*/

#define TX PA9
#define RX PA10

int main(void) {

uint32_t uid = 0x33b2e52e;
// Configure flash and clock
configureFlash();
configureClock();

// Set pins
gpioEnable(GPIO_PORT_A);
gpioEnable(GPIO_PORT_B);
gpioEnable(GPIO_PORT_C);

pinMode(TX, GPIO_ALT);
gpioAFSel(TX, 7);

pinMode(RX, GPIO_ALT);
gpioAFSel(RX, 7);



// Initialize USART
USART_TypeDef * USART = initUSART(USART_ID, 9600);

// Initialize timer
RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
initTIM(TIM);

// https://docs.cirkitdesigner.com/component/075ed813-0874-84b0-f20d-f5b9718d962e/df-player-mini
// https://picaxe.com/docs/spe033.pdf
//uint8_t msg = 0x7E;
//sendChar(USART, 0x7E);
//sendChar(USART, 0xFF); // Version
//sendChar(USART, 0x06); // Length
//sendChar(USART, 0x0C); // Command: Select device
//sendChar(USART, 0x00); // Feedback
//sendChar(USART, 0x02); // Parameter: TF card
//sendChar(USART, 0xFE); // Checksum high byte
//sendChar(USART, 0xED); // Checksum low byte
//sendChar(USART, 0xEF); // End byte
//delay_millis(TIM, 2000);

//sendChar(USART, 0x7E);
//sendChar(USART, 0xFF); // Version
//sendChar(USART, 0x06); // Length
//sendChar(USART, 0x09); // Command: Select device
//sendChar(USART, 0x00); // Feedback
//sendChar(USART, 0x02); // Parameter: TF card
//sendChar(USART, 0xFE); // Checksum high byte
//sendChar(USART, 0xED); // Checksum low byte
//sendChar(USART, 0xEF); // End byte
//delay_millis(TIM, 2000);
//    sendChar(USART, 0x7E);
//    sendChar(USART, 0xFF);
//    sendChar(USART, 0x06);
//    sendChar(USART, 0x0E);
//    sendChar(USART, 0x00);
//    sendChar(USART, 0x00);
//    sendChar(USART, 0x01);
//    sendChar(USART, 0xFE);
//    sendChar(USART, 0xF7);
//    sendChar(USART, 0xEF);
//    delay_millis(TIM, 2000);
///*
sendChar(USART, 0x7E);
sendChar(USART, 0xFF); // Version
sendChar(USART, 0x06); // Length
sendChar(USART, 0x09); // Command: Select device
sendChar(USART, 0x00); // Feedback
sendChar(USART, 0x00); // Parameter: TF card
sendChar(USART, 0x02); // Checksum high byte
//sendChar(USART, 0xED); // Checksum low byte
sendChar(USART, 0xEF); // End byte
delay_millis(TIM, 2000);


//while(1){

  //if(uid == 0x33b2e52e) {
  // Tells the player to play file 001.mp3
while(1) {
    sendChar(USART, 0x7E);
    sendChar(USART, 0xFF);
    sendChar(USART, 0x06);
    sendChar(USART, 0x12);
    sendChar(USART, 0x00);
    sendChar(USART, 0x00);
    sendChar(USART, 0x02);
    //sendChar(USART, 0xFF);
    //sendChar(USART, 0xE6);
    sendChar(USART, 0xEF);
    delay_millis(TIM, 2000);
  }
    
    //sendChar(USART, 0x7E);
    //sendChar(USART, 0xFF);
    //sendChar(USART, 0x06);
    //sendChar(USART, 0x0E);
    //sendChar(USART, 0x00);
    //sendChar(USART, 0x00);
    //sendChar(USART, 0x00);
    ////sendChar(USART, 0xFF);
    ////sendChar(USART, 0xE6);
    //sendChar(USART, 0xEF);
    //delay_millis(TIM, 2000);
  //}
//}
}
/*************************** End of file ****************************/
