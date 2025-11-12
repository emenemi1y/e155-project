// STM32L432KC_FLASH.c
// Source code for FLASH functions
// Taken from the tutorial interrupt code 
// and then expanded upon by Nina Jobanputra
// 11/10/25
// njobanputra@g.hmc.edu

#include "mcu/STM32L432KC_FLASH.h"

void configureFlash() {
  FLASH->ACR |= FLASH_ACR_LATENCY_4WS;
  FLASH->ACR |= FLASH_ACR_PRFTEN;
}

void unlockFlash() {
// Flash Keys used to unlock the Flash
int FLASH_KEYR_KEY1 = 0x45670123;
int FLASH_KEYR_KEY2 =  0xCDEF89AB;

// Setting the Flash Key to unlock the Flash
FLASH->KEYR |=  FLASH_KEYR_KEY1;
}

void eraseFlash() {
/* pg 84 refernce manual */
//Check that no Flash memory operation is ongoing by checking the BSY bit in the FLASH_SR register.
if(FLASH_SR_BSY == 0){
//Check and clear all error programming flags due to a previous programming. If not, PGSERR is set.

//Set the MER1 bit in the Flash control register (FLASH_CR).
  FLASH->CR |= FLASH_CR_MER1;
// Set the STRT bit in the FLASH_CR register.
FLASH->CR |= FLASH_CR_STRT;
//Wait for the BSY bit to be cleared in the Flash control register (FLASH_CR).

}
}
