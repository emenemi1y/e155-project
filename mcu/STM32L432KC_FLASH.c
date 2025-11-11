// STM32L432KC_FLASH.c
// Source code for FLASH functions
// Taken from the tutorial interrupt code 
// and then expanded upon by Nina Jobanputra
// 11/10/25
// njobanputra@g.hmc.edu

#include "STM32L432KC_FLASH.h"

void configureFlash() {
  FLASH->ACR |= FLASH_ACR_LATENCY_4WS;
  FLASH->ACR |= FLASH_ACR_PRFTEN;
}

void unlockFlash() {

int FLASH_KEYR_KEY1 = 0x45670123;
int FLASH_KEYR_KEY2 =  0xCDEF89AB;

FLASH->KEYR |=  0x45670123;
}

void eraseFlash() {
/* pg 84 refernce manual
1. Check that no Flash memory operation is ongoing by checking the BSY bit in the
FLASH_SR register.
2. Check and clear all error programming flags due to a previous programming. If not,
PGSERR is set.
3. Set the MER1 bit in the Flash control register (FLASH_CR).
4. Set the STRT bit in the FLASH_CR register.
5. Wait for the BSY bit to be cleared in the Flash control register (FLASH_CR).
*/
}
