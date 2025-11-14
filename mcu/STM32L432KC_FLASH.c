// STM32L432KC_FLASH.c
// Source code for FLASH functions
// Taken from the tutorial interrupt code 
// and then expanded upon by Nina Jobanputra
// 11/10/25
// njobanputra@g.hmc.edu

#include "C:\Users\njobanputra\Documents\GitHub\e155-project\mcu\STM32L432KC_FLASH.h"

void configureFlash() {
  FLASH->ACR |= FLASH_ACR_LATENCY_4WS;
  FLASH->ACR |= FLASH_ACR_PRFTEN;
}

void unlockFlash() {
// Flash Keys used to unlock the Flash
int FLASH_KEYR_KEY1 = 0x45670123;
int FLASH_KEYR_KEY2 =  0xCDEF89AB;

// Setting the Flash Key to unlock the Flash
FLASH->KEYR =  FLASH_KEYR_KEY1;
FLASH->KEYR = FLASH_KEYR_KEY2;
}

void eraseFlash() {
/* pg 84 refernce manual */
//Check that no Flash memory operation is ongoing by checking the BSY bit in the FLASH_SR register.
while (FLASH_SR_BSY == 0) {

//Check and clear all error programming flags due to a previous programming. If not, PGSERR is set.

//Set the PER bit and select the page you wish to erase (PNB) in the Flash control register (FLASH_CR).
// Set the STRT bit in the FLASH_CR register.
// Wait for the BSY bit to be cleared in the FLASH_SR register.
  }
}

void programFlash() {
// Check that no Flash main memory operation is ongoing by checking the BSY bit in the Flash status register (FLASH_SR).

// Check and clear all error programming flags due to a previous programming. If not, PGSERR is set.

// Set the PG bit in the Flash control register (FLASH_CR).

// Perform the data write operation at the desired memory address, inside main memory block or OTP area. Only double word can be programmed.
  //– Write a first word in an address aligned with double word
  //– Write the second word

// Wait until the BSY bit is cleared in the FLASH_SR register.

// Check that EOP flag is set in the FLASH_SR register (meaning that the programming operation has succeed), and clear it by software.
// Clear the PG bit in the FLASH_CR register if there no more programming request anymore.
}

void lockFlash(){
}