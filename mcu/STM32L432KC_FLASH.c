// STM32L432KC_FLASH.c
// Source code for FLASH functions
// Taken from the tutorial interrupt code 
// and then expanded upon by Nina Jobanputra
// 11/10/25
// njobanputra@g.hmc.edu

// References, they haven't really helped so far
// https://github.com/controllerstech/STM32/blob/master/LVGL/Simple_F446/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c#L433
// https://github.com/aloebs29/flash_management


#include "STM32L432KC_FLASH.h"

void configureFlash() {
  FLASH->ACR |= FLASH_ACR_LATENCY_4WS;
  FLASH->ACR |= FLASH_ACR_PRFTEN;
}

void unlockFlash() {
// Flash Keys used to unlock the Flash
int FLASH_KEYR_KEY1 = 0x45670123;
int FLASH_KEYR_KEY2 =  0xCDEF89AB;

// Setting the Flash Key to unlock the Flash
FLASH->KEYR =  (FLASH_KEYR_KEY1 << 0);
FLASH->KEYR = (FLASH_KEYR_KEY2 << 16);
}

void eraseFlash() {
// pg 84 refernce manual 
//Check that no Flash memory operation is ongoing by checking the BSY bit in the FLASH_SR register.
if (FLASH_SR_BSY == 0) {

//Check and clear all error programming flags due to a previous programming. If not, PGSERR is set.
  FLASH->SR |= _VAL2FLD(FLASH_SR_PROGERR, 1);
  FLASH->SR |= _VAL2FLD(FLASH_SR_SIZERR, 1);
  FLASH->SR |= _VAL2FLD(FLASH_SR_PGAERR, 1);
  FLASH->SR |= _VAL2FLD(FLASH_SR_PGSERR, 1);
  FLASH->SR |= _VAL2FLD(FLASH_SR_WRPERR, 1);
  FLASH->SR |= _VAL2FLD(FLASH_SR_MISERR, 1);
  FLASH->SR |= _VAL2FLD(FLASH_SR_FASTERR, 1);

//Set the PER bit and select the page you wish to erase (PNB) in the Flash control register (FLASH_CR).
  FLASH->CR |= _VAL2FLD(FLASH_CR_PER, 1);
  FLASH->CR |= _VAL2FLD(FLASH_CR_PNB, 11111111); // currently erasing page 255

// Set the STRT bit in the FLASH_CR register.
  FLASH->CR |= _VAL2FLD(FLASH_CR_STRT, 1);

// Wait for the BSY bit to be cleared in the FLASH_SR register.
  while(!(FLASH_SR_BSY));
  }
}

void programFlash(uint32_t Address, uint64_t Data) {
// Check that no Flash main memory operation is ongoing by checking the BSY bit in the Flash status register (FLASH_SR).
if (FLASH_SR_BSY == 0) {
// Check and clear all error programming flags due to a previous programming. If not, PGSERR is set.
  FLASH->SR |= _VAL2FLD(FLASH_SR_PROGERR, 1);
  FLASH->SR |= _VAL2FLD(FLASH_SR_SIZERR, 1);
  FLASH->SR |= _VAL2FLD(FLASH_SR_PGAERR, 1);
  FLASH->SR |= _VAL2FLD(FLASH_SR_PGSERR, 1);
  FLASH->SR |= _VAL2FLD(FLASH_SR_WRPERR, 1);
  FLASH->SR |= _VAL2FLD(FLASH_SR_MISERR, 1);
  FLASH->SR |= _VAL2FLD(FLASH_SR_FASTERR, 1);

// Set the PG bit in the Flash control register (FLASH_CR).
  FLASH->CR |= _VAL2FLD(FLASH_CR_PG, 1);
  }
// Perform the data write operation at the desired memory address, inside main memory block or OTP area. Only double word can be programmed.
  //– Write a first word in an address aligned with double word
  //– Write the second word
  *(__IO uint32_t*)Address = (uint32_t)Data;
  *(__IO uint32_t*)(Address+4U) = (uint32_t)(Data >> 32);

// Wait until the BSY bit is cleared in the FLASH_SR register.
  while(!(FLASH_SR_BSY));
// Check that EOP flag is set in the FLASH_SR register (meaning that the programming operation has succeed), and clear it by software.
  FLASH->SR |= _VAL2FLD(FLASH_SR_EOP, 1);

// Clear the PG bit in the FLASH_CR register if there no more programming request anymore.
  FLASH->CR |= _VAL2FLD(FLASH_CR_PG, 0);
}

void lockFlash(){
// The FLASH_CR register can be locked again by software by setting the LOCK bit in the FLASH_CR register.
if (FLASH_SR_BSY == 0) {
  FLASH->CR |= _VAL2FLD(FLASH_CR_LOCK, 1);
  }
}
