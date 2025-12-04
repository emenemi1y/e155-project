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
#include "FLASH_mcu/STM32L432KC.h"

void configureFlash(void) {
  FLASH->ACR |= FLASH_ACR_LATENCY_4WS;
  FLASH->ACR |= FLASH_ACR_PRFTEN;
}

void unlockFlash(void) {
  // Check if already unlocked
  if(!_FLD2VAL(FLASH_CR_LOCK, FLASH->CR)) {
    return; // Already unlocked
  }
  
  // Flash Keys used to unlock the Flash (from reference manual)
  FLASH->KEYR = 0x45670123;
  FLASH->KEYR = 0xCDEF89AB;
}

void eraseFlash(uint32_t address) {
  // Convert address to page number (each page is 2KB = 0x800 bytes)
  // Page number = (address - flash_base) / page_size
  uint32_t page_number = (address - FLASH_BASE) / FLASH_PAGE_SIZE;
  
  // Wait for any ongoing operation to complete
 while (FLASH->SR & FLASH_SR_BSY);

  // Clear all error flags by writing 1 to them
  FLASH->SR |= _VAL2FLD(FLASH_SR_PROGERR, 1) | 
               _VAL2FLD(FLASH_SR_SIZERR, 1) | 
               _VAL2FLD(FLASH_SR_PGAERR, 1) | 
               _VAL2FLD(FLASH_SR_PGSERR, 1) | 
               _VAL2FLD(FLASH_SR_WRPERR, 1) | 
               _VAL2FLD(FLASH_SR_MISERR, 1) | 
               _VAL2FLD(FLASH_SR_FASTERR, 1) | 
               _VAL2FLD(FLASH_SR_EOP, 1);

  // Clear PNB field, set PER bit and page number
  FLASH->CR = (FLASH->CR & ~FLASH_CR_PNB_Msk) | 
              _VAL2FLD(FLASH_CR_PER, 1) | 
              _VAL2FLD(FLASH_CR_PNB, page_number);

  // Start the erase operation
  FLASH->CR |= _VAL2FLD(FLASH_CR_STRT, 1);

  // Wait for operation to complete
  while(_FLD2VAL(FLASH_SR_BSY, FLASH->SR));
  
  // Check for and clear EOP flag
  if(_FLD2VAL(FLASH_SR_EOP, FLASH->SR)) {
    FLASH->SR |= _VAL2FLD(FLASH_SR_EOP, 1);
  }
  
  // Clear PER bit
  FLASH->CR &= ~_VAL2FLD(FLASH_CR_PER, 1);
}

void programFlash(uint32_t Address, uint64_t Data) {
  // Wait for any ongoing operation to complete
  while(_FLD2VAL(FLASH_SR_BSY, FLASH->SR));
  
  // Clear all error flags by writing 1 to them
  FLASH->SR |= _VAL2FLD(FLASH_SR_PROGERR, 1) | 
               _VAL2FLD(FLASH_SR_SIZERR, 1) | 
               _VAL2FLD(FLASH_SR_PGAERR, 1) | 
               _VAL2FLD(FLASH_SR_PGSERR, 1) | 
               _VAL2FLD(FLASH_SR_WRPERR, 1) | 
               _VAL2FLD(FLASH_SR_MISERR, 1) | 
               _VAL2FLD(FLASH_SR_FASTERR, 1) | 
               _VAL2FLD(FLASH_SR_EOP, 1);

  // Set the PG bit to enable programming
  FLASH->CR |= _VAL2FLD(FLASH_CR_PG, 1);
  
  // Perform the data write operation (double word - 64 bits)
  // CRITICAL: Must write as double word (2 words back-to-back)
  // Write first word (lower 32 bits)
  *(__IO uint32_t*)Address = (uint32_t)Data;
  // Write second word (upper 32 bits) immediately after
  *(__IO uint32_t*)(Address + 4) = (uint32_t)(Data >> 32);

  // Wait for operation to complete
  while(_FLD2VAL(FLASH_SR_BSY, FLASH->SR));
  
  // Check if EOP flag is set (end of operation)
  if(_FLD2VAL(FLASH_SR_EOP, FLASH->SR)) {
    FLASH->SR |= _VAL2FLD(FLASH_SR_EOP, 1);
  }

  // Clear the PG bit
  FLASH->CR &= ~_VAL2FLD(FLASH_CR_PG, 1);
}

void lockFlash(void){
  // Wait for any ongoing operation to complete
  while(_FLD2VAL(FLASH_SR_BSY, FLASH->SR));
  
  // Set LOCK bit to lock the Flash
  FLASH->CR |= _VAL2FLD(FLASH_CR_LOCK, 1);
}
