// STM32L432KC_FLASH.h
// Header for FLASH functions

#ifndef STM32L4_FLASH_H
#define STM32L4_FLASH_H

#include <stdint.h>
#include <stm32l432xx.h>
#include "FLASH_mcu/STM32L432KC.h"

#define FLASH_BASE      0x08000000UL
#define FLASH_PAGE_SIZE 0x800UL       // 2 KB
#define FLASH_PAGE_COUNT 128U         // for STM32L432KC
#define FLASH_PAGE_127    (FLASH_PAGE_COUNT - 1U)

///////////////////////////////////////////////////////////////////////////////
// Function prototypes
///////////////////////////////////////////////////////////////////////////////

void configureFlash(void);
void unlockFlash(void);
void eraseFlash(uint32_t address);
void programFlash(uint32_t Address, uint64_t Data);
void lockFlash(void);

#endif
