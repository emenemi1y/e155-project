// STM32L432KC_FLASH.h
// Header for FLASH functions

#ifndef STM32L4_FLASH_H
#define STM32L4_FLASH_H

#include <stdint.h>
#include <stm32l432xx.h>

#define FLASHBASE (0x08000000)
#define PAGE1 (0x08000800)
#define PAGE255 (0x0807F800)

///////////////////////////////////////////////////////////////////////////////
// Function prototypes
///////////////////////////////////////////////////////////////////////////////

void configureFlash();
void unlockFlash();
void eraseFlash();
static void programFlash(uint32_t Address, uint64_t Data);
void lockFlash();

#endif
