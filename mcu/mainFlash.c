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

#define FLASH_BASE        0x08000000UL
#define FLASH_PAGE_SIZE   0x800UL          // 2 KB
#define FLASH_PAGE_COUNT  128U             // 256 KB total
#define FLASH_PAGE_127    (FLASH_PAGE_COUNT - 1U)
#define PAGE127_ADDR      (FLASH_BASE + FLASH_PAGE_127 * FLASH_PAGE_SIZE)
// == 0x0803F800

// Simple delay
void delay(volatile uint32_t count) {
    while(count--);
}

int main(void) {
    volatile uint32_t cr_initial, sr_initial;
    volatile uint32_t cr_after_unlock, sr_after_unlock;
    volatile uint32_t cr_after_erase, sr_after_erase;
    volatile uint32_t cr_after_program, sr_after_program;
    volatile uint64_t value_initial, value_after_erase, value_after_program;
    
    // Configure flash and enable flash interface clock
    RCC->AHB1ENR |= RCC_AHB1ENR_FLASHEN;
    configureFlash();
    
    // 1. Read initial state
    cr_initial = FLASH->CR;
    sr_initial = FLASH->SR;
    value_initial = *(volatile uint64_t*)PAGE127_ADDR;
    
    // 2. Unlock Flash
    unlockFlash();
    delay(1000);
    cr_after_unlock = FLASH->CR;
    sr_after_unlock = FLASH->SR;
    
    // Check if unlock worked - bit 31 should be 0
    if(cr_after_unlock & FLASH_CR_LOCK) {
        // Still locked - stop here
        while(1) {
            // Set breakpoint here - flash didn't unlock
            __NOP();
        }
    }
    
    // 3. Erase page
    eraseFlash(PAGE127_ADDR);
    delay(10000);
    cr_after_erase = FLASH->CR;
    sr_after_erase = FLASH->SR;
    value_after_erase = *(volatile uint64_t*)PAGE127_ADDR;
    
    // After erase, value should be 0xFFFFFFFFFFFFFFFF
    if(value_after_erase != 0xFFFFFFFFFFFFFFFFULL) {
        // Erase didn't work - stop here
        while(1) {
            // Set breakpoint here - erase failed
            __NOP();
        }
    }
    
    // 4. Program Flash
    programFlash(PAGE127_ADDR, 0x1234567890ABCDEFULL);
    delay(10000);
    cr_after_program = FLASH->CR;
    sr_after_program = FLASH->SR;
    value_after_program = *(volatile uint64_t*)PAGE127_ADDR;
    
    // Check if programming worked
    if(value_after_program != 0x1234567890ABCDEFULL) {
        // Programming didn't work - stop here
        while(1) {
            // Set breakpoint here - programming failed
            __NOP();
        }
    }
    
    // 5. Lock Flash
    lockFlash();
    
    // Success!
    while(1) {
        // Set breakpoint here - all operations succeeded!
        // Inspect variables in debugger to see values
        __NOP();
    }
}