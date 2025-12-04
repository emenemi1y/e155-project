#include <stdio.h>
#include "STM32L432KC.h"
#include <stm32l432xx.h>


// Function used by printf to send characters to the laptop
int _write(int file, char *ptr, int len) {
  int i = 0;
  for (i = 0; i < len; i++) {
    ITM_SendChar((*ptr++));
  }
  return len;
}

////////////////////////////////////////////////
// Function Prototypes
////////////////////////////////////////////////

void readSendFlash(uint32_t base_addr, uint32_t length);

////////////////////////////////////////////////
// Main
////////////////////////////////////////////////

// Simple delay
void delay(volatile uint32_t count) {
    while(count--);
}

int main(void) {

    // Variables to help see if FLASH is being programmed correctly
    volatile uint32_t cr_initial, sr_initial;
    volatile uint32_t cr_after_unlock, sr_after_unlock;
    volatile uint32_t cr_after_erase, sr_after_erase;
    volatile uint32_t cr_after_program, sr_after_program;
    volatile uint64_t value_initial, value_after_erase, value_after_program, value_after_program1, value_after_program2;

  // Enable GPIOA clock
  RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN);
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
    eraseFlash(PAGE126_ADDR);
    eraseFlash(PAGE125_ADDR);
    delay(10000);

    // Getting debug variables set
    cr_after_erase = FLASH->CR;
    sr_after_erase = FLASH->SR;
    value_after_erase = *(volatile uint64_t*)PAGE127_ADDR;
    
    delay(20000);
    // After erase, value should be 0xFFFFFFFFFFFFFFFF
    if(value_after_erase != 0xFFFFFFFFFFFFFFFFULL) {
        // Erase didn't work - stop here
        while(1) {
            // Set breakpoint here - erase failed
            __NOP();
        }
    }
    
    // 4. Program Flash

    // Programming the 1st string of lights into page 127
    programFlash(PAGE127_ADDR, 0x1CFC039FF52FF5FCULL);
    programFlash(PAGE127_ADDR+8U, 0x17FC5C178D17FC17ULL);
    programFlash(PAGE127_ADDR+16U, 0x9DFCFFFFFFFFFFFFULL);

    // Programming the 2nd string of lights into page 126
    programFlash(PAGE126_ADDR, 0x7F32A87F32A87F32ULL);
    programFlash(PAGE126_ADDR+8U, 0xA89FF52F9FF52F9FULL);
    programFlash(PAGE126_ADDR+16U, 0xF52FFFFFFFFFFFFFULL);

    // Programming the 3rd string of lights into page 125
    programFlash(PAGE126_ADDR,    0x3474EB6DCBE33474ULL);
    programFlash(PAGE126_ADDR+8U, 0xEB6DCBE33474EB6DULL);
    programFlash(PAGE126_ADDR+16U, 0xCBE3FFFFFFFFFFFFULL);
  
    delay(10000);

    // Getting debug variables set
    cr_after_program = FLASH->CR;
    sr_after_program = FLASH->SR;
    value_after_program = *(volatile uint64_t*)PAGE126_ADDR;
    value_after_program1 = *(volatile uint64_t*)(PAGE126_ADDR+8U);
    value_after_program2 = *(volatile uint64_t*)(PAGE126_ADDR+16U);
    
    // 5. Lock Flash
    lockFlash();
    
    //// Success!
    //while(1) {
    //    // Set breakpoint here - all operations succeeded!
    //    // Inspect variables in debugger to see values
    //    __NOP();
    //}


  // "clock divide" = master clock frequency / desired baud rate
  // the phase for the SPI clock is 1 and the polarity is 0
  initSPI(0, 0, 0b101);


  // Load and done pins
  pinMode(LOAD_PIN, GPIO_OUTPUT);  // LOAD
  pinMode(FPGA_DONE, GPIO_INPUT);   // DONE
  

  // Artificial chip select signal to allow 8-bit CE-based SPI decoding on the logic analyzers.
  pinMode(SPI_CE, GPIO_OUTPUT);
  digitalWrite(SPI_CE, 0);

  gpioAFSel(SPI_SCK, 5);
  gpioAFSel(SPI_MISO, 5);
  gpioAFSel(SPI_MOSI, 5);


  // sending FLASH data
  readSendFlash(PAGE127_ADDR, 144);
}

////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////


void readSendFlash(uint32_t base_addr, uint32_t length) {

    // Treat flash contents as 16-bit words
    volatile uint16_t *src = (volatile uint16_t *)base_addr;
    uint32_t num_words = length / 2;   // 2 bytes per 16-bit word

    // Assert LOAD high to indicate to the FPGA that a new frame is starting
    digitalWrite(LOAD_PIN, 1);

    for (uint32_t i = 0; i < num_words; i++) {

        uint16_t word = src[i];

        // Wait until FPGA indicates it is ready for next 16-bit word
        while (!digitalRead(FPGA_DONE));

        // Start this 16-bit transaction
        digitalWrite(SPI_CE, 1);   // CE high

        // Send 16 bits as two 8-bit frames (MSB first, then LSB)
        uint8_t high = (word >> 8) & 0xFF;
        uint8_t low  =  word       & 0xFF;

        spiSendReceive(high);
        spiSendReceive(low);

        // Confirm all SPI transactions are completed
        while (SPI1->SR & SPI_SR_BSY);

        // End transaction
        digitalWrite(SPI_CE, 0);   // CE low
    }

    // De-assert LOAD when done
    digitalWrite(LOAD_PIN, 0);
}