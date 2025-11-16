// STM32L432KC_SPI.c
// Emily Kendrick
// ekendrick@hmc.edu
// 10/19/2025
// Function definitions for SPI peripheral

#include "STM32L432KC_SPI.h"
#include "STM32L432KC_RCC.h"
#include "STM32L432KC_GPIO.h"
#include <stdio.h>

/* Enables the SPI peripheral and intializes its clock speed (baud rate), polarity, and phase.
 *    -- br: (0b000 - 0b111). The SPI clk will be the master clock / 2^(BR+1).
 *    -- cpol: clock polarity (0: inactive state is logical 0, 1: inactive state is logical 1).
 *    -- cpha: clock phase (0: data captured on leading edge of clk and changed on next edge, 
 *          1: data changed on leading edge of clk and captured on next edge)
 * Refer to the datasheet for more low-level details. */ 

void initSPI(int br, int cpol, int cpha){

  // Enable system clock for SPI1
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  
  // Configure GPIO pins for SPI3
  // Set to alternate function mode
  pinMode(SPI_SCK, GPIO_ALT);     // SPI1_SCK
  pinMode(SPI_MISO, GPIO_ALT);    // SPI1_MISO
  pinMode(SPI_MOSI, GPIO_ALT);    // SPI1_MOSI
  pinMode(SPI_CE, GPIO_OUTPUT);   // Manual CS
  
  
  // Set output speed type to high for SCK
  GPIOB->OSPEEDR |= (0b11 << GPIO_OSPEEDR_OSPEED3_Pos);

  // Set to AF05 for SPI alternate functions 
  GPIOB->AFR[0] |= _VAL2FLD(GPIO_AFRL_AFSEL3, 5);
  GPIOB->AFR[0] |= _VAL2FLD(GPIO_AFRL_AFSEL4, 5);
  GPIOB->AFR[0] |= _VAL2FLD(GPIO_AFRL_AFSEL5, 5);

  // Set the baud rate 
  SPI1->CR1 |= _VAL2FLD(SPI_CR1_BR, br);

  // Set to controller configuration
  SPI1->CR1 |= SPI_CR1_MSTR;

  SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_LSBFIRST | SPI_CR1_SSM);

  SPI1->CR1 |= _VAL2FLD(SPI_CR1_CPOL, cpol); // Polarity
  SPI1->CR1 |= _VAL2FLD(SPI_CR1_CPHA, cpha); // Phase
  SPI1->CR2 |= _VAL2FLD(SPI_CR2_DS, 0b0111); // Data length for transfer
  SPI1->CR2 |= (SPI_CR2_FRXTH | SPI_CR2_SSOE);

  // Enable SPI
  SPI1->CR1 |= SPI_CR1_SPE;
  
}


/* Transmits a character (1 byte) over SPI and returns the received character.
 *    -- send: the character to send over SPI
 *    -- return: the character received over SPI */

char spiSendReceive(char send){
  // Wait until the transmit buffer is empty
  while(!(SPI1->SR & SPI_SR_TXE)); 

  // Trasnsmit the character over SPI
  *(volatile char *) (&SPI1->DR) = send;

  // Wait until data has been received and then set data
  while(!(SPI1->SR & SPI_SR_RXNE)); 
  char rec = (volatile char) SPI1->DR;

  return rec; // Return received character
}

// Trying to emulate SPI.begin(), might be wrong 
void beginSPI(void) {
  // Set chip enable to high  
  digitalWrite(SPI_CE, PIO_LOW);
}
