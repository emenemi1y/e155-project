// STM32L432KC.h
// Header to include all other STM32L432KC libraries.

#ifndef STM32L4_H
#define STM32L4_H


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stm32l432xx.h>

// Include other peripheral libraries
#include "/Users/ninajobanputra/Documents/SEGGER Embedded Studio Projects/E155Project/STM32L432KC_GPIO.h"
#include "/Users/ninajobanputra/Documents/E155Project/mcu/STM32L432KC_RCC.h"
#include "/Users/ninajobanputra/Documents/E155Project/mcu/STM32L432KC_TIM.h"
#include "/Users/ninajobanputra/Documents/E155Project/mcu/STM32L432KC_FLASH.h"
#include "/Users/ninajobanputra/Documents/E155Project/mcu/STM32L432KC_UART.h"
#include "/Users/ninajobanputra/Documents/E155Project/e155_project_mcu/STM32L432KC_SPI.h"

// Global defines
#define HSI_FREQ 16000000 // HSI clock is 16 MHz
#define MSI_FREQ 4000000  // HSI clock is 4 MHz
#define MCK_FREQ 100000

#define USART_ID USART1_ID
#define TIM TIM15

//#define TX PA9
//#define RX PA10
//#define RST PB6
//#define SDA PA5
//#define SCK PB3
//#define MOSI PB5
//#define MISO PB4

#define SPI_CE PA1
#define SPI_SCK PA5
#define SPI_MOSI PA7
#define SPI_MISO PA11
#define LOAD_PIN PA2
#define FPGA_DONE PA3

// Flash Memory Configurtion
#define FLASH_BASE        0x08000000UL
#define FLASH_PAGE_SIZE   0x800UL          // 2 KB
#define FLASH_PAGE_COUNT  128U             // 256 KB total

// Defining the page addresses for the lightshow
#define FLASH_PAGE_127    (FLASH_PAGE_COUNT - 1U)
#define PAGE127_ADDR      (FLASH_BASE + FLASH_PAGE_127 * FLASH_PAGE_SIZE)

#define FLASH_PAGE_126    (FLASH_PAGE_COUNT - 2U)
#define PAGE126_ADDR      (FLASH_BASE + FLASH_PAGE_126 * FLASH_PAGE_SIZE)

#define FLASH_PAGE_125    (FLASH_PAGE_COUNT - 3U)
#define PAGE125_ADDR      (FLASH_BASE + FLASH_PAGE_125 * FLASH_PAGE_SIZE)

#endif