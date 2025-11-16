// MFRC522.h
// Header for MFRC522 functions
// Emily Kendrick, ekendrick@hmc.edu, 11/15/2025

#ifndef MFRC522_h

#include <stdint.h> // Include stdint header
#include <stm32l432xx.h>
#include "STM32L432KC_GPIO.h"
#include "STM32L432KC_SPI.h"

///////////////////////////////////////////////////////////////////////////////
// Definitions
///////////////////////////////////////////////////////////////////////////////

#define RST_PIN   PA8
#define SS_PIN    PA5
#define CPOL      0
#define CPHA      0


void setup(void);
void PCD_Init(void);
void PCD_Reset(void);


#endif