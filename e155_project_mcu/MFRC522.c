// MFRC522.c
// Source code  for MFRC522 functions
// Emily Kendrick, ekendrick@hmc.edu, 11/15/25

#include "MFRC522.h"
#include "STM32L432KC_SPI.h"
#include "STM32L432KC_TIM.h"

void setup(void) {
  // Initialize SPI communication 
  initSPI(0b101, CPOL, CPHA);
  beginSPI();

}

void PCD_Init(void)
{

    int hardReset = 0;

    // Set resetPowerDownPin as digital input
    pinMode(RST_PIN, GPIO_INPUT);
    if (digitalRead(RST_PIN) == PIO_LOW) { // if MFRC522 chip is in power down mode
        pinMode(RST_PIN, GPIO_OUTPUT);       // Set reset pin as digitial output
        digitalWrite(RST_PIN, PIO_LOW);      // Make sure there is a clean LOW state
        delay_micros(TIM15, 2);              // Reset timing requirements indicate 100 ns, but allow extra time to be safe
        digitalWrite(RST_PIN , PIO_HIGH);    // Exit power down mode, triggering a hard reset.

        delay_micros(TIM15, 50);
        hardReset = 1;
    }

    // Perform a soft reset if no hard reset from above.
    if (!hardReset) { 
      PCD_Reset();
    }

    // Reset baud rates

    // Reset ModWidthReg


    // Need timeout if communicating with a PICC (proximity integrated circuit card) if something goes wrong
  
  
}


void PCD_Reset(void){
  // PCD = proximity coupling device
  PCD_WriteRegister(

}