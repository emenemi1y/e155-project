// WS2812B.h
// Header for WS2812 functions
// Emily Kendrick, ekendrick@hmc.edu, 11/18/2025

#ifndef WS2812B_H

#include <stdint.h> // Include stdint header
#include <stm32l432xx.h>
#include "STM32L432KC_GPIO.h"
#include "STM32L432KC_TIM.h"


// All of the variables defined below are in units of 50 ns (so T0H = 7 * 50 ns = 0.35 us)
#define T0H 400 //ns
#define T1H 800 //ns
#define T0L 850 //ns
#define T1L 450 //ns
#define RES 50  //us 
#define LED_PIN PA8

#define BUFFER_SIZE 24
#define LED_OFF 1*LED_CNT / 3 - 1 // slightly less than 1/3
#define LED_ON 2*LED_CNT/3 + 2 // slightly more than 2/3
#define LED_RESET_CYCLES 10 // Full 24-bit cycles

#define GL 0 // GREEN
#define RL 1 // RED
#define BL 2 // BLUE 

#define LED_CNT 100

typedef enum {
  WS2812_Ok,
  WS2812_Err,
  WS2812_Mem
} ws2812_resultTypeDef;

// statemachine states
typedef enum {
  LED_RES = 0, // Reset latch cycle
  LED_IDL = 1, // Idle doing nothing except waiting for is_dirty
  LED_DAT = 2, // Transferring led data - one led at a time
} ws2812_stateTypeDef;

typedef struct {
  TIM_TypeDef *timer;   // Timer running the pwm must run at 800 kHz
  uint32_t channel;           // Timer channel
  uint16_t dma_bugger[BUFFER_SIZE * 2]; // Fixed size DMA buffer
  uint16_t leds;              // Number of LEDs on the string
  uint8_t *led;               // Dynamically allocated array of LED RGB values
  ws2812_stateTypeDef led_state; // LED Transfer state machine
  uint8_t led_cnt;            // Counts through the leds starting from zero up to "leds"
  uint8_t res_cnt;            // Counts reset cycles when in reset state
  uint8_t is_dirty;           // Indicates to the call back that the led color a values have been updated
  uint8_t zero_halves;        // counts halves send during reset
  uint32_t dma_cbs;           // just used for statistics
  uint32_t dat_cbs;           // also used for statistics
} ws2812_handleTypeDef;

ws2812_resultTypeDef ws2812_init(ws2812_handleTypeDef *ws2812, TIM_TypeDef *timer, uint32_t channel, uint16_t leds);

void ws2812_update_buffer(ws2812_handleTypeDef *ws2812, uint16_t *dma_buffer_pointer);

// Set all led values to zero
ws2812_resultTypeDef zeroLedValues(ws2812_handleTypeDef *ws2812);

// Set a single led value
ws2812_resultTypeDef setLedValue(ws2812_handleTypeDef *ws2812, uint16_t led, uint8_t color, uint8_t value);

// Set values of all 3 leds 
ws2812_resultTypeDef setLedValues(ws2812_handleTypeDef *ws2812, uint16_t led, uint8_t r, uint8_t g, uint8_t b);





void initLED(void);
void send1Pulse(void);
void send0Pulse(void);
void sendVals(uint32_t bits);

#endif