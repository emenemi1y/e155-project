// WS2812B.c
// Source code  for WS2812 functions
// Emily Kendrick, ekendrick@hmc.edu, 11/18/2025

#include "WS2812B.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "color_values.h"


volatile uint8_t finished;

#define Num_of_LEDs 144
#define CH6 ((0x06 << 25))
#define AF01 (0x01)

// Update next 24 bits in the dma buffer, assume dma_buffer_pointer is pointing to the buffer
// that is safe to update. The dma_buffer_pointer and the call to this function is handled by the 
// dma callbacks
inline void ws2812_update_buffer(ws2812_handleTypeDef *ws2812, uint16_t *dma_buffer_pointer) {
  #ifdef BUFF_GPIO_Port

  #endif

  // state machine, either resettting (two buffers worth of zeros),
  // idle (zero buffers) or 
  // transmistting data for the "current" led.

  ++ws2812->dma_cbs;
  if(ws2812->led_state == LED_RES) { // Latch state - 10 or more full buffers of zeros
      if (ws2812->zero_halves < 2) {
        memset(dma_buffer_pointer, 0, 2 * BUFFER_SIZE); // fill buffer with zeros
        ws2812->zero_halves++;
      }

      ws2812->res_cnt++;

      if(ws2812->res_cnt >= LED_RESET_CYCLES) { // done enough reset  cycles move to next state
        ws2812->led_cnt = 0; // prepare to send data
        if (ws2812->is_dirty) {
          ws2812->is_dirty = false;
          ws2812->led_state = LED_DAT;
        } else {
          ws2812->led_state = LED_IDL;
        }
      } else { // LED_DAT
          ++ws2812->dat_cbs;

          // first deal with current led 
          uint8_t *led = (uint8_t*) &ws2812->led[3 * ws2812->led_cnt];

          for (uint8_t c = 0; c < 3; c++) { // Deal with the 3 color leds in one package
            // copy values from the pre-filled color_value buffer
            memcpy(dma_buffer_pointer,color_value[led[c]], 16);
            dma_buffer_pointer += 8; // next 8 bytes
        }

        // now move to next led switching to reset state when all leds have been updated
        ws2812->led_cnt++; // next led
        if (ws2812->led_cnt >= ws2812->leds) { // reached top
          ws2812->led_cnt = 0; // back to first
          ws2812->zero_halves = 0;
          ws2812->res_cnt = 0;
          ws2812->led_state = LED_RES;

        }
      }

      }

        ws2812_resultTypeDef zeroLedValues(ws2812_handleTypeDef *ws2812) {
            ws2812_resultTypeDef res = WS2812_Ok;
            memset(ws2812->led, 0, ws2812->leds * 3); // Zero it all
            ws2812->is_dirty = true; // Mark buffer dirty
            return res;
      }
    }

ws2812_resultTypeDef setLedValue(ws2812_handleTypeDef *ws2812, uint16_t led, uint8_t col, uint8_t value) {
    ws2812_resultTypeDef res = WS2812_Ok;
    if (led < ws2812->leds) {
    ws2812->led[3 * led + col] = value;
    ws2812->is_dirty = true; // Mark buffer dirty
    } else {
    res = WS2812_Err;
    }
    return res;
}
      // Just throw values into led_value array - the dma interrupt will
      // handle updating the dma buffer when needed
ws2812_resultTypeDef setLedValues(ws2812_handleTypeDef *ws2812, uint16_t led, uint8_t r, uint8_t g, uint8_t b) {
    ws2812_resultTypeDef res = WS2812_Ok;
    if (led < ws2812->leds) {
        ws2812->led[3 * led + RL] = r;
        ws2812->led[3 * led + GL] = g;
        ws2812->led[3 * led + BL] = b;
        ws2812->is_dirty = true; // Mark buffer dirty
    } else {
        res = WS2812_Err;
    }
    return res;
}

ws2812_resultTypeDef ws2812_init(ws2812_handleTypeDef *ws2812, TIM_TypeDef *timer, uint32_t channel, uint16_t leds) {

      ws2812_resultTypeDef res = WS2812_Ok;

      // Store timer handle for later
      ws2812->timer = timer;

      // Store channel
      ws2812->channel = channel;

      ws2812->leds = leds;

      ws2812->led_state = LED_RES;
      ws2812->is_dirty = 0;
      ws2812->zero_halves = 2;

      ws2812->led = malloc(leds * 3);
      if (ws2812->led != NULL) { // Memory for led values

          memset(ws2812->led, 0, leds * 3); // Zero it all

          // Start DMA to feed the PWM with values
          // At this point the buffer should be empty - all zeros
          HAL_TIM_PWM_Start_DMA(timer, channel, (uint32_t*) ws2812->dma_buffer, BUFFER_SIZE * 2);

      } else {
          res = WS2812_Mem;
      }

      return res;

  }










void WS2812_init(void){




}

void initLED(void){
  pinMode(LED_PIN, GPIO_OUTPUT); 
  digitalWrite(LED_PIN, PIO_LOW);
}

void send1Pulse(void){
  digitalWrite(LED_PIN, PIO_HIGH);
  delay_units(TIM16, T1H);
  digitalWrite(LED_PIN, PIO_LOW);
  delay_units(TIM16, T1L);
}

void send0Pulse(void){
  digitalWrite(LED_PIN, PIO_HIGH);
  delay_units(TIM16, T0H);
  digitalWrite(LED_PIN, PIO_LOW);
  delay_units(TIM16, T0L);
}

void sendVals(uint32_t bits){

 for (int i = 0; i < 24; i++) {
    if (bits & (1 << i))
      set_PWM_freq(TIM16, 800000, 80000000, T1H);
    else 
      set_PWM_freq(TIM16, 800000, 80000000, T0H);
    delay_units(TIM2, 25);
  }
}

