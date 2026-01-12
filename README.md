# Keil-Arm-ZebraCrossing
#include "stm32f4xx.h"

// ---------------- Configuration ----------------
#define NUM_LEDS        5

// WS2812 DMA settings
#define DMA_BUFFER_SIZE (50 + (NUM_LEDS*24) + 50) // Reset slots + LED data + reset
#define TIMER_ARR      19
#define WS2812_0_CODE  6
#define WS2812_1_CODE  13
#define WS2812_RESET   0

uint8_t led_data[NUM_LEDS*3];
uint32_t pwm_buffer[DMA_BUFFER_SIZE];
volatile uint8_t dma_done = 1;

#define BUZZER_PIN      2      //PD2
