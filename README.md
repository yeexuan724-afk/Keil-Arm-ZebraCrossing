# Keil-Arm-ZebraCrossing
#include "stm32f4xx.h"

// ---------------- Configuration ----------------
#define NUM_LEDS        5
#define BUZZER_PIN      2      //PD2
#define BUTTON_PIN      2   // PA2

// WS2812 DMA settings
#define DMA_BUFFER_SIZE (50 + (NUM_LEDS*24) + 50) // Reset slots + LED data + reset
#define TIMER_ARR      19
#define WS2812_0_CODE  6
#define WS2812_1_CODE  13
#define WS2812_RESET   0

uint8_t led_data[NUM_LEDS*3];
uint32_t pwm_buffer[DMA_BUFFER_SIZE];
volatile uint8_t dma_done = 1;

// 7-segment mapping (common cathode)
const uint8_t seg_map[] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};

// ---------------- Function Prototypes ----------------
void SystemClock_Config(void);
void GPIO_Init(void);
void TIM2_Init(void);
void DMA_Init(void);
void WS2812_Init(void);
void WS2812_SetPixel(uint16_t pixel, uint8_t r, uint8_t g, uint8_t b);
void WS2812_Show(void);
void Display_Double_Digit(int value);
void Fast_Delay(uint32_t ms);
void Delay_ms(uint32_t ms);
void Buzzer_Continuous(uint32_t duration_ms, uint32_t pulse_ms);


// ---------------- Main ----------------
int main(void){
    SystemClock_Config();
    GPIO_Init();
    TIM2_Init();
    DMA_Init();
    WS2812_Init();

// ---------------- BUZZER GPIO ----------------
    GPIOD->MODER |= (1U << (BUZZER_PIN*2));  // BUZZER -> PD2 output

// Configure PA2 as input with pull-down for button
    GPIOA->MODER &= ~(3U << (BUTTON_PIN*2));
    GPIOA->PUPDR |= (2U << (BUTTON_PIN*2));

   while(1){
        // ---------------- IDLE ----------------
        for(int i=0;i<NUM_LEDS;i++) WS2812_SetPixel(i, 150,0,0); // Red
        WS2812_Show();
        Display_Double_Digit(0);

// Wait for button press
        if(GPIOA->IDR & (1U << BUTTON_PIN)){
            // ---------- Phase 1: 5s countdown, red ----------
            for(int i=5;i>=0;i--){
                Display_Double_Digit(i);
                Delay_ms(600); // Slightly faster countdown (~0.6s)
            }


                    // ---------- Phase 2: 10s countdown, green ----------
            for(int i=0;i<NUM_LEDS;i++) WS2812_SetPixel(i, 0,150,0); // Green
            WS2812_Show();
            
            for(int i=10;i>=0;i--){
                Display_Double_Digit(i);

             }

                    // ---------- Phase 3: back to Red ----------
            for(int i=0;i<NUM_LEDS;i++) WS2812_SetPixel(i, 150,0,0); // Red
            WS2812_Show();
        }
    }
}
