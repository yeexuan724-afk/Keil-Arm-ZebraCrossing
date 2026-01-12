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

// Configure PD2 as output for buzzer
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
                
                // Determine buzzer speed
                uint32_t pulse_time = (i<=4)? 40 : 80; // faster pulses

                // Pulse buzzer continuously during this countdown tick (~0.8s per tick)
                Buzzer_Continuous(800, pulse_time);
             }
            
            // Turn off buzzer
            GPIOD->ODR &= ~(1U << BUZZER_PIN);

            // ---------- Phase 3: back to Red ----------
            for(int i=0;i<NUM_LEDS;i++) WS2812_SetPixel(i, 150,0,0); // Red
            WS2812_Show();
        }
    }
}

// ---------------- Buzzer Continuous Pulse ----------------
void Buzzer_Continuous(uint32_t duration_ms, uint32_t pulse_ms){
    uint32_t elapsed = 0;
    while(elapsed < duration_ms){
        GPIOD->ODR |= (1U << BUZZER_PIN);
        Fast_Delay(pulse_ms);
        GPIOD->ODR &= ~(1U << BUZZER_PIN);
        Fast_Delay(pulse_ms);
        elapsed += 2*pulse_ms;
    }
}

// ---------------- WS2812 Functions ----------------
void WS2812_Init(void){
    for(int i=0;i<DMA_BUFFER_SIZE;i++) pwm_buffer[i] = WS2812_RESET;
    for(int i=0;i<NUM_LEDS*3;i++) led_data[i] = 0;
    dma_done = 1;
}

void WS2812_SetPixel(uint16_t pixel, uint8_t r, uint8_t g, uint8_t b){
    if(pixel >= NUM_LEDS) return;
    led_data[pixel*3+0] = g;
    led_data[pixel*3+1] = r;
    led_data[pixel*3+2] = b;
}

void WS2812_Show(void){
    uint32_t idx = 50; // Reset slots
    while(!dma_done);

    for(int led=0;led<NUM_LEDS;led++){
        for(int c=0;c<3;c++){
            uint8_t byte_val = led_data[led*3+c];
            for(int bit=7;bit>=0;bit--){
                pwm_buffer[idx++] = (byte_val & (1<<bit))? WS2812_1_CODE:WS2812_0_CODE;
            }
        }
    }

    dma_done = 0;

    // Restart DMA
    DMA1_Stream1->CR &= ~DMA_SxCR_EN;
    while(DMA1_Stream1->CR & DMA_SxCR_EN);
    DMA1->LIFCR = 0x3F<<6;
    DMA1_Stream1->NDTR = DMA_BUFFER_SIZE;
    DMA1_Stream1->M0AR = (uint32_t)pwm_buffer;
    DMA1_Stream1->PAR  = (uint32_t)&TIM2->CCR3;

    TIM2->CNT = 0;
    DMA1_Stream1->CR |= DMA_SxCR_EN;
    TIM2->DIER |= TIM_DIER_CC3DE;
    TIM2->CR1 |= TIM_CR1_CEN;
}

void DMA1_Stream1_IRQHandler(void){
    if(DMA1->LISR & (1U<<11)){
        DMA1->LIFCR = (1U<<11);
        dma_done = 1;
        TIM2->CR1 &= ~TIM_CR1_CEN;
        TIM2->DIER &= ~TIM_DIER_CC3DE;
    }
}

void Fast_Delay(uint32_t ms){
    for(uint32_t i=0;i<ms*1500;i++) __NOP();
}

void Delay_ms(uint32_t ms){
    for(uint32_t i=0;i<ms*1200;i++) __NOP();
}

// ---------------- Hardware Init ----------------
void GPIO_Init(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOAEN;

    // RGB (PB10, TIM2 AF)
    GPIOB->MODER &= ~(3U<<20);    
    GPIOB->MODER |= (2U<<20);     
    GPIOB->OSPEEDR |= (3U<<20);   
    GPIOB->AFR[1] &= ~(0xF<<8);
    GPIOB->AFR[1] |= (1<<8);      
    
}

void TIM2_Init(void){
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 0;
    TIM2->ARR = TIMER_ARR;
    TIM2->CCMR2 |= (6U<<4)|TIM_CCMR2_OC3PE;
    TIM2->CCER |= TIM_CCER_CC3E;
    TIM2->EGR |= TIM_EGR_UG;
}

void DMA_Init(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    DMA1_Stream1->CR = 0;
    while(DMA1_Stream1->CR & DMA_SxCR_EN);

    DMA1_Stream1->CR = (3U<<25)|(1U<<10)|(2U<<11)|(2U<<13)|(1U<<6)|(1U<<4);

    NVIC_SetPriority(DMA1_Stream1_IRQn,0);
    NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

void SystemClock_Config(void){
    RCC->CR |= RCC_CR_HSION;
    while(!(RCC->CR & RCC_CR_HSIRDY));
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_HSI;
    while(((RCC->CFGR & RCC_CFGR_SWS)>>2)!=0);
}
