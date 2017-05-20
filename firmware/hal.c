#include "stm32f0xx.h"
#include "hal.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_class.h"
#include "usbd_usr.h"
#include <string.h>

#define INPUT_BUFFER_SIZE 256
#define OUTPUT_BUFFER_SIZE 256
#define PERIOD 96

// Buffer where samples input from the pins are put
static volatile uint8_t input_buffer[INPUT_BUFFER_SIZE] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

// Buffer where samples written to the pins are put
static volatile uint8_t output_buffer[OUTPUT_BUFFER_SIZE];

// Pointer to buffer where data is being read (lags DMA counter)
static volatile int input_read_ptr = 0;

// Pointer to buffer where data is being written (leads DMA counter)
static volatile int16_t output_write_ptr = 0;

// Whether we are streaming
volatile int hal_stream_input_enabled = 0;
volatile int hal_stream_output_enabled = 0;

USB_CORE_HANDLE  USB_Device_dev;

// Bring up all hardware
void hal_init() {
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    NVIC_InitTypeDef NVIC_InitStructure; 
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM17, ENABLE);

    // Pins
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Pin = 0x00FF;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Timers
    TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_Period = PERIOD - 1;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM16, &TIM_TimeBaseInitStruct);
    TIM_TimeBaseInit(TIM17, &TIM_TimeBaseInitStruct);
    TIM_DMACmd(TIM16, TIM_DMA_Update, ENABLE);
    TIM_DMACmd(TIM17, TIM_DMA_Update, ENABLE);
    TIM_Cmd(TIM16, ENABLE);
    TIM_Cmd(TIM17, ENABLE);
    TIM17->CNT = TIM16->CNT;
    TIM16->CNT = TIM16->CNT;

    // Input DMA
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_BufferSize = INPUT_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)input_buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(GPIOB->IDR);
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC | DMA_IT_HT, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Output DMA
    DMA_DeInit(DMA1_Channel3);
    DMA_InitStructure.DMA_BufferSize = OUTPUT_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)output_buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(GPIOB->ODR);
    DMA_Init(DMA1_Channel3, &DMA_InitStructure);
    DMA_ITConfig(DMA1_Channel3, DMA_IT_TC | DMA_IT_HT, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  
    // USB
    USBD_Init(&USB_Device_dev, &USR_desc,
              &USBD_custom_cb, &USR_cb);

    hal_stream_input_disable();
    hal_stream_output_disable();
}

// These commands control whether data flows from the pins into memory
void hal_stream_input_enable() {
    DMA_Cmd(DMA1_Channel1, DISABLE);
    DMA_SetCurrDataCounter(DMA1_Channel1, INPUT_BUFFER_SIZE);
    DMA_Cmd(DMA1_Channel1, ENABLE);
    hal_stream_input_enabled = 1;
}

void hal_stream_input_disable() {
    DMA_Cmd(DMA1_Channel1, DISABLE);
    hal_stream_input_enabled = 0;
}

// These commands control whether data flows from memory into pins
void hal_stream_output_enable() {
    DMA_Cmd(DMA1_Channel3, DISABLE);
    DMA_SetCurrDataCounter(DMA1_Channel1, OUTPUT_BUFFER_SIZE);
    DMA_Cmd(DMA1_Channel3, ENABLE);
    hal_stream_output_enabled = 1;
}

void hal_stream_output_disable() {
    DMA_Cmd(DMA1_Channel3, DISABLE);
    hal_stream_output_enabled = 0;
}

// Calculate the number of bytes behind the DMA counter the read pointer currently is.
// Assume no overflows.
int hal_stream_input_available() {
    int remaining = INPUT_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Channel1) - input_read_ptr;
    if(remaining < 0) remaining += INPUT_BUFFER_SIZE;
    return remaining;
}

// Read n samples from rx buffer into samples
void hal_stream_input(uint8_t* samples, int n) {
    for(int i = 0; i < n; i++) {
        samples[i] = input_buffer[input_read_ptr];
        input_read_ptr = (input_read_ptr + 1) % INPUT_BUFFER_SIZE;
    }
}

// Returns the maximum number of samples that can be written to the tx buffer.
// Assumes no overflows.
int hal_stream_output_space() {
    int available = DMA_GetCurrDataCounter(DMA1_Channel3) - output_write_ptr;
    if(available < 0) available += OUTPUT_BUFFER_SIZE;
    return available;
}

// Write n bytes into the tx buffer
void hal_stream_output(uint8_t *samples, int n) {
    for(int i=0; i<n; i++) {
        output_buffer[output_write_ptr] = samples[i];
        output_write_ptr = (output_write_ptr + 1) % OUTPUT_BUFFER_SIZE;
    }
}

// Reconfigure pin
void hal_configure_pin(uint8_t pin, uint8_t config) {
    if(config & HAL_CONF_OUTPUT) {
        GPIOB->MODER |= 1 << (2 * pin);
    } else {
        GPIOB->MODER &= ~(1 << (2 * pin));
    }
    if(config & HAL_CONF_PU) {
        uint32_t pupdr = GPIOB->PUPDR;
        pupdr |= 1 << (2 * pin);
        pupdr &= ~(2 << (2 * pin));
        GPIOB->PUPDR = pupdr;
    } else if(config & HAL_CONF_PD) {
        uint32_t pupdr = GPIOB->PUPDR;
        pupdr |= ~(2 << (2 * pin));
        pupdr &= ~(1 << (2 * pin));
        GPIOB->PUPDR = pupdr;
    } else {
        GPIOB->PUPDR &= ~(3 << (2 * pin));
    }
    if(config & HAL_CONF_OD) {
        GPIOB->OTYPER |= 1 << pin;
    } else {
        GPIOB->OTYPER &= ~(1 << pin);
    }
}

// INTERRUPTS

void NMI_Handler() {
}

void HardFault_Handler() {
    for(;;);
}

void SVC_Handler() {
}

void PendSV_Handler() {
}

void SysTick_Handler() {
}

void USB_IRQHandler() {
    USB_Istr();
}

void DMA1_Channel1_IRQHandler() {
    DMA_ClearITPendingBit(DMA1_IT_TC1 | DMA1_IT_HT1);
}

void DMA1_Channel2_3_IRQHandler() {
    DMA_ClearITPendingBit(DMA1_IT_TC3 | DMA1_IT_HT3);
}
