#include "stm32f0xx.h"
#include "hal.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_class.h"
#include "usbd_usr.h"
#include "program.h"
#include <string.h>

#define INPUT_BUFFER_SIZE 256
#define OUTPUT_BUFFER_SIZE 256

// Whether we are streaming
volatile int hal_stream_enabled = 0;
volatile int hal_stream_overrun = 0;

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
    TIM_TimeBaseInitStruct.TIM_Period = 0;
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
    DMA_InitStructure.DMA_BufferSize = 0;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryBaseAddr = 0;
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
    DMA_InitStructure.DMA_BufferSize = 0;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryBaseAddr = 0;
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

    hal_stream_disable();
}

void hal_stream_enable() {
    if(program->input_buffer) {
        if(program->output_buffer) {
            DMA_ITConfig(DMA1_Channel3, DMA_IT_TC | DMA_IT_HT, ENABLE);
        } else {
            DMA_ITConfig(DMA1_Channel1, DMA_IT_TC | DMA_IT_HT, ENABLE);
        }

        DMA1_Channel1->CMAR = (uint32_t)program->input_buffer;
        DMA_Cmd(DMA1_Channel1, DISABLE);
        DMA_SetCurrDataCounter(DMA1_Channel1, program->buffer_size);
        DMA_Cmd(DMA1_Channel1, ENABLE);
    }

    if(program->output_buffer) {
        DMA1_Channel3->CMAR = (uint32_t)program->output_buffer;
        DMA_Cmd(DMA1_Channel3, DISABLE);
        DMA_SetCurrDataCounter(DMA1_Channel1, program->buffer_size);
        DMA_Cmd(DMA1_Channel3, ENABLE);
    }

    hal_stream_enabled = 1;
    hal_stream_overrun = 0;

    TIM16->ARR = (SystemCoreClock / program->sample_rate) - 1;
    TIM17->ARR = TIM16->ARR;
    TIM16->CNT = 0;
    TIM17->CNT = 0;
    TIM_Cmd(TIM16, ENABLE);
    TIM_Cmd(TIM17, ENABLE);
    TIM17->CNT = TIM16->CNT;
    TIM16->CNT = TIM16->CNT;
}

void hal_stream_disable() {
    TIM_Cmd(TIM16, DISABLE);
    TIM_Cmd(TIM17, DISABLE);

    DMA_Cmd(DMA1_Channel1, DISABLE);
    DMA_Cmd(DMA1_Channel3, DISABLE);
    DMA_ITConfig(DMA1_Channel3, DMA_IT_TC | DMA_IT_HT, DISABLE);
    DMA_ITConfig(DMA1_Channel3, DMA_IT_TC | DMA_IT_HT, DISABLE);
    hal_stream_enabled = 0;
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

static void dma_interrupt(uint8_t which) {
    uint8_t *in = program->input_buffer;
    uint8_t *out = program->output_buffer;
    uint16_t len = program->buffer_size / 2;
    if(which) {
        if(in) in += len;
        if(out) out += len;
    }
    program->process(in, out, len);
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
    dma_interrupt((DMA1->ISR & DMA1_IT_TC1) ? 0 : 1);
    if(DMA1->ISR & (DMA1_IT_HT1 | DMA1_IT_TC1)) {
        hal_stream_overrun = 1;
        hal_stream_disable();
    }
}

void DMA1_Channel2_3_IRQHandler() {
    DMA_ClearITPendingBit(DMA1_IT_TC3 | DMA1_IT_HT3);
    dma_interrupt((DMA1->ISR & DMA1_IT_TC3) ? 0 : 1);
    if(DMA1->ISR & (DMA1_IT_HT3 | DMA1_IT_TC3)) {
        hal_stream_overrun = 1;
        hal_stream_disable();
    }
}
