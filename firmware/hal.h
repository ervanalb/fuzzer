#ifndef __HAL_H
#define __HAL_H

#include <stdint.h>

#define HAL_CONF_OUTPUT 1
#define HAL_CONF_PU 2
#define HAL_CONF_PD 4
#define HAL_CONF_OD 8

void hal_init();

void hal_configure_pin(uint8_t pin, uint8_t config);

void hal_stream_enable(uint8_t *input_buffer, uint8_t *output_buffer, uint16_t buffer_size);
void hal_stream_disable();

// Calculate the number of bytes behind the DMA counter the read pointer currently is.
// Assume no overflows.
int hal_stream_input_available();

extern volatile int hal_stream_enabled;
extern volatile int hal_stream_overrun;

#endif
