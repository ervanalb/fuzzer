#ifndef __HAL_H
#define __HAL_H

#include <stdint.h>

void hal_init();

void hal_stream_input_enable();
void hal_stream_input_disable();
void hal_stream_output_enable();
void hal_stream_output_disable();

// Calculate the number of bytes behind the DMA counter the read pointer currently is.
// Assume no overflows.
int hal_stream_input_available();

// Read n samples from rx buffer into samples
void hal_stream_input(uint8_t* samples, int n);

// Returns the maximum number of samples that can be written to the tx buffer.
// Assumes no overflows.
int hal_stream_output_space();

// Write n bytes into the tx buffer
void hal_stream_output(uint8_t *samples, int n);

extern volatile int hal_stream_input_enabled;
extern volatile int hal_stream_output_enabled;

#endif
