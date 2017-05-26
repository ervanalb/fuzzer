#ifndef __PROGRAM_H
#define __PROGRAM_H

#include <stdint.h>

extern uint8_t _suser_program;
extern uint8_t _euser_program;
extern struct program *program;

struct program {
    void *input_buffer;
    void *output_buffer;
    uint16_t buffer_size;

    void (*init)();
    void (*process)(uint8_t *input_buffer, uint8_t *output_buffer, uint16_t buffer_size);
};

#endif
