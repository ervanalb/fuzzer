#ifndef __PROGRAM_H
#define __PROGRAM_H

#include <stdint.h>

#define PROGRAM_CONF_OUTPUT 1
#define PROGRAM_CONF_PU 2
#define PROGRAM_CONF_PD 4
#define PROGRAM_CONF_OD 8

extern uint8_t _suser_program;
extern uint8_t _euser_program;

struct program {
    const struct program_api *api;
    void *input_buffer;
    void *output_buffer;
    uint16_t buffer_size;
    uint32_t sample_rate;

    void (*init)();
    void (*process)(uint8_t *input_buffer, uint8_t *output_buffer, uint16_t buffer_size);
};

struct program_api {
    void (*configure_pin)(uint8_t pin, uint8_t config);
};

extern volatile struct program *program;
extern const struct program_api program_api;

void program_loaded();

#endif
