#include "program.h"
#include "hal.h"

volatile struct program *program = (struct program *)&_suser_program;

const struct program_api program_api = {
    .configure_pin = hal_configure_pin
};

void program_loaded() {
    program->api = &program_api;
}
