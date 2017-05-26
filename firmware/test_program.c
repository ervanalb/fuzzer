#include "test_program.h"
#include "program.h"

#define BUF_SIZE 256

static uint8_t input_buf[BUF_SIZE];
static uint8_t output_buf[BUF_SIZE];
static uint16_t buffer_size = BUF_SIZE;

static void init() {
}

static void process(uint8_t *input_buffer, uint8_t *output_buffer, uint16_t size) {
}

void test_program_load() {
    program->input_buffer = input_buf;
    program->output_buffer = output_buf;
    program->buffer_size = buffer_size;
    program->init = init;
    program->process = process;
}
