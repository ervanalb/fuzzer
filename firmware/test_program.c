#include "test_program.h"
#include "program.h"

#define BUF_SIZE 1024
#define SAMPLE_RATE 10000000

//static uint8_t input_buf[BUF_SIZE];
static uint8_t output_buf[BUF_SIZE];
static uint16_t buffer_size = BUF_SIZE;
static uint32_t rate = SAMPLE_RATE;

static void (*configure_pin)(uint8_t pin, uint8_t config);

static void init() {
    configure_pin = program->api->configure_pin;
    configure_pin(0, PROGRAM_CONF_OUTPUT);
    configure_pin(1, 0);
    configure_pin(2, 0);
    configure_pin(3, 0);
    configure_pin(4, 0);
    configure_pin(5, 0);
    configure_pin(6, 0);
    configure_pin(7, 0);
}

static void process(uint8_t *input_buffer, uint8_t *output_buffer, uint16_t size) {
    for(int i=0; i<BUF_SIZE; i+=2) {
        output_buf[i] = 0;
        output_buf[i+1] = 1;
    }
}

void test_program_load() {
    program->input_buffer = 0;
    program->output_buffer = output_buf;
    program->buffer_size = buffer_size;
    program->sample_rate = rate;
    program->init = init;
    program->process = process;
    program_loaded();
}
