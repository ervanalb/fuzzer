#include "main.h"
#include "hal.h"
#include "test_program.h"

int main() {
    hal_init();
    test_program_load();
    for(;;);
}
