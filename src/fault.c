#include <stdio.h>

#include "fault.h"
#include "config.h"

static void drive_halt(void)
{
    gpio_clear(motor_enable_pin.port, motor_enable_pin.pin);
    gpio_clear(brake_relay_pin.port, brake_relay_pin.pin);
}

void hard_fault_handler(void)
{
    printf("hard fault, spinning\n");
    drive_halt();
    for (;;)
        ;
}
