#ifndef CONFIG_H
#define CONFIG_H

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>

#include <math.h>

#include "pid.h"

#define FILTER_BUF_SIZE 7
#define AVG_BUF_SIZE 7

#define NUM_JOINTS 6

struct pin {
    uint32_t port;
    uint16_t pin;
    uint16_t af;
};

struct pwm_output {
    struct pin pin;
    uint32_t timer_peripheral; /* e.g. TIM1 */
    enum tim_oc_id oc_id; /* e.g. TIM_OC1 */
};

struct motor {
    struct pwm_output pwm;
    struct pin dir;
};

struct adc_pin {
    uint32_t adc;
    uint8_t channel;
    struct pin pin;
};

static const struct pin brake_relay_pin = {
    .port = GPIOE, .pin = GPIO4
};

static const struct pin gripper_relay_pin = {
    .port = GPIOE, .pin = GPIO6
};

static const struct pin motor_enable_pin = {
    .port = GPIOE, .pin = GPIO5
};

static const struct joint_hw {
    const struct motor motor;
    const struct adc_pin pot;
    const struct adc_pin cur;
} joint_hws[] = {
    {
        .motor = {
            .pwm = { .timer_peripheral = TIM1,
                     .oc_id = TIM_OC1,
                     .pin = { .port = GPIOA, .pin = GPIO8, .af = GPIO_AF1 }
            },
            .dir = { .port = GPIOD, .pin = GPIO2 },
        },
        .pot = { /* 1 */
            .adc = ADC1,
            .channel = 11,
            .pin = { .port = GPIOC, .pin = GPIO1 }
        },
        .cur = {
            .adc = ADC1,
            .channel = 12,
            .pin = { .port = GPIOC, .pin = GPIO2 }
        },
    }, {
        .motor = {
            .pwm = { .timer_peripheral = TIM3,
                     .oc_id = TIM_OC2,
                     .pin = { .port = GPIOB, .pin = GPIO5, .af = GPIO_AF2 }
            },
            .dir = { .port = GPIOD, .pin = GPIO6 },
        },
        .pot = { /* 2 */
            .adc = ADC1,
            .channel = 1,
            .pin = { .port = GPIOA, .pin = GPIO1 }
        },
        .cur = {
            .adc = ADC1,
            .channel = 0,
            .pin = { .port = GPIOA, .pin = GPIO0 }
        },
    }, {
        .motor = {
            .pwm = { .timer_peripheral = TIM4,
                     .oc_id = TIM_OC2,
                     .pin = { .port = GPIOB, .pin = GPIO7, .af = GPIO_AF2 }
            },
            .dir = { .port = GPIOE, .pin = GPIO0 },
        },
        .pot = { /* 3 */
            .adc = ADC1,
            .channel = 3,
            .pin = { .port = GPIOA, .pin = GPIO3 }
        },
        .cur = {
            .adc = ADC1,
            .channel = 2,
            .pin = { .port = GPIOA, .pin = GPIO2 }
        },
    }, {
        .motor = {
            .pwm = { .timer_peripheral = TIM3,
                     .oc_id = TIM_OC1,
                     .pin = { .port = GPIOC, .pin = GPIO6, .af = GPIO_AF2 }
            },
            .dir = { .port = GPIOD, .pin = GPIO1 },
        },
        .pot = { /* 4 */
            .adc = ADC1,
            .channel = 5,
            .pin = { .port = GPIOA, .pin = GPIO5 }
        },
        .cur = {
            .adc = ADC1,
            .channel = 8,
            .pin = { .port = GPIOB, .pin = GPIO0 }
        },
    }, {
        .motor = {
            .pwm = { .timer_peripheral = TIM3,
                     .oc_id = TIM_OC3,
                     .pin = { .port = GPIOC, .pin = GPIO8, .af = GPIO_AF2 }
            },
            .dir = { .port = GPIOD, .pin = GPIO3 },
        },
        .pot = { /* 5 */
            .adc = ADC1,
            .channel = 15,
            .pin = { .port = GPIOC, .pin = GPIO5 }
        },
        .cur = {
            .adc = ADC1,
            .channel = 14,
            .pin = { .port = GPIOC, .pin = GPIO4 }
        },
    }, {
        .motor = {
            .pwm = { .timer_peripheral = TIM3,
                     .oc_id = TIM_OC4,
                     .pin = { .port = GPIOC, .pin = GPIO9, .af = GPIO_AF2 }
            },
            .dir = { .port = GPIOD, .pin = GPIO0 },
        },
        .pot = { /* 6 */
            .adc = ADC1,
            .channel = 9,
            .pin = { .port = GPIOB, .pin = GPIO1 }
        },
        .cur = {
            .adc = ADC1,
            .channel = 6,
            .pin = { .port = GPIOA, .pin = GPIO6 }
        },
    },
};

#endif
