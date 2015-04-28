static struct joint joints[] = {
    { /* joint 1, joints[0] */
        .motor = {
            .pwm = {.timer_peripheral = TIM4,
                    .oc_id = TIM_OC1,
                    .pin = {.port = GPIOD, .pin = GPIO12, .af = GPIO_AF2}
            },
            .dira = {.port = GPIOE, .pin = GPIO7},
            .dirb = {.port = GPIOE, .pin = GPIO9}
        },
        .pot = {
            .adc = ADC1,
            .channel = 0,
            .pin = { .port = GPIOA, .pin = GPIO0 }
        },
        .pid_state = {
            .prev_error = 0.0f,
            .integral = 0.0f
        },
        .pid_params = {
            .p = 10.0f,
            .i = 20.0f,
            .d = 0.0f,
            .i_max = 0.01f
        },
        .setpoint = 0.49f,
        .adc_angle = NAN
    },
    { /* joint 2, joints[1] */
        .motor = {
            .pwm = {.timer_peripheral = TIM4,
                    .oc_id = TIM_OC2,
                    .pin = {.port = GPIOD, .pin = GPIO13, .af = GPIO_AF2}
            },
            .dira = {.port = GPIOE, .pin = GPIO11},
            .dirb = {.port = GPIOE, .pin = GPIO13}
        },
        .pot = {
            .adc = ADC1,
            .channel = 2,
            .pin = { .port = GPIOA, .pin = GPIO2 }
        },
        .pid_state = {
            .prev_error = 0.0f,
            .integral = 0.0f
        },
        .pid_params = {
            .p = 10.0f,
            .i = 20.0f,
            .d = 0.0f,
            .i_max = 0.01f
        },
        .setpoint = 0.49f,
        .adc_angle = NAN
    },
    { /* joint 3, joints[2] */
        .motor = {
            .pwm = {.timer_peripheral = TIM4,
                    .oc_id = TIM_OC4,
                    .pin = {.port = GPIOD, .pin = GPIO15, .af = GPIO_AF2}
            },
            .dira = {.port = GPIOE, .pin = GPIO15},
            .dirb = {.port = GPIOD, .pin = GPIO10}
        },
        .pot = {
            .adc = ADC1,
            .channel = 4,
            .pin = { .port = GPIOA, .pin = GPIO4 }
        },
        .pid_state = {
            .prev_error = 0.0f,
            .integral = 0.0f
        },
        .pid_params = {
            .p = 10.0f,
            .i = 20.0f,
            .d = 0.0f,
            .i_max = 0.01f
        },
        .setpoint = 0.49f,
        .adc_angle = NAN
    },
    { /* joint 4, joints[3] */
        .motor = {
            .pwm = {.timer_peripheral = TIM4,
                    .oc_id = TIM_OC3,
                    .pin = {.port = GPIOD, .pin = GPIO14, .af = GPIO_AF2}
            },
            .dira = {.port = GPIOE, .pin = GPIO10},
            .dirb = {.port = GPIOE, .pin = GPIO12}
        },
        .pot = {
            .adc = ADC1,
            .channel = 6,
            .pin = { .port = GPIOA, .pin = GPIO6 }
        },
        .pid_state = {
            .prev_error = 0.0f,
            .integral = 0.0f
        },
        .pid_params = {
            .p = 10.0f,
            .i = 20.0f,
            .d = 0.0f,
            .i_max = 0.01f
        },
        .setpoint = 0.49f,
        .adc_angle = NAN
    },
    { /* joint 5, joints[4] */
        .motor = {
            .pwm = {.timer_peripheral = TIM3,
                    .oc_id = TIM_OC3,
                    .pin = {.port = GPIOC, .pin = GPIO8, .af = GPIO_AF2}
            },
            .dira = {.port = GPIOE, .pin = GPIO14},
            .dirb = {.port = GPIOE, .pin = GPIO4}
        },
        .pot = {
            .adc = ADC1,
            .channel = 14,
            .pin = { .port = GPIOC, .pin = GPIO4 }
        },
        .pid_state = {
            .prev_error = 0.0f,
            .integral = 0.0f
        },
        .pid_params = {
            .p = 10.0f,
            .i = 20.0f,
            .d = 0.0f,
            .i_max = 0.01f
        },
        .setpoint = 0.49f,
        .adc_angle = NAN
    },
    { /* joint 6, joints[5] */
        .motor = {
            .pwm = {.timer_peripheral = TIM4,
                    .oc_id = TIM_OC1,
                    .pin = {.port = GPIOD, .pin = GPIO12, .af = GPIO_AF2}
            },
            .dira = {.port = GPIOE, .pin = GPIO7},
            .dirb = {.port = GPIOE, .pin = GPIO9}
        },
        .pot = {
            .adc = ADC1,
            .channel = 0,
            .pin = { .port = GPIOA, .pin = GPIO0 }
        },
        .pid_state = {
            .prev_error = 0.0f,
            .integral = 0.0f
        },
        .pid_params = {
            .p = 10.0f,
            .i = 20.0f,
            .d = 0.0f,
            .i_max = 0.01f
        },
        .setpoint = 0.49f,
        .adc_angle = NAN
    }
};


