static struct joint joints[] = {
    { /* joint 1, joints[0] */
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
        .pid_state = {
            .prev_error = 0.0f,
            .integral = 0.0f
        },
        .pid_params = {
            .p = 2.0f,
            .i = 0.0f,
            .d = 0.0f,
            .i_max = 0.01f
        },
        .setpoint = 0.49f,
        .adc_angle = NAN
    },
    { /* joint 2, joints[1] */
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
        .pid_state = {
            .prev_error = 0.0f,
            .integral = 0.0f
        },
        .pid_params = {
            .p = 20.0f,
            .i = 5.0f,
            .d = 0.0f,
            .i_max = 0.01f
        },
        .setpoint = 0.49f,
        .adc_angle = NAN
    },
    { /* joint 3, joints[2] */
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
        .pid_state = {
            .prev_error = 0.0f,
            .integral = 0.0f
        },
        .pid_params = {
            .p = 20.0f,
            .i = 10.0f,
            .d = 0.0f,
            .i_max = 0.01f
        },
        .setpoint = 0.49f,
        .adc_angle = NAN
    },
    { /* joint 4, joints[3] */
        .motor = {
            .pwm = { .timer_peripheral = TIM3,
                     .oc_id = TIM_OC1,
                     .pin = { .port = GPIOC, .pin = GPIO6, .af = GPIO_AF2 }
            },
            .dir = { .port = GPIOA, .pin = GPIO13 },
        },
        .pot = { /* 4 */
            .adc = ADC1,
            .channel = 5,
            .pin = { .port = GPIOA, .pin = GPIO5 }
        },
        .pid_state = {
            .prev_error = 0.0f,
            .integral = 0.0f
        },
        .pid_params = {
            .p = 15.0f,
            .i = 10.0f,
            .d = 0.0f,
            .i_max = 0.01f
        },
        .setpoint = 0.49f,
        .adc_angle = NAN
    },
    { /* joint 5, joints[4] */
        .motor = {
            .pwm = { .timer_peripheral = TIM3,
                     .oc_id = TIM_OC3,
                     .pin = { .port = GPIOC, .pin = GPIO8, .af = GPIO_AF2 }
            },
            .dir = { .port = GPIOA, .pin = GPIO15 },
        },
        .pot = { /* 5 */
            .adc = ADC1,
            .channel = 15,
            .pin = { .port = GPIOC, .pin = GPIO5 }
        },
        .pid_state = {
            .prev_error = 0.0f,
            .integral = 0.0f
        },
        .pid_params = {
            .p = 15.0f,
            .i = 10.0f,
            .d = 0.0f,
            .i_max = 0.01f
        },
        .setpoint = 0.49f,
        .adc_angle = NAN
    },
    { /* joint 6, joints[5] */
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
        .pid_state = {
            .prev_error = 0.0f,
            .integral = 0.0f
        },
        .pid_params = {
            .p = 3.0f,
            .i = 0.0f,
            .d = 0.0f,
            .i_max = 0.01f
        },
        .setpoint = 0.49f,
        .adc_angle = NAN
    }
};
