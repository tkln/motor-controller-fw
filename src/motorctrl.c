#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <stdlib.h>

static uint32_t timer_get_period(uint32_t timer_peripheral)
{
    return TIM_ARR(timer_peripheral);
}

struct pwm_output {
    uint32_t timer_peripheral; /* e.g. TIM1 */
    enum tim_oc_id oc_id; /* e.g. TIM_OC1 */
};

static void pwm_output_init(struct pwm_output output)
{
    timer_set_oc_mode(output.timer_peripheral, output.oc_id, TIM_OCM_PWM1);
    timer_enable_oc_output(output.timer_peripheral, output.oc_id);
}

struct motor {
    struct pwm_output a;
    struct pwm_output b;
};

static void motor_init(struct motor motor)
{
    pwm_output_init(motor.a);
    pwm_output_init(motor.b);
}

static void pwm_output_set(struct pwm_output output, float val)
{
    uint32_t period = timer_get_period(output.timer_peripheral);
    timer_set_oc_value(output.timer_peripheral, output.oc_id, period * val);
}

static const struct motor motors[] = {
    {{TIM1, TIM_OC1}, {TIM1, TIM_OC2}}
};

static void clock_setup(void)
{
    rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_120MHZ]);
}

static void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOD);
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
}

static void timer_setup(void)
{
    size_t i;
    rcc_periph_clock_enable(RCC_GPIOE); /* PWM */
    rcc_periph_clock_enable(RCC_TIM1); /* PWM */
    gpio_mode_setup(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO11);
    gpio_set_af(GPIOE, GPIO_AF1, GPIO9 | GPIO11);
    rcc_periph_clock_enable(RCC_TIM1);
    timer_reset(TIM1);
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE,
                   TIM_CR1_DIR_UP);
    for (i = 0; i < sizeof(motors)/sizeof(motors[0]); ++i)
        motor_init(motors[i]);
    timer_enable_break_main_output(TIM1);
    timer_set_period(TIM1, 6000);
    timer_enable_counter(TIM1);
}

static void adc_setup(void)
{
    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
    adc_off(ADC1);
    adc_disable_scan_mode(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_3CYC);
    adc_power_on(ADC1);
}

static float read_adc_simple(uint8_t channel)
{
    uint8_t channel_array[16];
    channel_array[0] = channel;
    adc_set_regular_sequence(ADC1, 1, channel_array);
    adc_start_conversion_regular(ADC1);
    while (!adc_eoc(ADC1))
        ;
    uint16_t reg16 = adc_read_regular(ADC1);
    return (float)reg16 / (1<<12);
}

static void set_motor(struct motor motor, float val)
{
    pwm_output_set(motor.a, val);
    pwm_output_set(motor.b, 1.0f - val);
}

struct pid_state {
    float prev_error;
    float integral;
};

struct pid_params {
    float p;
    float i;
    float d;
};

static float pid(struct pid_state *state, struct pid_params k, float measured,
                 float setpoint, float delta_t)
{
    float error = setpoint - measured;
    state->integral += error * delta_t; /* TODO gapping the integral */
    float derivative = (state->prev_error - error) / delta_t;
    state->prev_error = error;
    return k.p * error + k.i * state->integral + k.d * derivative;
}

int main(void)
{
    int i;

    clock_setup();
    gpio_setup();
    timer_setup();
    adc_setup();

    float setpoint = 0.5f;

    struct pid_state state = {.prev_error = 0.0f, .integral = 0.0f};
    struct pid_params params = {.p = 10.0f, .i = 10.0f, .d = 0.0f};

    while (1) {
        set_motor(motors[0], pid(&state, params, read_adc_simple(0), setpoint,
                  10000.0f / 120000000.0f));
        gpio_toggle(GPIOD, GPIO12);
        for (i = 0; i < 50000; i++)
            __asm__("nop");
    }

    return 0;
}
