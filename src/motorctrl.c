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

static void pwm_output_set(struct pwm_output output, float val)
{
    uint32_t period = timer_get_period(output.timer_peripheral);
    timer_set_oc_value(output.timer_peripheral, output.oc_id, period * val);
}


static const struct pwm_output motor_outputs[] = {
    {TIM1, TIM_OC1},
    {TIM1, TIM_OC2}
};

static void clock_setup(void)
{
    rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_120MHZ]);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOE); /* PWM */
    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_TIM1); /* PWM */
    rcc_periph_clock_enable(RCC_GPIOA); /* USB */
    rcc_periph_clock_enable(RCC_OTGFS); /* USB */

}

static void gpio_setup(void)
{
    gpio_mode_setup(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    gpio_mode_setup(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11);
    gpio_set_af(GPIOE, GPIO_AF1, GPIO9 );
    gpio_set_af(GPIOE, GPIO_AF1, GPIO11);
    rcc_periph_clock_enable(RCC_GPIOD);
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
}

static void timer_setup(void)
{
    size_t i;
    rcc_periph_clock_enable(RCC_TIM1);
    timer_reset(TIM1);
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE,
                   TIM_CR1_DIR_UP);
    for (i = 0; i < sizeof(motor_outputs)/sizeof(motor_outputs[0]); ++i)
        pwm_output_init(motor_outputs[i]);
    timer_enable_break_main_output(TIM1);
    timer_set_period(TIM1, 6000);
    timer_enable_counter(TIM1);
}

static void adc_setup(void)
{
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

static void set_motor(struct pwm_output a, struct pwm_output b, float val)
{
    (void) b;
    pwm_output_set(a, val);
    /*
    if (val < 0) {
        pwm_output_set(a, val + 1);
        pwm_output_set(b, val);
    }
    else {
        pwm_output_set(a, val);
        pwm_output_set(b, 1.0f - val);
    }
    */
}

int main(void)
{
    int i;

    clock_setup();
    gpio_setup();
    timer_setup();
    adc_setup();

    float setpoint = 0.5f;
    float error;
    float integral = 0;
    float derivative = 0;
    float prev_error = 0;
    float val;

    while (1) {
        error = setpoint - read_adc_simple(0);
        integral += error * 0.01f;
        derivative = error - prev_error;

        val = 3 * error + integral + derivative * 20.0f;

        pwm_output_set(motor_outputs[1], 1.0f - val);
        pwm_output_set(motor_outputs[0], val);
        gpio_toggle(GPIOD, GPIO12);
        for (i = 0; i < 50000; i++)
            __asm__("nop");
        prev_error = error;
    }

    return 0;
}
