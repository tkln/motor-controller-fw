#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <stdlib.h>

struct pwm_output {
    uint32_t timer_peripheral; /* e.g. TIM1 */
    enum tim_oc_id oc_id; /* e.g. TIM_OC1 */
};

static void pwm_output_init(struct pwm_output output)
{
    timer_set_oc_mode(output.timer_peripheral, output.oc_id, TIM_OCM_PWM2);
    timer_enable_oc_output(output.timer_peripheral, output.oc_id);
}

static const struct pwm_output motor_outputs[] = {
    {TIM1, TIM_OC1},
    {TIM1, TIM_OC2}
};

static void clock_setup(void)
{
    rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOE);
    rcc_periph_clock_enable(RCC_TIM1);
}

static void gpio_setup(void)
{
    gpio_mode_setup(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    gpio_mode_setup(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11);
    gpio_set_af(GPIOE, GPIO_AF1, GPIO9 );
    gpio_set_af(GPIOE, GPIO_AF1, GPIO11);
}

static void timer_setup(void)
{
    size_t i;
    rcc_periph_clock_enable(RCC_TIM1);
    timer_reset(TIM1);
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1,
    TIM_CR1_DIR_UP);
    for (i = 0; i < sizeof(motor_outputs)/sizeof(motor_outputs[0]); ++i)
        pwm_output_init(motor_outputs[i]);
    timer_enable_break_main_output(TIM1);
    timer_set_oc_value(TIM1, TIM_OC1, 1500);
    timer_set_oc_value(TIM1, TIM_OC2, 500);
    timer_set_period(TIM1, 4000);
    timer_enable_counter(TIM1);
}

int main(void)
{
    int i;

    clock_setup();
    gpio_setup();
    timer_setup();

    while (1)
        for (i = 0; i < 6000000; i++) 
            __asm__("nop");

    return 0;
}
