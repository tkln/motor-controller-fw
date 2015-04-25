#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <stdlib.h>
#include <stdio.h>
#include "spinlock.h"

static uint32_t timer_get_period(uint32_t timer_peripheral)
{
    return TIM_ARR(timer_peripheral);
}

struct pwm_output {
    uint32_t timer_peripheral; /* e.g. TIM1 */
    enum tim_oc_id oc_id; /* e.g. TIM_OC1 */
};

struct joint {
    struct pwm_output pwm;
    uint32_t dira_pin;
    uint32_t dirb_pin;
};

static void pwm_output_init(struct pwm_output output)
{
    timer_set_oc_mode(output.timer_peripheral, output.oc_id, TIM_OCM_PWM1);
    timer_enable_oc_output(output.timer_peripheral, output.oc_id);
}


static void motor_init(struct joint joint)
{
    pwm_output_init(joint.pwm);
    //pwm_output_init(motor.b);
}

static void pwm_output_set(struct pwm_output output, float val)
{
    uint32_t period = timer_get_period(output.timer_peripheral);
    timer_set_oc_value(output.timer_peripheral, output.oc_id, period * val);
}

static const struct joint joints[] = {
    {{TIM4, TIM_OC1}, }
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
    rcc_periph_clock_enable(RCC_GPIOD); /* PWM */
    rcc_periph_clock_enable(RCC_TIM4); /* PWM */
    gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12);
    gpio_set_af(GPIOD, GPIO_AF2, GPIO12);
    rcc_periph_clock_enable(RCC_TIM4);
    timer_reset(TIM4);
    timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE,
                   TIM_CR1_DIR_UP);
    for (i = 0; i < sizeof(joints)/sizeof(joints[0]); ++i)
        motor_init(joints[i]);
    timer_enable_break_main_output(TIM4);
    timer_set_period(TIM4, 12000);
    timer_enable_counter(TIM4);
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

static void set_motor(struct joint joint, float val)
{
    pwm_output_set(joint.pwm, val);
    //pwm_output_set(motor.b, 1.0f - val);
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

static void uart_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOD); 
    gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO9);
    gpio_set_af(GPIOD, GPIO_AF7, GPIO8 | GPIO9);
    rcc_periph_clock_enable(RCC_USART3);
    usart_set_baudrate(USART3, 115200);
    usart_set_databits(USART3, 8);
    usart_set_stopbits(USART3, USART_STOPBITS_1);
    usart_set_mode(USART3, USART_MODE_TX_RX);
    usart_set_parity(USART3, USART_PARITY_NONE);
    usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
    usart_enable(USART3);
    //nvic_enable_irq(NVIC_USART3_IRQ);
    //usart_enable_rx_interrupt(USART3);
}

/*
static void usart_putc(char c)
{
    while (!(USART_SR(USART3) & USART_SR_TXE))
        ;
    USART_DR(USART3) = (uint16_t) c & 0xff;
}
*/

#define RECV_BUFFER_SIZE 128
char recv_buffer[RECV_BUFFER_SIZE];
volatile unsigned int recv_w_idx;

void usart3_isr(void)
{
    recv_buffer[recv_w_idx++ % RECV_BUFFER_SIZE] = USART_DR(USART3);
}

int main(void)
{
    int i;
    float duty = 0.1f;

    clock_setup();
    gpio_setup();
    timer_setup();
    adc_setup();
    uart_setup();

    /*
    float setpoint = 0.75f;

    struct pid_state state = {.prev_error = 0.0f, .integral = 0.0f};
    struct pid_params params = {.p = 10.0f, .i = 10.0f, .d = 0.0f};
    */
    printf("hullo\n\r");

    pwm_output_set(joints[0].pwm, duty);
    while (1) {
        //usart_putc('e');
        //printf("lel\n");
        //setpoint += 0.01;
        /* 
        set_motor(motors[0], pid(&state, params, read_adc_simple(0), setpoint,
                  10000.0f / 120000000.0f));
                  */
        //set_motor(motors[0], 0.5);
        //gpio_toggle(GPIOD, GPIO12);
        for (i = 0; i < 50000; i++)
            __asm__("nop");
        gpio_toggle(GPIOD, GPIO14);
        if (scanf("%f", &duty) == 1) {
            printf("%f\n", duty);
            pwm_output_set(joints[0].pwm, duty);
        }
            
    }

    return 0;
}
