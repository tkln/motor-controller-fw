#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <stdlib.h>
#include <stdio.h>
#include "spinlock.h"
#include "pid.h"

struct pin {
    uint32_t port;
    uint16_t pin;
};

struct pwm_output { /* TODO add pin */
    uint32_t timer_peripheral; /* e.g. TIM1 */
    enum tim_oc_id oc_id; /* e.g. TIM_OC1 */
};

struct motor {
    struct pwm_output pwm;
    struct pin dira;
    struct pin dirb;
};

struct pot {
    uint32_t adc;
    uint8_t channel;
    struct pin pin;
};

struct joint {
    struct motor motor;
    struct pot pot;
    struct pid_state pid_state;
    struct pid_params pid_params;
    float setpoint;
};

#define USART_BUF_LEN 128 + 1

volatile int new_message = 0;
volatile uint8_t usart_buf[USART_BUF_LEN] = {0};
volatile uint16_t usart_msg_len = 0;

void usart3_isr(void)
{
    static uint8_t data;
    if ((USART_CR1(USART3) & USART_CR1_RXNEIE) &&
        (USART_SR(USART3) & USART_SR_RXNE)) {
        usart_msg_len %= (USART_BUF_LEN - 1);
        data = usart_recv(USART3);
        usart_buf[usart_msg_len++] = data;
        if (data == '\r') {
            usart_buf[usart_msg_len] = '\0';
            new_message = 1;
        }
    }
}

static void pot_input_init(struct pot pot)
{
    gpio_mode_setup(pot.pin.port, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, pot.pin.pin);
    adc_off(ADC1);
    adc_disable_scan_mode(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_3CYC);
    adc_power_on(ADC1);
 
}

static uint32_t timer_get_period(uint32_t timer_peripheral)
{
    return TIM_ARR(timer_peripheral);
}

static void pwm_output_init(struct pwm_output output)
{
    timer_set_oc_mode(output.timer_peripheral, output.oc_id, TIM_OCM_PWM1);
    timer_enable_oc_output(output.timer_peripheral, output.oc_id);
}

static void motor_dir_pin_init(struct pin pin)
{
    gpio_mode_setup(pin.port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, pin.pin);
}

static void motor_init(struct motor motor)
{
    pwm_output_init(motor.pwm);
    motor_dir_pin_init(motor.dira);
    motor_dir_pin_init(motor.dirb);
}

static void joint_init(struct joint joint)
{
    motor_init(joint.motor);
    pot_input_init(joint.pot);
}

static void pwm_output_set(struct pwm_output output, float val)
{
    uint32_t period = timer_get_period(output.timer_peripheral);
    timer_set_oc_value(output.timer_peripheral, output.oc_id, period * val);
}

static void clock_setup(void)
{
    rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_120MHZ]);
}

static void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_GPIOE); /* dirs */
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
}

static void timer_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOD); /* PWM */
    rcc_periph_clock_enable(RCC_TIM4); /* PWM */
    gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12);
    gpio_set_af(GPIOD, GPIO_AF2, GPIO12);
    rcc_periph_clock_enable(RCC_TIM4);
    timer_reset(TIM4);
    timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE,
                   TIM_CR1_DIR_UP);
    gpio_set(joints[0].motor.dira.port, joints[0].motor.dira.pin);
    timer_enable_break_main_output(TIM4);
    timer_set_period(TIM4, 3000);
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

static float read_adc_simple(uint32_t adc, uint8_t channel)
{
    uint8_t channel_array[16];
    channel_array[0] = channel;
    adc_set_regular_sequence(adc, 1, channel_array);
    adc_start_conversion_regular(adc);
    while (!adc_eoc(adc))
        ;
    uint16_t reg16 = adc_read_regular(adc);
    return (float)reg16 / (1<<12);
}

static float pot_input_read(struct pot pot)
{
    return read_adc_simple(pot.adc, pot.channel);
}

static void set_motor(struct motor motor, float val)
{
    if (val >= 1.0f)
        val = 0.99f;
    if (-val <= -1.0f)
        val = -0.99f;
    if (val > 0.0f) {
        pwm_output_set(motor.pwm, 1 - val);
        gpio_set(motor.dira.port, motor.dira.pin);
        gpio_clear(motor.dirb.port, motor.dirb.pin);
    }
    else {
        pwm_output_set(motor.pwm, 1 - (-val));
        gpio_set(motor.dirb.port, motor.dirb.pin);
        gpio_clear(motor.dira.port, motor.dira.pin);
    }
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
    nvic_enable_irq(NVIC_USART3_IRQ);
    usart_enable_rx_interrupt(USART3);
}


#define DEBUG

static void joint_control(struct joint *joint, float delta_t)
{
    float measured = pot_input_read(joint->pot);
    float output = pid(&joint->pid_state, joint->pid_params, measured,
                       joint->setpoint, delta_t);
    #ifdef DEBUG
    printf("m: %f, s: %f, o: %f, i: %f\n\r", measured, joint->setpoint,
            output, joint->pid_state.integral);
    #endif
    set_motor(joint->motor, output);
}

int main(void)
{
    int i;

    clock_setup();
    gpio_setup();
    timer_setup();
    adc_setup();
    uart_setup();

    for (i = 0; (unsigned)i < sizeof(joints)/sizeof(joints[0]); ++i)
        joint_init(joints[i]);

    printf("hullo\n\r");

    while (1) {
        for (i = 0; i < 500000; i++)
            __asm__("nop");
        joint_control(&joints[0], 500000.0f / 120000000.0f);
        if (new_message) {
            printf("\\o/\n\r");
            sscanf((const char *)usart_buf, "%f", &joints[0].setpoint);
            new_message = 0;
            usart_msg_len = 0;
        }
    }

    return 0;
}
