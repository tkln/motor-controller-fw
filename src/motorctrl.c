#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "pid.h"

#define ARRAY_LEN(a) (sizeof(a) / sizeof(a[0]))

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

#define FILTER_BUF_SIZE 7
#define AVG_BUF_SIZE 7

struct joint {
    const struct motor motor;
    const struct adc_pin pot;
    const struct adc_pin cur;
    struct pid_state pid_state;
    struct pid_params pid_params;
    float setpoint;
    float adc_angle;
    float adc_cur;
    float output;
    float prev_adc[FILTER_BUF_SIZE];
    float avg_buf[AVG_BUF_SIZE];
};

#include "config.h"

volatile int safemode = 1;
volatile int brake = 1;
volatile int gripper = 0;

const struct pin brake_relay_pin = {
    .port = GPIOE, .pin = GPIO4
};

const struct pin gripper_relay_pin = {
    .port = GPIOE, .pin = GPIO6
};

const struct pin motor_enable_pin = {
    .port = GPIOE, .pin = GPIO5
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
        if (data == '\r' || data == '\n') {
            usart_buf[usart_msg_len] = '\0';
            new_message = 1;
        }
    }
}

static void adc_input_init(struct adc_pin adc_pin)
{
    gpio_mode_setup(adc_pin.pin.port, GPIO_MODE_ANALOG, GPIO_PUPD_NONE,
                    adc_pin.pin.pin);
    adc_power_off(ADC1);
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
    gpio_mode_setup(output.pin.port, GPIO_MODE_AF, GPIO_PUPD_NONE, output.pin.pin);
    gpio_set_af(output.pin.port, output.pin.af, output.pin.pin);
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
    motor_dir_pin_init(motor.dir);
}

static void joint_init(struct joint joint)
{
    motor_init(joint.motor);
    adc_input_init(joint.pot);
    adc_input_init(joint.cur);
}

static void pwm_output_set(struct pwm_output output, float val)
{
    uint32_t period = timer_get_period(output.timer_peripheral);
    timer_set_oc_value(output.timer_peripheral, output.oc_id, period * val);
}

static void clock_setup(void)
{
    rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_120MHZ]);
}

static void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_GPIOE); /* dirs */
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
}

static void relay_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOD);
    gpio_mode_setup(brake_relay_pin.port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                    brake_relay_pin.pin);
    gpio_mode_setup(gripper_relay_pin.port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                    gripper_relay_pin.pin);
    gpio_mode_setup(motor_enable_pin.port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                    motor_enable_pin.pin);
}

static void timer_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOA); /* PWM */
    rcc_periph_clock_enable(RCC_TIM1); /* PWM */
    rcc_periph_reset_pulse(RST_TIM1);
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE,
                   TIM_CR1_DIR_UP);
    timer_enable_break_main_output(TIM1);
    timer_set_period(TIM1, 3000);
    timer_enable_counter(TIM1);

    rcc_periph_clock_enable(RCC_GPIOD); /* PWM */
    rcc_periph_clock_enable(RCC_TIM4); /* PWM */
    rcc_periph_reset_pulse(RST_TIM4);
    timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE,
                   TIM_CR1_DIR_UP);
    timer_enable_break_main_output(TIM4);
    timer_set_period(TIM4, 3000);
    timer_enable_counter(TIM4);

    rcc_periph_clock_enable(RCC_GPIOC); /* PWM */
    rcc_periph_clock_enable(RCC_GPIOB); /* PWM */
    rcc_periph_clock_enable(RCC_TIM3); /* PWM */
    rcc_periph_reset_pulse(RST_TIM3);
    timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE,
                   TIM_CR1_DIR_UP);
    timer_enable_break_main_output(TIM3);
    timer_set_period(TIM3, 3000);
    timer_enable_counter(TIM3);
}

static void adc_setup(void)
{
    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
    adc_power_off(ADC1);
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

static float pot_input_read(struct adc_pin pot)
{
    return read_adc_simple(pot.adc, pot.channel);
}

static float clamp(float val, float min, float max)
{
    if (val >= max)
        return max;
    if (val <= min)
        return min;
    return val;
}

static void set_motor(struct motor motor, float val)
{
    val = -val;

    val = clamp(val, -1.00f, 1.00f);
    if (isnan(val)) {
        pwm_output_set(motor.pwm, 0); /* stop the fet gate pulse */
        if (signbit(val)) /* brake */
            gpio_set(motor.dir.port, motor.dir.pin);
        else /* float */
            gpio_set(motor.dir.port, motor.dir.pin);
        return;
    }

    if (val < 0.0f)
        gpio_clear(motor.dir.port, motor.dir.pin);
    else
        gpio_set(motor.dir.port, motor.dir.pin);

    pwm_output_set(motor.pwm, fabs(val));
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

static inline void swap(float *a, float *b)
{
    float tmp;
    tmp = *a;
    *a = *b;
    *b = tmp;
}

static inline void sort(float *buf, size_t len)
{
    int sorted = 0;
    size_t i;
    while (!sorted) {
        sorted = 1;
        for (i = 0; i < len - 1; ++i) {
            if (buf[i] > buf[i + 1]) {
                sorted = 0;
                swap(buf + i, buf + i + 1);
            }
        }
    }
}

static float median_filter(struct joint *joint, float input)
{
    float filter_buf[FILTER_BUF_SIZE];
    memmove(joint->prev_adc, joint->prev_adc + 1, (FILTER_BUF_SIZE - 1) *
                                                  sizeof(float));
    joint->prev_adc[FILTER_BUF_SIZE - 1] = input;
    memcpy(filter_buf, joint->prev_adc, FILTER_BUF_SIZE * sizeof(float));
    sort(filter_buf, FILTER_BUF_SIZE);
    return filter_buf[FILTER_BUF_SIZE / 2];
}

static float avg_filter(struct joint *joint, float input)
{
    float filter_buf[AVG_BUF_SIZE];
    memmove(joint->avg_buf, joint->avg_buf + 1, (AVG_BUF_SIZE - 1) *
                                                  sizeof(float));
    joint->avg_buf[AVG_BUF_SIZE - 1] = input;
    memcpy(filter_buf, joint->avg_buf, AVG_BUF_SIZE * sizeof(float));
    float sum = 0.0f;
    size_t i;
    for (i = 0; i < AVG_BUF_SIZE; ++i)
        sum += filter_buf[i];
    return sum / AVG_BUF_SIZE;
}

static void joint_control(struct joint *joint, float delta_t)
{
    float measured = pot_input_read(joint->pot);
    float cur_measured = pot_input_read(joint->cur);
    float filtered = median_filter(joint, measured);
    float output = 0;

    filtered = avg_filter(joint, filtered);
    if (!safemode && !brake) {
        output = pid(&joint->pid_state, joint->pid_params, filtered,
                joint->setpoint, delta_t);
        joint->output = output;
        set_motor(joint->motor, output);
    }

    joint->adc_cur = cur_measured;
    joint->adc_angle = measured;
}

const char *state_msg_format = "j1: %f, j2: %f, j3: %f, j4: %f, j5: %f, j6: %f, safemode: %i, brake: %i, gripper: %i\n";

static void response(void)
{
    printf(state_msg_format, joints[0].adc_angle, joints[1].adc_angle,
           joints[2].adc_angle, joints[3].adc_angle, joints[4].adc_angle,
           joints[5].adc_angle, safemode, brake, gripper);
}

static void debug(void)
{
    size_t i;
    for (i = 0; i < ARRAY_LEN(joints); ++i)
        printf("m: %f, s: %f, o: %f, i: %f\r\n", joints[i].adc_angle,
               joints[i].setpoint, joints[i].output,
               joints[i].pid_state.integral);
}

static void set_setpoints(float *setpoints)
{
    size_t i;
    for (i = 0; i < ARRAY_LEN(joints); ++i)
        joints[i].setpoint = setpoints[i];
}

static void handle_msg(void)
{
    float setpoints[ARRAY_LEN(joints)];
    int ret;
    int new_safemode = 1, new_brake = 1, new_gripper = 1;
    char *msg = (char *)usart_buf;

    ret = sscanf(msg, state_msg_format, setpoints, setpoints + 1, setpoints + 2,
                setpoints + 3, setpoints + 4, setpoints + 5, &new_safemode,
                &new_brake, &new_gripper);

    if (ret == 9) {
        set_setpoints(setpoints);
        brake = new_brake;
        safemode = new_safemode;
        gripper = new_gripper;
    } else if (!strncmp(msg, "debug", 5)) {
        debug();
    } else {
        printf("invalid command\n");
    }

    response();
    new_message = 0;
    usart_msg_len = 0;
}

int main(void)
{
    unsigned i;
    const unsigned delay = 50000;

    clock_setup();
    gpio_setup();
    timer_setup();
    adc_setup();
    uart_setup();
    relay_setup();

    for (i = 0; i < ARRAY_LEN(joints); ++i)
        joint_init(joints[i]);

    printf("boot\n");

    while (1) {
        for (i = 0; i < delay; i++)
            __asm__("nop");

        if (new_message)
            handle_msg();

        gpio_toggle(GPIOD, GPIO12);
        for (i = 0; i < ARRAY_LEN(joints); ++i)
            joint_control(&joints[i], delay / 120000000.0f);

        if (brake)
            gpio_clear(brake_relay_pin.port, brake_relay_pin.pin);
        else
            gpio_set(brake_relay_pin.port, brake_relay_pin.pin);

        if (!gripper)
            gpio_set(gripper_relay_pin.port, gripper_relay_pin.pin);
        else
            gpio_clear(gripper_relay_pin.port, gripper_relay_pin.pin);

        if (brake || safemode)
            gpio_clear(motor_enable_pin.port, motor_enable_pin.pin);
        else
            gpio_set(motor_enable_pin.port, motor_enable_pin.pin);
    }
    return 0;
}
