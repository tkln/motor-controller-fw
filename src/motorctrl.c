#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "pid.h"
#include "ringbuf.h"
#include "joint.h"

#define ARRAY_LEN(a) (sizeof(a) / sizeof(a[0]))

#include "config.h"

volatile int safemode = 1;
volatile int brake = 1;
volatile int gripper = 0;


#define USART_BUF_LEN 512 + 1

static char msg_queue_buf[1024];
static struct ringbuf msg_queue;
volatile int new_messages = 0;


void usart3_isr(void)
{
    static uint8_t data;
    static uint8_t msg_len = 0;
    static size_t msg_len_pos = 0;

    if (!((USART_CR1(USART3) & USART_CR1_RXNEIE) &&
        (USART_SR(USART3) & USART_SR_RXNE)))
        return;

    data = usart_recv(USART3);

    if (ringbuf_full(&msg_queue))
        return;

    ringbuf_write(&msg_queue, &data, sizeof(data));
    ++msg_len;

    if (data == '\r' || data == '\n') {
        /* Round even to keep length fields aligned */
        if (msg_len & 1) {
            ringbuf_write(&msg_queue, "", 1);
            ++msg_len;
        }

        msg_queue.buf[msg_len_pos] = msg_len;

        msg_len = 0;
        ++new_messages;
        msg_len_pos = ringbuf_mask(&msg_queue, msg_queue.head);
        ringbuf_write(&msg_queue, &msg_len, sizeof(msg_len));
    }
}

struct cmd {
    float setpoints[6];
    bool safemode;
    bool brake;
    bool gripper;
    int dt; /* ms */
};

static char cmd_queue_buf[512];
static struct ringbuf cmd_queue;

volatile uint32_t system_millis;

void sys_tick_handler(void)
{
    ++system_millis;
}

static void systick_setup(void)
{
    /* 1 ms interrupt rate */
    systick_set_reload(120000);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
    systick_interrupt_enable();
}

static void msleep(uint32_t delay)
{
    uint32_t wake = system_millis + delay;
    while (system_millis < wake)
        ;
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

static void uart_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOD);
    gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO9);
    gpio_set_af(GPIOD, GPIO_AF7, GPIO8 | GPIO9);
    rcc_periph_clock_enable(RCC_USART3);
    usart_set_baudrate(USART3, 576000);
    usart_set_databits(USART3, 8);
    usart_set_stopbits(USART3, USART_STOPBITS_1);
    usart_set_mode(USART3, USART_MODE_TX_RX);
    usart_set_parity(USART3, USART_PARITY_NONE);
    usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
    usart_enable(USART3);
    nvic_enable_irq(NVIC_USART3_IRQ);
    usart_enable_rx_interrupt(USART3);
}

/* Start of the 11th (last) general purpose flash sector */
#define PID_PARAM_FLASH_ADDR 0x080e0000
#define PID_PARAM_FLASH_SECTOR 11

static void save_pid_params(void)
{
    flash_unlock();
    /* XXX This may freeze if done more than once after reset */
    flash_erase_sector(PID_PARAM_FLASH_SECTOR, 2); /* TODO Check status */
    flash_program(PID_PARAM_FLASH_ADDR, (void *)&joint_pid_params,
                  sizeof(joint_pid_params));
}

static void load_pid_params(void)
{
    memcpy(joint_pid_params, (void *)PID_PARAM_FLASH_ADDR,
           sizeof(joint_pid_params));
}

const char *state_msg_format = "angle: j1: %f, j2: %f, j3: %f, j4: %f, j5: %f, j6: %f, safemode: %i, brake: %i, gripper: %i, dt: %i\n";

const char *pid_param_msg_format = "pid: j: %d, p: %f, i: %f, d: %f, i_max: %f\n";

static void print_response(void)
{
    printf(state_msg_format,
           joint_states[0].angle, joint_states[1].angle,
           joint_states[2].angle, joint_states[3].angle,
           joint_states[4].angle, joint_states[5].angle,
           safemode, brake, gripper);
}

static void print_debug(void)
{
    size_t i;
    for (i = 0; i < ARRAY_LEN(joint_states); ++i)
        printf("m: %f, s: %f, o: %f, i: %f\r\n", joint_states[i].angle,
               joint_states[i].setpoint, joint_states[i].output,
               joint_states[i].pid_state.integral);
}

static void print_currents(void)
{
    printf("current: j1: %f, j2: %f, j3: %f, j4: %f, j5: %f, j6: %f\n",
           joint_states[0].current, joint_states[1].current,
           joint_states[2].current, joint_states[3].current,
           joint_states[4].current, joint_states[5].current);
}

static void set_setpoints(float *setpoints)
{
    size_t i;
    for (i = 0; i < ARRAY_LEN(joint_states); ++i)
        joint_states[i].setpoint = setpoints[i];
}

static void print_pid_params(void)
{
    size_t i;
    struct pid_params *params;

    for (i = 0; i < ARRAY_LEN(joint_states); ++i) {
        params = joint_pid_params + i;
        printf(pid_param_msg_format, i + 1, params->p, params->i, params->d,
               params->i_max);
    }
}

static void print_cmd_queue_status(void)
{
    printf("buffer used: %zu/%zu\n",
           ringbuf_space_used(&cmd_queue) / sizeof(struct cmd),
           ringbuf_capacity(&cmd_queue) / sizeof(struct cmd));
}

static void handle_msg(void)
{
    static char msg[256];
    uint8_t msg_len;
    struct cmd cmd;
    struct pid_params params;
    size_t sz;
    int joint;
    int ret;

    ringbuf_read(&msg_queue, &msg_len, sizeof(msg_len));
    ringbuf_read(&msg_queue, &msg, msg_len);
    --new_messages;
    
    ret = sscanf(msg, state_msg_format,
                 cmd.setpoints, cmd.setpoints + 1,
                 cmd.setpoints + 2, cmd.setpoints + 3,
                 cmd.setpoints + 4, cmd.setpoints + 5,
                 &cmd.safemode, &cmd.brake, &cmd.gripper, &cmd.dt);

    if (ret == 10) {
        sz = ringbuf_space_avail(&cmd_queue);
        if (sz < sizeof(cmd)) {
            printf("buffer overrun\n");
            goto out;
        }
        ringbuf_write(&cmd_queue, &cmd, sizeof(cmd));
        print_cmd_queue_status();
        goto out;
    }

    ret = sscanf(msg, pid_param_msg_format, &joint, &params.p, &params.i,
                 &params.d, &params.i_max);
    if (ret == 5) {
        joint_pid_params[joint - 1] = params;
        print_pid_params();
        goto out;
    }

    switch (msg[0]) {
    case 'l':
        load_pid_params();
        print_pid_params();
        goto out;
    case 'b':
        print_cmd_queue_status();
        goto out;
    case 's':
        save_pid_params();
        print_pid_params();
        goto out;
    case 'p':
        print_pid_params();
        goto out;
    case 'c':
        print_currents();
        goto out;
    case 'd':
        print_debug();
        goto out;
    case '\r':
    case '\n':
        print_response();
        goto out;
    default:
        break;
    }

    printf("invalid command\n");

out:
    gpio_set(GPIOD, GPIO12);
    msleep(1);
    gpio_clear(GPIOD, GPIO12);

}

static inline float lerp(float a, float b, float x)
{
    return a + x * (b - a);
}


int main(void)
{
    uint32_t prev_millis = 0;
    struct cmd cur_cmd;
    float start_angles[6];
    float setpoints[6];
    int cmd_dt = 0;
    unsigned i;
    size_t sz;
    int dt;

    clock_setup();
    systick_setup();
    gpio_setup();
    timer_setup();
    adc_setup();
    uart_setup();
    relay_setup();

    for (i = 0; i < ARRAY_LEN(joint_hws); ++i)
        joint_init(joint_hws[i]);

    ringbuf_init(&msg_queue, msg_queue_buf, 10); /* 1<<10 == 1024 */
    ringbuf_init(&cmd_queue, cmd_queue_buf, 9); /* 1<<9 == 512 */
    ringbuf_write(&msg_queue, "", 1); /* Place a padding byte */

    printf("boot\n");

    while (1) {
        dt = system_millis - prev_millis;
        prev_millis = system_millis;

        msleep(10);

        while (new_messages)
            handle_msg();

        for (i = 0; i < ARRAY_LEN(joint_states); ++i) {
            joint_measure_current(joint_hws + i, joint_states + i);
            joint_measure_angle(joint_hws + i, joint_states + i);

            if (!safemode && !brake) {
                joint_drive(joint_hws + i, joint_pid_params + i,
                            joint_states + i, dt * 0.001);
            }
        }

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

        if (cmd_dt <= 0) {
            /* 
             * Maybe this should also wait for the joints to actually reach the
             * setpoints within a given margin?
             */
            sz = ringbuf_space_used(&cmd_queue);
            if (sz < sizeof(cur_cmd)) {
                /* Use current state as starting point if the buffer is empty */
                for (i = 0; i < 6; ++i)
                    start_angles[i] = joint_states[i].angle;
                continue;
            } else {
                /* Use the old setpoins as the start angles */
                for (i = 0; i < ARRAY_LEN(joint_states); ++i)
                    start_angles[i] = setpoints[i];
            }

            ringbuf_read(&cmd_queue, &cur_cmd, sizeof(cur_cmd));

            cmd_dt += cur_cmd.dt;

        }

        for (i = 0; i < ARRAY_LEN(setpoints); ++i)
            /* The parameters are that way around because cmd_dt counts down. */
            setpoints[i] = lerp(cur_cmd.setpoints[i], start_angles[i],
                                (float)cmd_dt / cur_cmd.dt);
        set_setpoints(setpoints);
        brake = cur_cmd.brake;
        safemode = cur_cmd.safemode;
        gripper = cur_cmd.gripper;

        cmd_dt -= dt;
    }
    return 0;
}
