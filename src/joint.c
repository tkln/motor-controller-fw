#include <stdint.h>
#include <string.h>

#include "joint.h"
#include "config.h"

struct pid_params joint_pid_params[] = {
    {
        .p = 10.0f,
        .i = 0.0f,
        .d = 0.0f,
        .i_max = 0.01f
    }, {
        .p = 50.0f,
        .i = 10.0f,
        .d = 0.0f,
        .i_max = 0.01f
    }, {
        .p = 50.0f,
        .i = 10.0f,
        .d = 0.0f,
        .i_max = 0.01f
    }, {
        .p = 15.0f,
        .i = 10.0f,
        .d = 0.0f,
        .i_max = 0.01f
    }, {
        .p = 15.0f,
        .i = 10.0f,
        .d = 0.0f,
        .i_max = 0.01f
    }, {
        .p = 10.0f,
        .i = 0.0f,
        .d = 0.0f,
        .i_max = 0.01f
    }
};

struct joint_state joint_states[] = {
    { /* joint 1 */
        .pid_state = {
            .prev_error = 0.0f,
            .integral = 0.0f
        },
        .setpoint = 0.49f,
        .angle = NAN
    },
    { /* joint 2 */
        .pid_state = {
            .prev_error = 0.0f,
            .integral = 0.0f
        },
        .setpoint = 0.49f,
        .angle = NAN
    },
    { /* joint 3 */
        .pid_state = {
            .prev_error = 0.0f,
            .integral = 0.0f
        },
        .setpoint = 0.49f,
        .angle = NAN
    },
    { /* joint 4 */
        .pid_state = {
            .prev_error = 0.0f,
            .integral = 0.0f
        },
        .setpoint = 0.49f,
        .angle = NAN
    },
    { /* joint 5 */
        .pid_state = {
            .prev_error = 0.0f,
            .integral = 0.0f
        },
        .setpoint = 0.49f,
        .angle = NAN
    },
    { /* joint 6 */
        .pid_state = {
            .prev_error = 0.0f,
            .integral = 0.0f
        },
        .setpoint = 0.49f,
        .angle = NAN
    }
};

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

static void adc_input_init(struct adc_pin adc_pin)
{
    gpio_mode_setup(adc_pin.pin.port, GPIO_MODE_ANALOG, GPIO_PUPD_NONE,
                    adc_pin.pin.pin);
    adc_power_off(ADC1);
    adc_disable_scan_mode(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_3CYC);
    adc_power_on(ADC1);
}

void joint_init(struct joint_hw joint_hw)
{
    motor_init(joint_hw.motor);
    adc_input_init(joint_hw.pot);
    adc_input_init(joint_hw.cur);
}

static uint32_t timer_get_period(uint32_t timer_peripheral)
{
    return TIM_ARR(timer_peripheral);
}

static void pwm_output_set(struct pwm_output output, float val)
{
    uint32_t period = timer_get_period(output.timer_peripheral);
    timer_set_oc_value(output.timer_peripheral, output.oc_id, period * val);
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

void joint_drive(const struct joint_hw *joint_hw,
                 const struct pid_params *pid_params,
                 struct joint_state *joint,
                 float delta_t)
{
    float output = 0;

    output = pid(&joint->pid_state, *pid_params, joint->angle,
                 joint->setpoint, delta_t);
    joint->output = output;
    set_motor(joint_hw->motor, output);
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

static float median_filter(struct joint_state *joint, float input)
{
    float filter_buf[FILTER_BUF_SIZE];
    memmove(joint->prev_adc, joint->prev_adc + 1, (FILTER_BUF_SIZE - 1) *
                                                  sizeof(float));
    joint->prev_adc[FILTER_BUF_SIZE - 1] = input;
    memcpy(filter_buf, joint->prev_adc, FILTER_BUF_SIZE * sizeof(float));
    sort(filter_buf, FILTER_BUF_SIZE);
    return filter_buf[FILTER_BUF_SIZE / 2];
}

static float avg_filter(struct joint_state *joint, float input)
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

static float adc_read(struct adc_pin pot)
{
    return read_adc_simple(pot.adc, pot.channel);
}

void joint_measure_angle(const struct joint_hw *joint_hw,
                         struct joint_state *joint)
{
    float angle = adc_read(joint_hw->pot);

    angle = median_filter(joint, angle);
    angle = avg_filter(joint, angle);
    joint->angle = angle;
}

void joint_measure_current(const struct joint_hw *joint_hw,
                           struct joint_state *joint)
{
    joint->current = adc_read(joint_hw->cur); /* TODO Scaling */
}


