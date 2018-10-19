#ifndef JOINT_H
#define JOINT_H

#include "config.h"

extern struct joint_state {
    struct pid_state pid_state;
    float setpoint;
    float angle;
    float current;
    float output;
    float prev_adc[FILTER_BUF_SIZE];
    float avg_buf[AVG_BUF_SIZE];
} joint_states[NUM_JOINTS];

extern struct pid_params joint_pid_params[NUM_JOINTS];

void joint_init(struct joint_hw joint_hw);
void joint_drive(const struct joint_hw *joint_hw,
                 const struct pid_params *pid_params,
                 struct joint_state *joint,
                 float delta_t);
void joint_measure_current(const struct joint_hw *joint_hw,
                           struct joint_state *joint);
void joint_measure_angle(const struct joint_hw *joint_hw,
                         struct joint_state *joint);

#endif
