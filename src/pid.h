#ifndef PID_H
#define PID_H
struct pid_state {
    float prev_error;
    float integral;
};

struct pid_params {
    float p;
    float i;
    float d;
    float i_max;
};
float pid(struct pid_state *state, struct pid_params k, float measured,
          float setpoint, float delta_t);
#endif
