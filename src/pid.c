#include "pid.h"
#include <math.h>
float pid(struct pid_state *state, struct pid_params k, float measured,
          float setpoint, float delta_t)
{
    float error = setpoint - measured;
    if (fabs(state->integral + error * delta_t) < k.i_max)
        state->integral += error * delta_t; 
    float derivative = (state->prev_error - error) / delta_t;
    state->prev_error = error;
    return k.p * error + k.i * state->integral + k.d * derivative;
}


