#include <math.h>

#include "angle.h"

#define DEG2RAD(d) ((d) * M_PI / 180.0f)

static const struct conv_params {
    float adc_a;
    float adc_b;
    float angle_a;
    float angle_b;
} joint_conversions[] = {
    {       /* j1 */
        0.263184,           0.833252,
        DEG2RAD(-160.0f),   DEG2RAD(160.0f)
    }, {    /* j2 */
        0.297259,           0.814907,
        DEG2RAD(180.0f),    DEG2RAD(0.0f)
    }, {    /* j3 */
        0.693185,           0.432826,
        DEG2RAD(0.0f),      DEG2RAD(-180)
    }, {    /* j4 */
        0.740479,           0.374268,
        DEG2RAD(-90),       DEG2RAD(90)
    }, {    /* j5 */
        0.385498,           0.733398,
        DEG2RAD(-90),       DEG2RAD(90)
    }, {    /* j6 */
        0.145020,           0.885498,
        DEG2RAD(-180),      DEG2RAD(180)
    }
};

static inline float linear_map(float x, float x0, float x1, float y0, float y1)
{
    return (x - x0) * (y1 - y0) / (x1 - x0) + y0;
}

float joint_adc2rad(int joint, float adc)
{
    const struct conv_params *p = joint_conversions + joint;
    return linear_map(adc, p->adc_a, p->adc_b, p->angle_a, p->angle_b);
}

float joint_rad2adc(int joint, float rad)
{
    const struct conv_params *p = joint_conversions + joint;
    return linear_map(rad, p->angle_a, p->angle_b, p->adc_a, p->adc_b);
}
