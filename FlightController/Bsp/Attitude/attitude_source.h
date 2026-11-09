#ifndef ATTITUDE_SOURCE_H
#define ATTITUDE_SOURCE_H

#include <stdbool.h>
#include <stdint.h>

/* Body angles in degrees; body angular rates in degrees per second. */
typedef struct {
    float pitch, roll, yaw, gx, gy, gz;
    uint32_t sequence, tick;
} AttitudeOutput;

void AttitudeSource_Init(void);
bool AttitudeSource_IsFaulted(void);
bool AttitudeSource_Read(AttitudeOutput *output);

#endif
