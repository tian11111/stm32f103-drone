#ifndef TELEMETRY_STATE_H
#define TELEMETRY_STATE_H

#include <stdint.h>

typedef struct
{
    uint32_t last_valid_ms;
    uint32_t last_poll_ms;
    uint8_t value;
} TelemetryState;

void TelemetryState_Reset(TelemetryState *state, uint32_t now_ms);
void TelemetryState_Update(TelemetryState *state, uint8_t value, uint32_t now_ms);
uint8_t TelemetryState_Expire(TelemetryState *state, uint32_t now_ms);
const char *TelemetryState_Label(const TelemetryState *state);

#endif
