#ifndef TELEMETRY_STATE_H
#define TELEMETRY_STATE_H

#include <stdint.h>

typedef struct
{
    uint32_t last_valid_ms;
    uint32_t last_trim_ms;
    uint32_t last_poll_ms;
    uint8_t value;
    uint8_t trim_valid;
    int8_t pitch_zero_deci;
    int8_t roll_zero_deci;
} TelemetryState;

void TelemetryState_Reset(TelemetryState *state, uint32_t now_ms);
void TelemetryState_Update(TelemetryState *state, uint8_t value, uint32_t now_ms);
void TelemetryState_UpdateTrim(TelemetryState *state, uint8_t tag, int8_t pitch_deci, int8_t roll_deci, uint32_t now_ms);
uint8_t TelemetryState_Expire(TelemetryState *state, uint32_t now_ms);
const char *TelemetryState_Label(const TelemetryState *state);
void TelemetryState_FormatZero(const TelemetryState *state, char axis, char *out, uint8_t out_size);

#endif
