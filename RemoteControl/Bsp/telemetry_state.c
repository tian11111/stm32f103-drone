#include "telemetry_state.h"

#define STATE_LOCKED 0xA1u
#define STATE_UNLOCKED 0xA2u
#define STATE_TIMEOUT_MS 1000u

void TelemetryState_Reset(TelemetryState *state, uint32_t now_ms)
{
    state->value = 0;
    state->last_poll_ms = now_ms;
}

void TelemetryState_Update(TelemetryState *state, uint8_t value, uint32_t now_ms)
{
    if (value == STATE_LOCKED || value == STATE_UNLOCKED)
    {
        state->value = value;
        state->last_valid_ms = now_ms;
    }
    else
    {
        state->value = 0;
    }
}

uint8_t TelemetryState_Expire(TelemetryState *state, uint32_t now_ms)
{
    uint8_t expired = (uint32_t)(now_ms - state->last_poll_ms) > STATE_TIMEOUT_MS ||
        (state->value != 0 && (uint32_t)(now_ms - state->last_valid_ms) > STATE_TIMEOUT_MS);
    state->last_poll_ms = now_ms;
    if (expired)
    {
        state->value = 0;
        return 1;
    }
    return 0;
}

const char *TelemetryState_Label(const TelemetryState *state)
{
    if (state->value == STATE_LOCKED) return "LOCK";
    if (state->value == STATE_UNLOCKED) return "UNLOCK";
    return "UNKNOWN";
}
