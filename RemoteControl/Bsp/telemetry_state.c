#include "telemetry_state.h"

#define STATE_LOCKED 0xA1u
#define STATE_UNLOCKED 0xA2u
#define TRIM_VALID_TAG 0xB5u
#define STATE_TIMEOUT_MS 1000u

void TelemetryState_Reset(TelemetryState *state, uint32_t now_ms)
{
    state->value = 0;
    state->trim_valid = 0;
    state->pitch_zero_deci = 0;
    state->roll_zero_deci = 0;
    state->last_valid_ms = now_ms;
    state->last_trim_ms = now_ms;
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

void TelemetryState_UpdateTrim(TelemetryState *state, uint8_t tag, int8_t pitch_deci, int8_t roll_deci, uint32_t now_ms)
{
    if (tag == TRIM_VALID_TAG)
    {
        state->trim_valid = 1;
        state->pitch_zero_deci = pitch_deci;
        state->roll_zero_deci = roll_deci;
        state->last_trim_ms = now_ms;
    }
    else
    {
        state->trim_valid = 0;
    }
}

uint8_t TelemetryState_Expire(TelemetryState *state, uint32_t now_ms)
{
    uint8_t poll_stalled = (uint32_t)(now_ms - state->last_poll_ms) > STATE_TIMEOUT_MS;
    uint8_t lock_expired = state->value != 0 && (uint32_t)(now_ms - state->last_valid_ms) > STATE_TIMEOUT_MS;
    uint8_t trim_expired = state->trim_valid != 0 && (uint32_t)(now_ms - state->last_trim_ms) > STATE_TIMEOUT_MS;
    state->last_poll_ms = now_ms;
    if (poll_stalled || lock_expired)
    {
        state->value = 0;
        state->trim_valid = 0;
        return 1;
    }
    if (trim_expired)
    {
        state->trim_valid = 0;
    }
    return 0;
}

const char *TelemetryState_Label(const TelemetryState *state)
{
    if (state->value == STATE_LOCKED) return "LOCK";
    if (state->value == STATE_UNLOCKED) return "UNLOCK";
    return "UNKNOWN";
}

void TelemetryState_FormatZero(const TelemetryState *state, char axis, char *out, uint8_t out_size)
{
    uint8_t i = 0;
    if (out == 0 || out_size < 9u)
    {
        if (out != 0 && out_size > 0u) out[0] = '\0';
        return;
    }
    out[i++] = axis;
    out[i++] = ':';
    if (!state->trim_valid)
    {
        out[i++] = '-';
        out[i++] = '-';
        out[i++] = '.';
        out[i++] = '-';
    }
    else
    {
        int16_t deci = (axis == 'R') ? (int16_t)state->roll_zero_deci : (int16_t)state->pitch_zero_deci;
        uint16_t mag;
        uint16_t whole;
        if (deci < 0)
        {
            out[i++] = '-';
            mag = (uint16_t)(-deci);
        }
        else
        {
            out[i++] = '+';
            mag = (uint16_t)deci;
        }
        whole = mag / 10u;
        if (whole >= 10u)
        {
            out[i++] = (char)('0' + (whole / 10u) % 10u);
        }
        out[i++] = (char)('0' + (whole % 10u));
        out[i++] = '.';
        out[i++] = (char)('0' + (mag % 10u));
    }
    while (i < 8u)
    {
        out[i++] = ' ';
    }
    out[8] = '\0';
}
