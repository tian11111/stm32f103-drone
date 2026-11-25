#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "telemetry_state.h"

int main(void)
{
    TelemetryState state = {0};
    char pbuf[9], rbuf[9];

    TelemetryState_Reset(&state, 100);
    assert(strcmp(TelemetryState_Label(&state), "UNKNOWN") == 0);
    TelemetryState_FormatZero(&state, 'P', pbuf, sizeof(pbuf));
    TelemetryState_FormatZero(&state, 'R', rbuf, sizeof(rbuf));
    assert(strcmp(pbuf, "P:--.-  ") == 0 && strcmp(rbuf, "R:--.-  ") == 0);

    TelemetryState_Update(&state, 0xA2, 100);
    TelemetryState_UpdateTrim(&state, 0xB5, -5, 100, 100);
    assert(strcmp(TelemetryState_Label(&state), "UNLOCK") == 0);
    TelemetryState_FormatZero(&state, 'P', pbuf, sizeof(pbuf));
    TelemetryState_FormatZero(&state, 'R', rbuf, sizeof(rbuf));
    assert(strcmp(pbuf, "P:-0.5  ") == 0 && strcmp(rbuf, "R:+10.0 ") == 0);

    assert(!TelemetryState_Expire(&state, 1100));
    assert(TelemetryState_Expire(&state, 1101));
    assert(strcmp(TelemetryState_Label(&state), "UNKNOWN") == 0);
    TelemetryState_FormatZero(&state, 'P', pbuf, sizeof(pbuf));
    assert(strcmp(pbuf, "P:--.-  ") == 0);

    TelemetryState_Update(&state, 0xA1, 1101);
    TelemetryState_UpdateTrim(&state, 0xB5, -100, 0, 1101);
    assert(strcmp(TelemetryState_Label(&state), "LOCK") == 0);
    TelemetryState_FormatZero(&state, 'P', pbuf, sizeof(pbuf));
    TelemetryState_FormatZero(&state, 'R', rbuf, sizeof(rbuf));
    assert(strcmp(pbuf, "P:-10.0 ") == 0 && strcmp(rbuf, "R:+0.0  ") == 0);

    /* Extreme int8_t (-128) must format without signed overflow. */
    TelemetryState_UpdateTrim(&state, 0xB5, -128, 127, 1102);
    TelemetryState_FormatZero(&state, 'P', pbuf, sizeof(pbuf));
    TelemetryState_FormatZero(&state, 'R', rbuf, sizeof(rbuf));
    assert(strcmp(pbuf, "P:-12.8 ") == 0 && strcmp(rbuf, "R:+12.7 ") == 0);

    /* Legacy FC with tag=0 invalidates trim display. */
    TelemetryState_UpdateTrim(&state, 0x00, 0, 0, 1102);
    TelemetryState_FormatZero(&state, 'P', pbuf, sizeof(pbuf));
    assert(strcmp(pbuf, "P:--.-  ") == 0);

    TelemetryState_Update(&state, 0, 1102);
    assert(strcmp(TelemetryState_Label(&state), "UNKNOWN") == 0);
    TelemetryState_Update(&state, 0xFF, 1103);
    assert(strcmp(TelemetryState_Label(&state), "UNKNOWN") == 0);
    assert(TelemetryState_Expire(&state, 2102)); /* unknown too: discard queued frames after a stall */

    TelemetryState_Reset(&state, UINT32_MAX - 500);
    TelemetryState_Update(&state, 0xA2, UINT32_MAX - 500);
    TelemetryState_UpdateTrim(&state, 0xB5, 15, -25, UINT32_MAX - 500);
    assert(!TelemetryState_Expire(&state, 499));
    TelemetryState_FormatZero(&state, 'P', pbuf, sizeof(pbuf));
    TelemetryState_FormatZero(&state, 'R', rbuf, sizeof(rbuf));
    assert(strcmp(pbuf, "P:+1.5  ") == 0 && strcmp(rbuf, "R:-2.5  ") == 0);
    assert(TelemetryState_Expire(&state, 500));
    TelemetryState_FormatZero(&state, 'P', pbuf, sizeof(pbuf));
    assert(strcmp(pbuf, "P:--.-  ") == 0);

    TelemetryState_Update(&state, 0xA2, 500);
    TelemetryState_UpdateTrim(&state, 0xB5, 5, 5, 500);
    TelemetryState_Reset(&state, 501); /* re-enter page */
    assert(strcmp(TelemetryState_Label(&state), "UNKNOWN") == 0);
    TelemetryState_FormatZero(&state, 'P', pbuf, sizeof(pbuf));
    assert(strcmp(pbuf, "P:--.-  ") == 0);

    /* Independent trim expiry clears trim without dropping a still-fresh lock state. */
    TelemetryState_Reset(&state, 1000);
    TelemetryState_Update(&state, 0xA1, 1000);
    TelemetryState_UpdateTrim(&state, 0xB5, 25, -35, 1000);
    TelemetryState_Update(&state, 0xA1, 1500);
    assert(!TelemetryState_Expire(&state, 2000));
    TelemetryState_FormatZero(&state, 'P', pbuf, sizeof(pbuf));
    assert(strcmp(pbuf, "P:+2.5  ") == 0);
    assert(!TelemetryState_Expire(&state, 2001));
    assert(strcmp(TelemetryState_Label(&state), "LOCK") == 0);
    TelemetryState_FormatZero(&state, 'P', pbuf, sizeof(pbuf));
    assert(strcmp(pbuf, "P:--.-  ") == 0);

    /* Short or null output buffer guard must fail safely without overflow. */
    {
        char tiny[4] = "abc";
        TelemetryState_FormatZero(&state, 'P', tiny, sizeof(tiny));
        assert(tiny[0] == '\0');
        TelemetryState_FormatZero(&state, 'P', NULL, 9);
    }
    return 0;
}
