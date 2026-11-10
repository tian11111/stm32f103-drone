#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "telemetry_state.h"

int main(void)
{
    TelemetryState state = {0};
    TelemetryState_Reset(&state, 100);
    assert(strcmp(TelemetryState_Label(&state), "UNKNOWN") == 0);
    TelemetryState_Update(&state, 0xA2, 100);
    assert(strcmp(TelemetryState_Label(&state), "UNLOCK") == 0);
    assert(!TelemetryState_Expire(&state, 1100));
    assert(TelemetryState_Expire(&state, 1101));
    assert(strcmp(TelemetryState_Label(&state), "UNKNOWN") == 0);
    TelemetryState_Update(&state, 0xA1, 1101);
    assert(strcmp(TelemetryState_Label(&state), "LOCK") == 0);
    TelemetryState_Update(&state, 0, 1102);
    assert(strcmp(TelemetryState_Label(&state), "UNKNOWN") == 0);
    TelemetryState_Update(&state, 0xFF, 1103);
    assert(strcmp(TelemetryState_Label(&state), "UNKNOWN") == 0);
    assert(TelemetryState_Expire(&state, 2102)); /* unknown too: discard queued frames after a stall */
    TelemetryState_Reset(&state, UINT32_MAX - 500);
    TelemetryState_Update(&state, 0xA2, UINT32_MAX - 500);
    assert(!TelemetryState_Expire(&state, 499));
    assert(TelemetryState_Expire(&state, 500));
    TelemetryState_Update(&state, 0xA2, 500);
    TelemetryState_Reset(&state, 501); /* re-enter page */
    assert(strcmp(TelemetryState_Label(&state), "UNKNOWN") == 0);
    return 0;
}
