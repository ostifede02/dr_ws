#include "error_log.h"

// Error handle loop
void error_loop(void)
{
    while (1)
    {
        for (int i = 0; i < 400; ++i)
        {
            do_half_step(PIN_STEPPER_1_STEP, i);
            delayMicroseconds(500);
        }

        delay(500);
    }
}