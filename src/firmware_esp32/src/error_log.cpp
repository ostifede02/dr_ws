#include "error_log.h"

// Error handle loop
void error_loop(void)
{
  while (1)
  {
    for (int i = 0; i < 2; ++i)
    {
      mdriver.toggle_step_tick(PIN_STEPPER_1_STEP, i);
      delayMicroseconds(400);
    }

    delay(1000);
  }
}