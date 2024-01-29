#include "stepper.h"

ShiftRegister74HC595<NUMBER_SHIFT_REG> sr(I2S_DATA_PIN, I2S_CLOCK_PIN, I2S_LATCH_PIN);


void do_half_step(char PIN_STEPPER_X_STEP, int counter)
{
    if(counter%2 == 0){
        sr.set(PIN_STEPPER_X_STEP, HIGH);
    }
    else {
        sr.set(PIN_STEPPER_X_STEP, LOW);
    }
    return;
}


void set_direction(char PIN_STEPPER_X_STEP, float sign)
{
    if(sign < 0){
        sr.set(PIN_STEPPER_X_STEP, HIGH);
    }
    else {
        sr.set(PIN_STEPPER_X_STEP, LOW);
    }
    return;
}