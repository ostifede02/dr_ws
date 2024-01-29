#ifndef STEPPER_H
#define STEPPER_H

#include "Arduino.h"
#include "ShiftRegister74HC595.h"

#include "board_pinout.h"


extern ShiftRegister74HC595<NUMBER_SHIFT_REG> sr;

void do_half_step(char PIN_STEPPER_X_STEP, int counter);
void set_direction(char PIN_STEPPER_X_STEP, float sign);

#endif // STEPPER_H