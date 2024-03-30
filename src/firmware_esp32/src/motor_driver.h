#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "Arduino.h"
#include "ShiftRegister74HC595.h"

#include "board_pinout.h"

// extern ShiftRegister74HC595<NUMBER_SHIFT_REG> sr;

class MotorDriver
{
private:
  float delta_q1_remainder_mm = 0;
  float delta_q2_remainder_mm = 0;
  float delta_q3_remainder_mm = 0;
  float ticks_per_mm = 161.9;  // after manual calibration

public:
  MotorDriver();
  void go_to_next_set_point(float delta_q1, float delta_q2, float delta_q3, long unsigned int delta_t_micros);
  void toggle_step_tick(char PIN_STEPPER_X_STEP, int counter);
  void set_direction(char PIN_STEPPER_X_STEP, float sign);
  void homing(void);
};

extern MotorDriver mdriver;

#endif  // MOTOR_DRIVER_H