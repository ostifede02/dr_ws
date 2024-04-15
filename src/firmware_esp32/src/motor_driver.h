#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "Arduino.h"
#include "ShiftRegister74HC595.h"
#include "board_pinout.h"

#define STEPPER1_IDX 0
#define STEPPER2_IDX 1
#define STEPPER3_IDX 2


class MotorDriver
{
    private:
        float MM_PER_STEP = 0.025;  // after manual calibration

        float delta_q_remainder_mm[3]   = {0.0, 0.0, 0.0};
        int PIN_STEPPERS_STEP[3]        = {PIN_STEPPER_1_STEP, PIN_STEPPER_2_STEP, PIN_STEPPER_3_STEP};
        int PIN_STEPPERS_STEP__state[3] = {LOW, LOW, LOW};

        int get_total_stepper_ticks(float delta_q, int stepper_index);
        int get_delay_step_ticks(int stepper_ticks_total, int delta_t_micros);

    public:
        MotorDriver();

        void move_steppers_async(   float delta_q1, 
                                    float delta_q2, 
                                    float delta_q3, 
                                    int delta_t_micros);
        void homing(void);

        void toggle_stepper_step_tick(int stepper_index);
        void set_direction(char PIN_STEPPER_X_STEP, float sign);
};

extern MotorDriver mdriver;

#endif  // MOTOR_DRIVER_H