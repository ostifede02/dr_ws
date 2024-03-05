#include "motor_driver.h"

ShiftRegister74HC595<NUMBER_SHIFT_REG> sr(I2S_DATA_PIN, I2S_CLOCK_PIN, I2S_LATCH_PIN);


void MotorDriver::do_half_step(char PIN_STEPPER_X_STEP, int counter)
{
    if(counter%2 == 0){
        sr.set(PIN_STEPPER_X_STEP, HIGH);
    }
    else {
        sr.set(PIN_STEPPER_X_STEP, LOW);
    }
    return;
}

void MotorDriver::set_direction(char PIN_STEPPER_X_DIR, float sign)
{
    if(sign < 0){
        sr.set(PIN_STEPPER_X_DIR, HIGH);
    }
    else {
        sr.set(PIN_STEPPER_X_DIR, LOW);
    }
    return;
}



void MotorDriver::go_to_next_set_point(float delta_q1, float delta_q2, float delta_q3, long unsigned int delta_t_micros)
{
    // stepper 1
    unsigned int stepper1_steps;
    unsigned int stepper1_delay_micros;
    float stepper1_counter;

    delta_q1 = abs(delta_q1) + delta_q1_remainder;
    stepper1_steps = delta_q1 / q_min_displacement;
    delta_q1_remainder = delta_q1 - stepper1_steps*q_min_displacement;

    if (stepper1_steps != 0){   // avoid zero division
        stepper1_delay_micros = delta_t_micros / stepper1_steps;
    }
    stepper1_counter = 0;

    // stepper 2
    unsigned int stepper2_steps;
    unsigned int stepper2_delay_micros;
    float stepper2_counter;

    delta_q2 = abs(delta_q2) + delta_q2_remainder;
    stepper2_steps = delta_q2 / q_min_displacement;
    delta_q2_remainder = delta_q2 - stepper2_steps*q_min_displacement;

    if (stepper2_steps != 0){   // avoid zero division
        stepper2_delay_micros = delta_t_micros / stepper2_steps;
    }
    stepper2_counter = 0;


    // stepper 3
    unsigned int stepper3_steps;
    unsigned int stepper3_delay_micros;
    float stepper3_counter;

    delta_q3 = abs(delta_q3) + delta_q3_remainder;
    stepper3_steps = delta_q3 / q_min_displacement;
    delta_q3_remainder = delta_q3 - stepper3_steps*q_min_displacement;

    if (stepper3_steps != 0){   // avoid zero division
        stepper3_delay_micros = delta_t_micros / stepper3_steps;
    }
    stepper3_counter = 0;


    // control all three steppers simultaneously
    int time_dt = 0;
    unsigned int initial_time = micros();

    while ( (stepper1_counter < stepper1_steps) && 
            (stepper2_counter < stepper2_steps) && 
            (stepper3_counter < stepper3_steps))
    {
        time_dt = micros() - initial_time;

        // stepper 1
        if( (stepper1_counter < stepper1_steps) && 
            (time_dt >= stepper1_delay_micros*stepper1_counter))
        {
            do_half_step(PIN_STEPPER_1_STEP, stepper1_counter);
            stepper1_counter += 0.5;
        }

        // stepper 2
        else if((stepper2_counter < stepper2_steps) &&
                (time_dt >= stepper2_delay_micros*stepper2_counter))
        {
            do_half_step(PIN_STEPPER_2_STEP, stepper2_counter);
            stepper2_counter += 0.5;
        }

        // stepper 3
        else if((stepper3_counter < stepper3_steps) &&
                (time_dt >= stepper3_delay_micros*stepper3_counter))
        {
            do_half_step(PIN_STEPPER_3_STEP, stepper3_counter);
            stepper3_counter += 0.5;
        }
    }

    return;
}