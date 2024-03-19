#include "motor_driver.h"

ShiftRegister74HC595<NUMBER_SHIFT_REG> sr(I2S_DATA_PIN, I2S_CLOCK_PIN, I2S_LATCH_PIN);


void MotorDriver::do_half_step(char PIN_STEPPER_X_STEP, int half_step_counter)
{
    if(half_step_counter%2 == 0){
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
    unsigned int stepper1_half_steps;
    unsigned int stepper1_delay_micros;
    unsigned int stepper1_counter = 0;

    delta_q1 = abs(delta_q1) + delta_q1_remainder;
    stepper1_half_steps = delta_q1 / q_min_displacement;
    delta_q1_remainder = delta_q1 - (stepper1_half_steps*q_min_displacement);

    if (stepper1_half_steps != 0){   // avoid zero division
        stepper1_delay_micros = delta_t_micros / stepper1_half_steps;
    }

    // stepper 2
    unsigned int stepper2_half_steps;
    unsigned int stepper2_delay_micros;
    unsigned int stepper2_counter = 0;

    delta_q2 = abs(delta_q2) + delta_q2_remainder;
    stepper2_half_steps = delta_q2 / q_min_displacement;
    delta_q2_remainder = delta_q2 - (stepper2_half_steps*q_min_displacement);

    if (stepper2_half_steps != 0){   // avoid zero division
        stepper2_delay_micros = delta_t_micros / stepper2_half_steps;
    }


    // stepper 3
    unsigned int stepper3_half_steps;
    unsigned int stepper3_delay_micros;
    unsigned int stepper3_counter = 0;

    delta_q3 = abs(delta_q3) + delta_q3_remainder;
    stepper3_half_steps = delta_q3 / q_min_displacement;
    delta_q3_remainder = delta_q3 - (stepper3_half_steps*q_min_displacement);

    if (stepper3_half_steps != 0){   // avoid zero division
        stepper3_delay_micros = delta_t_micros / stepper3_half_steps;
    }


    // control all three steppers simultaneously
    unsigned int time_dt = 0;
    unsigned int initial_time = micros();

    while ( (stepper1_counter < stepper1_half_steps) && 
            (stepper2_counter < stepper2_half_steps) && 
            (stepper3_counter < stepper3_half_steps))
    {
        time_dt = micros() - initial_time;

        // stepper 1
        if( (stepper1_counter < stepper1_half_steps) && 
            (time_dt >= stepper1_delay_micros*stepper1_counter))
        {
            do_half_step(PIN_STEPPER_1_STEP, stepper1_counter);
            stepper1_counter += 1;
        }

        // stepper 2
        else if((stepper2_counter < stepper2_half_steps) &&
                (time_dt >= stepper2_delay_micros*stepper2_counter))
        {
            do_half_step(PIN_STEPPER_2_STEP, stepper2_counter);
            stepper2_counter += 1;
        }

        // stepper 3
        else if((stepper3_counter < stepper3_half_steps) &&
                (time_dt >= stepper3_delay_micros*stepper3_counter))
        {
            do_half_step(PIN_STEPPER_3_STEP, stepper3_counter);
            stepper3_counter += 1;
        }
    }

    return;
}

void MotorDriver::homing(void)
{
    bool is_homing_1 = true;
    // bool is_homing_2 = true;
    // bool is_homing_3 = true;
    unsigned int half_step_index = 0;

    set_direction(PIN_STEPPER_1_DIR, float(1.0));
    // set_direction(PIN_STEPPER_2_DIR, float(1.0));
    // set_direction(PIN_STEPPER_3_DIR, float(1.0));

    while(true){
        // if all limit switches are pressed
        // if (!is_homing_1 && !is_homing_2 && !is_homing_3){
        //     break;
        // }
        if (!is_homing_1){
            break;
        }

        // check if limit switch is pressed
        if (analogRead(LIMIT_SWITCH_1_PIN) == 0){
            is_homing_1 = false;
        }
        // if (analogRead(LIMIT_SWITCH_2_PIN) == 0){
        //     is_homing_2 = false;
        // }
        // if (analogRead(LIMIT_SWITCH_3_PIN) == 0){
        //     is_homing_3 = false;
        // }
        
        // do half step
        if (is_homing_1){
            do_half_step(PIN_STEPPER_1_STEP, half_step_index);
        }
        // if (is_homing_2){
        //     do_half_step(PIN_STEPPER_2_STEP, half_step_index);
        // }
        // if (is_homing_3){
        //     do_half_step(PIN_STEPPER_3_STEP, half_step_index);
        // }

        // wait
        delayMicroseconds(500);
        half_step_index += 1;
    }

    delay(500);

    // go back by default offset
    set_direction(PIN_STEPPER_1_DIR, float(-1.0));

    for(int i=0; i<300; ++i){
        do_half_step(PIN_STEPPER_1_STEP, i);
        // do_half_step(PIN_STEPPER_2_STEP, i);
        // do_half_step(PIN_STEPPER_3_STEP, i);
        delayMicroseconds(700);
    }

    return;
}