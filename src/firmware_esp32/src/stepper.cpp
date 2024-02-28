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


void set_direction(char PIN_STEPPER_X_DIR, float sign)
{
    if(sign < 0){
        sr.set(PIN_STEPPER_X_DIR, HIGH);
    }
    else {
        sr.set(PIN_STEPPER_X_DIR, LOW);
    }
    return;
}


void go_to_next_set_point(float delta_q1, float delta_q2, float delta_q3, float delta_t_micros)
{
    // stepper 1:
    int stepper1_half_steps = ((abs(delta_q1) / (PI * D_PULLEY))*STEPS_PER_REV);;
    int stepper1_delay_micros = delta_t_micros / stepper1_half_steps;
    int stepper1_counter = 0;

    // // stepper 2:
    // int stepper2_half_steps = 4*((abs(delta_q2) / (PI * D_PULLEY))*STEPS_PER_REV);;
    // int stepper2_delay_micros = delta_t_micros / stepper2_half_steps;
    // int stepper2_counter = 0;

    // // stepper 3:
    // int stepper3_half_steps = 4*((abs(delta_q3) / (PI * D_PULLEY))*STEPS_PER_REV);
    // int stepper3_delay_micros = delta_t_micros / stepper3_half_steps;
    // int stepper3_counter = 0;    

    int time_dt = 0;
    int initial_time = micros();

    // ******** TO DO: control all steppers simulteinously ***********
    while (true)
    {
        time_dt = micros() - initial_time;

        if(time_dt >= stepper1_delay_micros*stepper1_counter){
            do_half_step(PIN_STEPPER_1_STEP, stepper1_counter);
            stepper1_counter += 1;
        }

        if(stepper1_counter > stepper1_half_steps){
            break;
        }
    }

    return;
}