#include "motor_driver.h"

ShiftRegister74HC595<NUMBER_SHIFT_REG> sr(I2S_DATA_PIN, I2S_CLOCK_PIN, I2S_LATCH_PIN);

MotorDriver::MotorDriver(void)
{
    // initialize stepper pins to LOW
    sr.set(PIN_STEPPER_1_STEP, LOW);
    sr.set(PIN_STEPPER_2_STEP, LOW);
    sr.set(PIN_STEPPER_3_STEP, LOW);

    return;
}

void MotorDriver::toggle_step_tick(char PIN_STEPPER_X_STEP, int half_ticks_total_counter)
{
    if (half_ticks_total_counter % 2 == 0) {
        sr.set(PIN_STEPPER_X_STEP, HIGH);
    }
    else {
        sr.set(PIN_STEPPER_X_STEP, LOW);
    }
    return;
}

void MotorDriver::set_direction(char PIN_STEPPER_X_DIR, float sign)
{
    if (sign < 0) {
        sr.set(PIN_STEPPER_X_DIR, HIGH);
    }
    else {
        sr.set(PIN_STEPPER_X_DIR, LOW);
    }
    return;
}

void MotorDriver::go_to_next_via_point(float delta_q1_mm, float delta_q2_mm, float delta_q3_mm, long int delta_t_micros)
{
    /**********************************************************
     *                        STEPPER 1
     **********************************************************/
    int stepper1_ticks_total;
    int stepper1_tick_delay_micros = 0;
    int stepper1_ticks_counter = 0;

    delta_q1_mm += delta_q1_remainder_mm;
    stepper1_ticks_total = 2 * (delta_q1_mm) / MM_PER_STEP;
    
    // take lowest even number
    if (stepper1_ticks_total % 2 != 0){
        stepper1_ticks_total -= 1;
    }

    if (stepper1_ticks_total != 0) {  // avoid zero division
        stepper1_tick_delay_micros = delta_t_micros / stepper1_ticks_total;
        delta_q1_remainder_mm = delta_q1_mm - (0.5 * float(stepper1_ticks_total) * MM_PER_STEP);
    }

    stepper1_ticks_total = abs(stepper1_ticks_total) - 1;
    stepper1_tick_delay_micros = abs(stepper1_tick_delay_micros);


    /**********************************************************
     *                        STEPPER 2
     **********************************************************/
    int stepper2_ticks_total;
    long int stepper2_tick_delay_micros = 0;
    int stepper2_ticks_counter = 0;

    delta_q2_mm += delta_q2_remainder_mm;
    stepper2_ticks_total = 2 * delta_q2_mm / MM_PER_STEP;
    
    // take lowest even number
    if (stepper2_ticks_total % 2 != 0){
        stepper2_ticks_total -= 1;
    }

    if (stepper2_ticks_total != 0) {  // avoid zero division
        stepper2_tick_delay_micros = delta_t_micros / stepper2_ticks_total;
        delta_q2_remainder_mm = delta_q2_mm - (0.5 * float(stepper2_ticks_total) * MM_PER_STEP);
    }

    stepper2_ticks_total = abs(stepper2_ticks_total) - 1;
    stepper2_tick_delay_micros = abs(stepper2_tick_delay_micros);


    /**********************************************************
     *                        STEPPER 3
     **********************************************************/
    int stepper3_ticks_total;
    long int stepper3_tick_delay_micros = 0;
    int stepper3_ticks_counter = 0;

    delta_q3_mm += delta_q3_remainder_mm;
    stepper3_ticks_total = 2 * delta_q3_mm / MM_PER_STEP;
    
    // take lowest even number
    if (stepper3_ticks_total % 2 != 0){
        stepper3_ticks_total -= 1;
    }

    if (stepper3_ticks_total != 0) {  // avoid zero division
        stepper3_tick_delay_micros = delta_t_micros / stepper3_ticks_total;
        delta_q3_remainder_mm = delta_q3_mm - (0.5 * float(stepper3_ticks_total) * MM_PER_STEP);
    }

    stepper3_ticks_total = abs(stepper3_ticks_total) - 1;
    stepper3_tick_delay_micros = abs(stepper3_tick_delay_micros);


    /**********************************************************
     *                      CONTROL STEPPERS
     **********************************************************/
    int time_dt = 0;
    int initial_time = micros();

    while ((stepper1_ticks_counter < stepper1_ticks_total) && (stepper2_ticks_counter < stepper2_ticks_total) &&
    (stepper3_ticks_counter < stepper3_ticks_total)) {
        time_dt = micros() - initial_time;

        // stepper 1
        if ((stepper1_ticks_counter < stepper1_ticks_total) &&
            (time_dt >= stepper1_tick_delay_micros * (stepper1_ticks_counter + 1))) {
            toggle_step_tick(PIN_STEPPER_1_STEP, stepper1_ticks_counter);
            stepper1_ticks_counter += 1;
        }

        // stepper 2
        else if ((stepper2_ticks_counter < stepper2_ticks_total) &&
                 (time_dt >= stepper2_tick_delay_micros * (stepper2_ticks_counter + 1))) {
            toggle_step_tick(PIN_STEPPER_2_STEP, stepper2_ticks_counter);
            stepper2_ticks_counter += 1;
        }

        // stepper 3
        else if ((stepper3_ticks_counter < stepper3_ticks_total) &&
                 (time_dt >= stepper3_tick_delay_micros * (stepper3_ticks_counter + 1))) {
            toggle_step_tick(PIN_STEPPER_3_STEP, stepper3_ticks_counter);
            stepper3_ticks_counter += 1;
        }
    }

    return;
}

void MotorDriver::homing(void)
{
    bool is_homing_1 = true;
    bool is_homing_2 = true;
    bool is_homing_3 = true;
    int half_step_index = 0;

    set_direction(PIN_STEPPER_1_DIR, float(1.0));
    set_direction(PIN_STEPPER_2_DIR, float(1.0));
    set_direction(PIN_STEPPER_3_DIR, float(1.0));

    while (true) {
        // if all limit switches are pressed
        if (!is_homing_1 && !is_homing_2 && !is_homing_3) {
            break;
        }

        // check if limit switch is pressed
        if (analogRead(LIMIT_SWITCH_1_PIN) > 0) {
            is_homing_1 = false;
        }
        if (analogRead(LIMIT_SWITCH_2_PIN) > 0) {
            is_homing_2 = false;
        }
        if (analogRead(LIMIT_SWITCH_3_PIN) > 0) {
            is_homing_3 = false;
        }

        // do half step
        if (is_homing_1) {
            toggle_step_tick(PIN_STEPPER_1_STEP, half_step_index);
        }
        if (is_homing_2) {
            toggle_step_tick(PIN_STEPPER_2_STEP, half_step_index);
        }
        if (is_homing_3) {
            toggle_step_tick(PIN_STEPPER_3_STEP, half_step_index);
        }

        // wait
        delayMicroseconds(700);
        half_step_index += 1;
    }

    delay(500);

    // go back by default offset
    set_direction(PIN_STEPPER_1_DIR, float(-1.0));
    set_direction(PIN_STEPPER_2_DIR, float(-1.0));
    set_direction(PIN_STEPPER_3_DIR, float(-1.0));

    for (int i = 0; i < 300; ++i) {
        toggle_step_tick(PIN_STEPPER_1_STEP, i);
        toggle_step_tick(PIN_STEPPER_2_STEP, i);
        toggle_step_tick(PIN_STEPPER_3_STEP, i);
        delayMicroseconds(700);
    }

    return;
}