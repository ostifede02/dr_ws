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
  if (half_ticks_total_counter % 2 == 0)
  {
    sr.set(PIN_STEPPER_X_STEP, LOW);
  }
  else
  {
    sr.set(PIN_STEPPER_X_STEP, HIGH);
  }
  return;
}

void MotorDriver::set_direction(char PIN_STEPPER_X_DIR, float sign)
{
  if (sign < 0)
  {
    sr.set(PIN_STEPPER_X_DIR, HIGH);
  }
  else
  {
    sr.set(PIN_STEPPER_X_DIR, LOW);
  }
  return;
}

void MotorDriver::go_to_next_set_point(float delta_q1_mm, float delta_q2_mm, float delta_q3_mm,
                                       long unsigned int delta_t_micros)
{
  // stepper 1
  unsigned int stepper1_ticks_total;            // ONE tick is either pin HIGH or LOW
  unsigned int stepper1_tick_delay_micros = 0;  // time period between each tick
  unsigned int stepper1_ticks_counter = 1;

  delta_q1_mm = abs(delta_q1_mm) + delta_q1_remainder_mm;
  stepper1_ticks_total = int(delta_q1_mm * ticks_per_mm);  //  [mm / (mm/tick)] -> ticks

  if (stepper1_ticks_total != 0)
  {  // avoid zero division
    stepper1_tick_delay_micros = delta_t_micros / stepper1_ticks_total;
    delta_q1_remainder_mm = delta_q1_mm - (float(stepper1_ticks_total) / ticks_per_mm);
  }

  // stepper 2
  unsigned int stepper2_ticks_total;
  unsigned int stepper2_tick_delay_micros;
  unsigned int stepper2_ticks_counter = 1;

  delta_q2_mm = abs(delta_q2_mm) + delta_q2_remainder_mm;
  stepper2_ticks_total = int(delta_q2_mm * ticks_per_mm);  //  [mm / (mm/tick)] -> ticks

  if (stepper2_ticks_total != 0)
  {  // avoid zero division
    stepper2_tick_delay_micros = delta_t_micros / stepper2_ticks_total;
    delta_q2_remainder_mm = delta_q2_mm - (float(stepper2_ticks_total) / ticks_per_mm);
  }

  // stepper 3
  unsigned int stepper3_ticks_total;
  unsigned int stepper3_tick_delay_micros;
  unsigned int stepper3_ticks_counter = 1;

  delta_q3_mm = abs(delta_q3_mm) + delta_q3_remainder_mm;
  stepper3_ticks_total = int(delta_q3_mm * ticks_per_mm);  //  [mm / (mm/tick)] -> ticks

  if (stepper3_ticks_total != 0)
  {  // avoid zero division
    stepper3_tick_delay_micros = delta_t_micros / stepper3_ticks_total;
    delta_q3_remainder_mm = delta_q3_mm - (float(stepper3_ticks_total) / ticks_per_mm);
  }

  // control all three steppers simultaneously
  unsigned int time_dt = 0;
  unsigned int initial_time = micros();

  while ((stepper1_ticks_counter <= stepper1_ticks_total) && (stepper2_ticks_counter <= stepper2_ticks_total) &&
         (stepper3_ticks_counter <= stepper3_ticks_total))
  {
    time_dt = micros() - initial_time;

    // stepper 1
    if ((stepper1_ticks_counter <= stepper1_ticks_total) &&
        (time_dt >= stepper1_tick_delay_micros * stepper1_ticks_counter))
    {
      toggle_step_tick(PIN_STEPPER_1_STEP, stepper1_ticks_counter);
      stepper1_ticks_counter += 1;
    }

    // stepper 2
    else if ((stepper2_ticks_counter <= stepper2_ticks_total) &&
             (time_dt >= stepper2_tick_delay_micros * stepper2_ticks_counter))
    {
      toggle_step_tick(PIN_STEPPER_2_STEP, stepper2_ticks_counter);
      stepper2_ticks_counter += 1;
    }

    // stepper 3
    else if ((stepper3_ticks_counter <= stepper3_ticks_total) &&
             (time_dt >= stepper3_tick_delay_micros * stepper3_ticks_counter))
    {
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
  unsigned int half_step_index = 0;

  set_direction(PIN_STEPPER_1_DIR, float(1.0));
  set_direction(PIN_STEPPER_2_DIR, float(1.0));
  set_direction(PIN_STEPPER_3_DIR, float(1.0));

  while (true)
  {
    // if all limit switches are pressed
    if (!is_homing_1 && !is_homing_2 && !is_homing_3)
    {
      break;
    }

    // check if limit switch is pressed
    if (analogRead(LIMIT_SWITCH_1_PIN) == 0)
    {
      is_homing_1 = false;
    }
    if (analogRead(LIMIT_SWITCH_2_PIN) == 0)
    {
      is_homing_2 = false;
    }
    if (analogRead(LIMIT_SWITCH_3_PIN) == 0)
    {
      is_homing_3 = false;
    }

    // do half step
    if (is_homing_1)
    {
      toggle_step_tick(PIN_STEPPER_1_STEP, half_step_index);
    }
    if (is_homing_2)
    {
      toggle_step_tick(PIN_STEPPER_2_STEP, half_step_index);
    }
    if (is_homing_3)
    {
      toggle_step_tick(PIN_STEPPER_3_STEP, half_step_index);
    }

    // wait
    delayMicroseconds(500);
    half_step_index += 1;
  }

  delay(500);

  // go back by default offset
  set_direction(PIN_STEPPER_1_DIR, float(-1.0));

  for (int i = 0; i < 300; ++i)
  {
    toggle_step_tick(PIN_STEPPER_1_STEP, i);
    toggle_step_tick(PIN_STEPPER_2_STEP, i);
    toggle_step_tick(PIN_STEPPER_3_STEP, i);
    delayMicroseconds(700);
  }

  return;
}