#ifndef CALLBACK_FUNCTIONS_H
#define CALLBACK_FUNCTIONS_H

// #include "stepper.h"
#include "motor_driver.h"
#include "configuration.h"
#include "custom_messages.h"
#include "board_pinout.h"
#include "error_log.h"
#include "rcl_publishers.h"


void trajectory_task_callback(const void * msgin);
void homing_task_callback(const void * msgin);

#endif  //CALLBACK_FUNCTIONS_H