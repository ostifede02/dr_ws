#include "callback_functions.h"

MotorDriver mdriver;

void trajectory_task_callback(const void* msgin)
{
  // unpack message trajectory
  micro_custom_messages__msg__JointTrajectoryReducedArray* set_points_array_msg =
      (micro_custom_messages__msg__JointTrajectoryReducedArray*)msgin;

  size_t n_set_points = set_points_array_msg->array_size;

  float delta_q1;
  float delta_q2;
  float delta_q3;
  float delta_time;
  unsigned long int delta_time_micros;

  for (int set_point_index = 0; set_point_index < n_set_points - 1; ++set_point_index)
  {
    // stepper 1
    delta_q1 = set_points_array_msg->set_points.data[set_point_index + 1].q1 -
               set_points_array_msg->set_points.data[set_point_index].q1;

    // stepper 2
    delta_q2 = set_points_array_msg->set_points.data[set_point_index + 1].q2 -
               set_points_array_msg->set_points.data[set_point_index].q2;

    // stepper 3
    delta_q3 = set_points_array_msg->set_points.data[set_point_index + 1].q3 -
               set_points_array_msg->set_points.data[set_point_index].q3;

    // delta time
    delta_time = set_points_array_msg->set_points.data[set_point_index + 1].t_set_point -
                 set_points_array_msg->set_points.data[set_point_index].t_set_point;
    delta_time_micros = delta_time * 1000000;

    mdriver.set_direction(PIN_STEPPER_1_DIR, delta_q1);
    mdriver.set_direction(PIN_STEPPER_2_DIR, delta_q2);
    mdriver.set_direction(PIN_STEPPER_3_DIR, delta_q3);

    mdriver.go_to_next_set_point(delta_q1, delta_q2, delta_q3, delta_time_micros);
  }

  // publish ack
  micro_custom_messages__msg__TaskAck task_ack_msg;
  task_ack_msg.task_ack = true;
  RCCHECK(rcl_publish(&task_ack_pub, &task_ack_msg, NULL));

  return;
}

void homing_task_callback(const void* msgin)
{
  std_msgs__msg__Bool* task_homing_msg = (std_msgs__msg__Bool*)msgin;

  if (task_homing_msg->data)
  {
    mdriver.homing();
  }
  else
  {
    // publish nak
    micro_custom_messages__msg__TaskAck task_ack_msg;
    task_ack_msg.task_ack = false;
    RCCHECK(rcl_publish(&task_ack_pub, &task_ack_msg, NULL));
  }

  // publish ack
  micro_custom_messages__msg__TaskAck task_ack_msg;
  task_ack_msg.task_ack = true;
  RCCHECK(rcl_publish(&task_ack_pub, &task_ack_msg, NULL));
}