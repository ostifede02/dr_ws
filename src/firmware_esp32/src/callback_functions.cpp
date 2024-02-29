#include "callback_functions.h"


void trajectory_task_callback(const void * msgin)
{
    // unpack message trajectory
    micro_custom_messages__msg__JointTrajectoryReducedArray * set_points_array_msg = 
        (micro_custom_messages__msg__JointTrajectoryReducedArray *) msgin;

    size_t n_set_points = set_points_array_msg->array_size;

    float delta_q1;
    float delta_time;

    delta_q1 = set_points_array_msg->set_points.data[0+1].q1 -
                    set_points_array_msg->set_points.data[0].q1;

    delta_time = set_points_array_msg->set_points.data[0+1].t_set_point -
                set_points_array_msg->set_points.data[0].t_set_point;

    set_direction(PIN_STEPPER_1_DIR, delta_q1);
    go_to_next_set_point(delta_q1, 0, 0, delta_time*1000000);


    // for(int set_point_index = 0; set_point_index < n_set_points-1; ++set_point_index){
    //     delta_q1 = set_points_array_msg->set_points.data[set_point_index+1].q1 -
    //                 set_points_array_msg->set_points.data[set_point_index].q1;

    //     delta_time = set_points_array_msg->set_points.data[set_point_index+1].t_set_point -
    //                 set_points_array_msg->set_points.data[set_point_index].t_set_point;

    //     go_to_next_set_point(delta_q1, 0, 0, delta_time*1000000);
    // }


    // publish ack
    micro_custom_messages__msg__TaskAck task_ack_msg;
    task_ack_msg.task_ack = true;
    RCCHECK(rcl_publish(&task_ack_pub, &task_ack_msg, NULL));

    return;
}