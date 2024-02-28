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



    // rcl_publisher_t * task_ack_pub;

    // micro_custom_messages__msg__TaskAck task_ack_msg;
    // task_ack_msg.task_ack = true;
    // RCCHECK(rcl_publish(task_ack_pub, &task_ack_msg, NULL));
    return;
}
















    // while(true){
    //     // unpack message set point
    //     delta_t_micros = msg->set_points[set_point_index].delta_t;

    //     if(delta_t_micros < 0){         // break if end of msg (delta_t_micros -> -1)
    //         break;
    //     }
    //     else if(set_point_index >= MSG__SET_POINT_ARRAY__MAX_SIZE){       
    //         break;                      // break if index out of bound 
    //     }
    //     else if(delta_t_micros == 0){   // do not process message if delta_t is zero
    //         set_point_index += 1;       // process next set point
    //         continue;
    //     }
        
    //     delta_q1 = msg->set_points[set_point_index].delta_q1;
    //     delta_q2 = msg->set_points[set_point_index].delta_q2;
    //     delta_q3 = msg->set_points[set_point_index].delta_q3;

    //     // Process set point message
    //     set_direction(PIN_STEPPER_1_DIR, delta_q1);
    //     set_direction(PIN_STEPPER_2_DIR, delta_q2);
    //     set_direction(PIN_STEPPER_3_DIR, delta_q3);

    //     go_to_next_set_point(delta_q1, delta_q2, delta_q3, delta_t_micros);

    //     set_point_index += 1;   // process next set point
    // }