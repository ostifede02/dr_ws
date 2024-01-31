#include "callback_functions.h"


void trajectory_task_callback(const void * msgin)
{
    // unpack message trajectory
    const micro_custom_messages__msg__SetPointArray * msg = (const micro_custom_messages__msg__SetPointArray *) msgin;

    float delta_q1;
    float delta_q2;
    float delta_q3;
    float delta_t_micros;
    int set_point_index = 0;


    while(true){
        // unpack message set point
        delta_t_micros = msg->set_points[set_point_index].delta_t;

        if(delta_t_micros < 0){         // break if end of msg (delta_t_micros -> -1)
            break;
        }
        else if(set_point_index >= MSG__SET_POINT_ARRAY__MAX_SIZE){       
            break;                      // break if index out of bound 
        }
        else if(delta_t_micros == 0){   // do not process message if delta_t is zero
            set_point_index += 1;       // process next set point
            continue;
        }
        
        delta_q1 = msg->set_points[set_point_index].delta_q1;
        delta_q2 = msg->set_points[set_point_index].delta_q2;
        delta_q3 = msg->set_points[set_point_index].delta_q3;

        // Process set point message
        set_direction(PIN_STEPPER_1_DIR, delta_q1);
        set_direction(PIN_STEPPER_2_DIR, delta_q2);
        set_direction(PIN_STEPPER_3_DIR, delta_q3);

        go_to_next_set_point(delta_q1, delta_q2, delta_q3, delta_t_micros);

        set_point_index += 1;   // process next set point
    }

    return;
}