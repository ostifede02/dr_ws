#include "callback_functions.h"


void subscription_callback(const void * msgin)
{
    const micro_custom_messages__msg__SetPointArray * msg = (const micro_custom_messages__msg__SetPointArray *) msgin;

    // int loops = abs(msg->set_points[2].delta_q1);

    // for(int j=0; j<loops; j++){
	// 	for (int i = 0; i < 1600; ++i)
    //     {
    //         do_half_step(PIN_STEPPER_1_STEP, i);
    //         delayMicroseconds(200);
    //     }
	// 	delay(1000);
	// }


    float delta_q1;
    float delta_q2;
    float delta_q3;
    float delta_t_micros;
    int set_point_index = 0;

    int stepper1_half_steps;
    int stepper1_delay_micros;
    int stepper1_counter;

    int time_dt;
    int initial_time;
    

    while(true){
        // unpack message set point
        delta_t_micros = msg->set_points[set_point_index].delta_t;

        if(delta_t_micros < 0){         // break if end of msg
            break;
        }
        else if(delta_t_micros == 0){   // do not process message if delta_t is zero
            set_point_index += 1;
            continue;
        }
        
        delta_q1 = msg->set_points[set_point_index].delta_q1;
        delta_q2 = msg->set_points[set_point_index].delta_q2;
        delta_q3 = msg->set_points[set_point_index].delta_q3;

        set_point_index += 1;

        // Process set point message
        set_direction(PIN_STEPPER_1_DIR, delta_q1);
        set_direction(PIN_STEPPER_2_DIR, delta_q2);
        set_direction(PIN_STEPPER_3_DIR, delta_q3);

        stepper1_half_steps = 2*((abs(delta_q1) / (PI * D_PULLEY))*STEPS_PER_REV);
        stepper1_delay_micros = delta_t_micros / stepper1_half_steps;
        stepper1_counter = 0;

        time_dt = 0;
        initial_time = micros();

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
    }

    return;
}