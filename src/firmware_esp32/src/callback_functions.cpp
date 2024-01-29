#include "callback_functions.h"


void subscription_callback(const void * msgin)
{
    // Cast received message to used type
    const micro_custom_messages__msg__QuaternionArray * msg = (const micro_custom_messages__msg__QuaternionArray *) msgin;
    
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
        delta_t_micros = msg[set_point_index].data->w;
        if(delta_t_micros < 0){         // break if end of msg
            break;
        }
        else if(delta_t_micros == 0){   // do not process message if delta_t is zero
            continue;
        }
        
        delta_q1 = msg[set_point_index].data->x;
        delta_q2 = msg[set_point_index].data->y;
        delta_q3 = msg[set_point_index].data->z;

        // Process message
        set_direction(PIN_STEPPER_1_DIR, delta_q1);
        set_direction(PIN_STEPPER_2_DIR, delta_q2);
        set_direction(PIN_STEPPER_3_DIR, delta_q3);

        stepper1_half_steps = 2*((delta_q1 / (PI * D_PULLEY))*STEPS_PER_REV);
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