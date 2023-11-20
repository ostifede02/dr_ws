import numpy as np
import random as rnd
import time

from delta_robot import DeltaRobot
import configuration as conf


dr = DeltaRobot(viewer=True)

# initialize neutral position
pos_end = conf.configuration["trajectory"]["pos_neutral"]
trajectory_routine = conf.DIRECT_TRAJECTORY_ROUTINE
state = conf.SET_TRAJECTORY_STATE


max_cycle_counter = 200
elapsed_time = np.empty(max_cycle_counter+1)
stepper_steps = np.empty(max_cycle_counter+1)
cycle_counter = 0

while True:
    start_time = time.time()

    # *****   READ CAMERA   *****
    if state == conf.READ_CAM_STATE:
        #
        # ...read camera position prediction and detect object type
        #

        pos_end = np.array([rnd.randint(-100, 100), 0, -280])
        t_total = 1.2 + 0.5*rnd.random()
        
        # based on object type select place position, either left or right
        if rnd.random() < 0.5:
            pos_place = np.array([-180, 0, -250])
        else:
            pos_place = np.array([180, 0, -250])

        trajectory_routine = conf.PICK_TRAJECTORY_ROUTINE
        state = conf.SET_TRAJECTORY_STATE


    # *****   SET TRAJECTORY   *****
    if state == conf.SET_TRAJECTORY_STATE:
        if trajectory_routine == conf.PICK_TRAJECTORY_ROUTINE:
            dr.set_trajectory_routine(trajectory_routine, pos_end, t_total)
        
        elif trajectory_routine == conf.PLACE_TRAJECTORY_ROUTINE:
            dr.set_trajectory_routine(trajectory_routine, pos_end)
        
        elif trajectory_routine == conf.DIRECT_TRAJECTORY_ROUTINE:
            dr.set_trajectory_routine(trajectory_routine, pos_end)
        
        state = conf.COMPUTE_NEXT_POS_STATE            

    
    # *****   COMPUTE NEXT POSITION   *****
    if state == conf.COMPUTE_NEXT_POS_STATE:
        pos_next = dr.get_pos_next(simulation=False)
        
        if pos_next is None:                               # end of path
            state = conf.ROUTINE_HANDLER_STATE
            continue
        
        q_next_continuos = dr.get_q_next_continuos(pos_next)
        
        if dr.check_collisions(q_next_continuos):
            break

        stepper_1_steps, stepper_2_steps = dr.get_number_of_steps()
        delta_t = dr.get_delta_t()

        state = conf.DISPLAY_STATE


    # *****   SELECT TRAJECTORY ROUTINE   *****
    if state == conf.ROUTINE_HANDLER_STATE:
        if trajectory_routine == conf.PICK_TRAJECTORY_ROUTINE:
            trajectory_routine = conf.PLACE_TRAJECTORY_ROUTINE
            pos_end = pos_place
            time.sleep(0.2)
            state = conf.SET_TRAJECTORY_STATE
        
        elif trajectory_routine == conf.PLACE_TRAJECTORY_ROUTINE:
            trajectory_routine = conf.DIRECT_TRAJECTORY_ROUTINE
            pos_end = conf.configuration["trajectory"]["pos_neutral"]
            time.sleep(0.4)
            state = conf.SET_TRAJECTORY_STATE
        
        elif trajectory_routine == conf.DIRECT_TRAJECTORY_ROUTINE:
            trajectory_routine = conf.PICK_TRAJECTORY_ROUTINE
            time.sleep(0.2)
            state = conf.READ_CAM_STATE

    
    # *****   DISPLAY GEPETTO VIEWER   *****
    if state ==  conf.DISPLAY_STATE:
        # dr.display(q_next_continuos)
        # time.sleep(delta_t)

        state = conf.COMPUTE_NEXT_POS_STATE


    # end_time = time.time()
    # delta_t_program = end_time-start_time
    # elapsed_time[cycle_counter] = delta_t_program

    if stepper_1_steps == 0:
        print(f"stepper_1 will not move!")
    else:
        # print(f"stepper_1_steps: \t{np.mean(stepper_1_steps)}\n")
        # print(f"millisec_per_step: \t{np.mean(millisec_per_step)*1000} milliseconds")
        t_per_step = delta_t / (2*stepper_1_steps)
        stepper_steps[cycle_counter] = abs(t_per_step)
        cycle_counter += 1


    if cycle_counter > max_cycle_counter:
        break

# print(f"mean time per cycle: \t{np.mean(elapsed_time)*1000} milliseconds")
# print(f"max time per cycle: \t{max(elapsed_time)*1000} milliseconds")
# print(f"min time per cycle: \t{min(elapsed_time)*1000} milliseconds")

print(f"mean time per step: \t{round(np.mean(stepper_steps)*1e3, 3)} milli-seconds")
print(f"max time per step: \t{round(max(stepper_steps)*1e3, 3)} milli-seconds")
print(f"min time per step: \t{round(min(stepper_steps)*1e3, 3)} milli-seconds")


