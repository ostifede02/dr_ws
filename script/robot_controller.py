import numpy as np
import random as rnd
import time

from script.delta_robot import DeltaRobot
import script.configuration as conf


reduced_steps = False        # if reduced_steps == True ,the set-points are closer to each other for a smoother visualization                   
viewer = True                # if viewr == True, can view the robot in gepetto viewer

def main():
    dr = DeltaRobot(viewer)

    # initialize neutral position
    pos_end = conf.configuration["trajectory"]["pos_neutral"]
    trajectory_routine = conf.DIRECT_TRAJECTORY_ROUTINE
    state = conf.TRAJECTORY_PLANNER_STATE

    while True:

        # *****   READ CAMERA   *****
        if state == conf.READ_CAM_STATE:
            #
            # ...read camera position prediction and detect object type
            #

            pos_end = np.array([rnd.randint(-100, 100), 0, -260])
            pos_end[0] -= dr.end_effector_x_offset
            t_total = 1.5 + 0.2*rnd.random()
            
            # based on object type select place position, either left or right
            if rnd.random() < 0.5:
                pos_place = np.array([-190, 0, -280])
            else:
                pos_place = np.array([190, 0, -280])

            trajectory_routine = conf.PICK_TRAJECTORY_ROUTINE
            state = conf.TRAJECTORY_PLANNER_STATE


        # *****   SET TRAJECTORY   *****
        if state == conf.TRAJECTORY_PLANNER_STATE:
            if trajectory_routine == conf.PICK_TRAJECTORY_ROUTINE:
                error = dr.trajectory_planner(trajectory_routine, pos_end, t_total)
                if error is None:
                    break
            
            elif trajectory_routine == conf.PLACE_TRAJECTORY_ROUTINE:
                dr.trajectory_planner(trajectory_routine, pos_end)
            
            elif trajectory_routine == conf.DIRECT_TRAJECTORY_ROUTINE:
                dr.trajectory_planner(trajectory_routine, pos_end)
            
            state = conf.COMPUTE_NEXT_POS_STATE            

        
        # *****   COMPUTE NEXT VIA-POINT POSITION   *****
        if state == conf.COMPUTE_NEXT_POS_STATE:
            # goto next set (via) point
            pos_next = dr.get_pos_next(reduced_steps)
            
            if pos_next is None:                               # end of path
                state = conf.TASK_PLANNER_STATE
                continue
            
            q_next = dr.get_q_next_continuos(pos_next)
            
            if dr.check_collisions(q_next):
                print(f"ERROR! Collision detected")
                break

            stepper_1_steps, stepper_2_steps = dr.get_number_of_steps()
            delta_t = dr.get_delta_t()

            # state = conf.SEND_TO_MICRO_STATE
            state = conf.DISPLAY_STATE


        # *****   TASK PLANNER   *****
        if state == conf.TASK_PLANNER_STATE:
            if trajectory_routine == conf.PICK_TRAJECTORY_ROUTINE:
                trajectory_routine = conf.PLACE_TRAJECTORY_ROUTINE
                pos_end = pos_place
                # time.sleep(0.2)     # simulate grab object
                state = conf.TRAJECTORY_PLANNER_STATE
            
            elif trajectory_routine == conf.PLACE_TRAJECTORY_ROUTINE:
                trajectory_routine = conf.DIRECT_TRAJECTORY_ROUTINE
                pos_end = conf.configuration["trajectory"]["pos_neutral"]
                # time.sleep(0.4)     # simulate place object
                state = conf.TRAJECTORY_PLANNER_STATE
            
            elif trajectory_routine == conf.DIRECT_TRAJECTORY_ROUTINE:
                trajectory_routine = conf.PICK_TRAJECTORY_ROUTINE
                # time.sleep(0.2)     # simulate wait cam for new cmd
                state = conf.READ_CAM_STATE

        
        # *****   DISPLAY GEPETTO VIEWER   *****
        if state ==  conf.DISPLAY_STATE:
            if viewer:
                dr.display(q_next)
                time.sleep(delta_t)

            state = conf.COMPUTE_NEXT_POS_STATE



if __name__ == "__main__":
    main()