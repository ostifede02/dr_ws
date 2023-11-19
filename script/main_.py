import numpy as np
import random as rnd
import time

from delta_robot import DeltaRobot
import finite_state_machine as fsm
import configuration as conf


dr = DeltaRobot(viewer=False)

state = fsm.READ_CAM_STATE

while True:
    # *****   READ CAMERA   *****
    if state == fsm.READ_CAM_STATE:
        #
        # ...read camera position prediction and detect object type
        #

        pos_end = np.array([rnd.randint(-100, 100), 0, -280])
        t_total = rnd.randint(4,7)
        
        # based on object type select place position, either left or right
        if rnd.random() < 0.5:
            pos_place = np.array([-120, 0, -250])
        else:
            pos_place = np.array([120, 0, -250])

        trajectory_routine = fsm.PICK_TRAJECTORY_ROUTINE
        state = fsm.SET_TRAJECTORY_STATE


    # *****   SET TRAJECTORY   *****
    if state == fsm.SET_TRAJECTORY_STATE:
        if trajectory_routine == fsm.PICK_TRAJECTORY_ROUTINE:
            dr.set_trajectory_routine(trajectory_routine, pos_end, t_total)
        
        elif trajectory_routine == fsm.PLACE_TRAJECTORY_ROUTINE:
            dr.set_trajectory_routine(trajectory_routine, pos_end)
        
        elif trajectory_routine == fsm.PLACE_TRAJECTORY_ROUTINE:
            dr.set_trajectory_routine(trajectory_routine, pos_end)
        
        state = fsm.COMPUTE_NEXT_POS_STATE            

    
    # *****   COMPUTE NEXT POSITION   *****
    if state == fsm.COMPUTE_NEXT_POS_STATE:
        pos_next = dr.get_pos_next()
        if pos_next is None:
            state = fsm.ROUTINE_HANDLER_STATE
        
        q_next_continuos = dr.get_q_continuos(pos_next)


    # *****   SELECT TRAJECTORY ROUTINE   *****
    if state == fsm.ROUTINE_HANDLER_STATE:
        if trajectory_routine == fsm.PICK_TRAJECTORY_ROUTINE:
            trajectory_routine = fsm.PLACE_TRAJECTORY_ROUTINE
            pos_end = pos_place
            state = fsm.SET_TRAJECTORY_STATE
        
        elif trajectory_routine == fsm.PLACE_TRAJECTORY_ROUTINE:
            trajectory_routine = fsm.DIRECT_TRAJECTORY_ROUTINE
            pos_end = conf.configuration["trajectory"]["pos_neutral"]
            state = fsm.SET_TRAJECTORY_STATE
        
        elif trajectory_routine == fsm.DIRECT_TRAJECTORY_ROUTINE:
            trajectory_routine = fsm.PICK_TRAJECTORY_ROUTINE
            state = fsm.READ_CAM_STATE
        


