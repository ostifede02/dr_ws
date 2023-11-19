import numpy as np
import random as rnd
import time

from delta_robot import DeltaRobot
import configuration as conf


dr = DeltaRobot(viewer=False)

state = conf.READ_CAM_STATE

while True:
    # *****   READ CAMERA   *****
    if state == conf.READ_CAM_STATE:
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

        trajectory_routine = conf.PICK_TRAJECTORY_ROUTINE
        state = conf.SET_TRAJECTORY_STATE


    # *****   SET TRAJECTORY   *****
    if state == conf.SET_TRAJECTORY_STATE:
        if trajectory_routine == conf.PICK_TRAJECTORY_ROUTINE:
            dr.set_trajectory_routine(trajectory_routine, pos_end, t_total)
        
        elif trajectory_routine == conf.PLACE_TRAJECTORY_ROUTINE:
            dr.set_trajectory_routine(trajectory_routine, pos_end)
        
        elif trajectory_routine == conf.PLACE_TRAJECTORY_ROUTINE:
            dr.set_trajectory_routine(trajectory_routine, pos_end)
        
        state = conf.COMPUTE_NEXT_POS_STATE            

    
    # *****   COMPUTE NEXT POSITION   *****
    if state == conf.COMPUTE_NEXT_POS_STATE:
        pos_next = dr.get_pos_next()
        if pos_next is None:
            state = conf.ROUTINE_HANDLER_STATE
        
        q_next_continuos = dr.get_q_next_continuos(pos_next)
        q_next_discrete = dr.get_q_discrete(q_next_continuos)


    # *****   SELECT TRAJECTORY ROUTINE   *****
    if state == conf.ROUTINE_HANDLER_STATE:
        if trajectory_routine == conf.PICK_TRAJECTORY_ROUTINE:
            trajectory_routine = conf.PLACE_TRAJECTORY_ROUTINE
            pos_end = pos_place
            state = conf.SET_TRAJECTORY_STATE
        
        elif trajectory_routine == conf.PLACE_TRAJECTORY_ROUTINE:
            trajectory_routine = conf.DIRECT_TRAJECTORY_ROUTINE
            pos_end = conf.configuration["trajectory"]["pos_neutral"]
            state = conf.SET_TRAJECTORY_STATE
        
        elif trajectory_routine == conf.DIRECT_TRAJECTORY_ROUTINE:
            trajectory_routine = conf.PICK_TRAJECTORY_ROUTINE
            state = conf.READ_CAM_STATE
        


