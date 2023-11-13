def inverse_geometry_step_block():
    steps_1, steps_2, delta_t = 0

    '''
        INPUT:  robot, q1, q2, s_next, x_traveled, t_traveled, 
                collision_pairs, steps_per_rev, d_pulley

        OUTPUT: steps_1, steps_2, delta_t 
                    or 
                None if collision    
    
    '''

    # calculate next via point
        # pos_des: s -> pos_des

    # calculate inverse geometry
        # q1_next, q2_next: robot, q1, q2, pos_des -> q1_next, q2_next

    # check for collisions
        # true, false: robot, q1, q2, collision_pairs -> true, false

    # calculate equivalent number of steps
        # steps_1, steps_2: delta_q1, delta_q2, steps_per_rev, d_pulley -> steps_1, steps_2

    # calculate delta t
        # delta_t: x_traveled, delta_x, t_traveled -> delta_t



    return steps_1, steps_2, delta_t