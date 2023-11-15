import numpy as np

from ..trajectory_planning.trajectory_functions import bezier_curve, time_optimal_bang_bang_profile
from ..collision.collision_detection import is_collision

def inverse_geometry_step_block(robot, s, q1, q2, pos_current, x_current, t_current, ik_solver_1, ik_solver_2, q1_reminder, q2_reminder, trj_config, robot_config):
    '''
        INPUT:  
            variant:
                q1, q2,
                s_next, 
                x_current, t_current
            invariant:
                robot, ik_solver_1, ik_solver_2,
                (collision_pairs),
                steps_per_rev, d_pulley

        OUTPUT: steps_1, steps_2, 
                delta_t, delta_x

                or None if collision
    '''


    # calculate next via point
        # pos_des: s -> pos_des
    s_next = s + trj_config["delta_s"]
    pos_des = bezier_curve(s_next, trj_config["via_points"])

    # calculate inverse geometry
        # q1_next, q2_next: robot, q1, q2, pos_des -> q1_next, q2_next
    q1_next = ik_solver_1.solve_GN(q1, pos_des)
    q2_next = ik_solver_2.solve_GN(q2, pos_des)

    # check for collisions
        # true, false: robot, q1, q2, collision_pairs -> true, false
    # applying joint constraints
    q1_next[2] = q1_next[1]         # keeps the ee parallel to ground
    q2_next[5] = q2_next[4]         # keeps rod_3 parallel to rod_2
    q = np.concatenate((q1_next[0:3], q2_next[3:6]))

    # collision pairs: 0 -> rod1-rails and 1 -> rod2-rails
    if is_collision(robot, q, (0, 1)):
        return None

    # calculate equivalent number of steps
        # steps_1, steps_2: delta_q1, delta_q2, steps_per_rev, d_pulley -> steps_1, steps_2
    # calculating the number of steps to do (further to send to the motorcontroller)
    c_pulley = robot_config["pulley"]["n_teeth"] * robot_config["pulley"]["module"]
    min_displacement = c_pulley / robot_config["stepper"]["n_steps"]       # minimum carriage displacement

    # stepper 1
    delta_q_1 = q1_next[0] - q1
    stepper_1_steps = (delta_q_1 + q1_reminder) // min_displacement     # number of steps to do
    q1_reminder = (delta_q_1 + q1_reminder) % min_displacement          # the decimal part of steps

    # stepper 2
    delta_q_2 = q2_next[3] - q2
    stepper_2_steps = (delta_q_2 + q2_reminder) // min_displacement     # number of steps to do
    q2_reminder = (delta_q_2 + q2_reminder) % min_displacement          # the decimal part of steps


    # calculate delta t
        # delta_t: x_current, delta_x, t_current -> delta_t
    delta_x = np.linalg.norm(pos_des-pos_current)
    x_current += delta_x
    t_next = time_optimal_bang_bang_profile(x_current, 
                                            trj_config["x_acc_flag"],
                                            trj_config["x_total"],
                                            trj_config["t_acc_flag"],
                                            trj_config["t_total"],
                                            trj_config["vel"],
                                            trj_config["acc"],
                                            )
    
    delta_t = t_next - t_current
    t_current = t_next

    return stepper_1_steps, stepper_2_steps, delta_t