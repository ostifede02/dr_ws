#!/usr/bin/env python3

import numpy as np

configuration = {}

configuration["paths"] = {
    "package_path": "/home/ostifede02/dr_ws/src/deltarobot_description",
    "gui_assets_path": "/home/ostifede02/dr_ws/assets/gui/",
}

configuration["physical"] = {
    "end_effector_radius": 40,
    # "link_pitch": 50,
    "end_effector_x_offset": 22.5
}

configuration["trajectory"] = {
    "max_acceleration": 600,
    "max_velocity": 300,

    "delta_s_high_resolution": 0.05,        # used for path length
    "mean_distance_between_set_points": 3,
    "min_distance_between_set_points": 5,

    "min_delta_time": 0.02,

    "pos_home": np.array([0, 0, -40])

}

configuration["inverse_geometry"] = {
    "collision_pair": (0, 1),
    "parameters": {
        "max_iterations": 300,
        "max_back_tracking_iterations": 30,
        "absolute_pos_threshold": 1e-3,         # absolute tolerance on position error
        "gradient_threshold": 1e-3,             # absolute tolerance on gradient's norm
        "beta": 0.1,                            # backtracking line search parameter
        "gamma": 1e-2,                          # line search convergence parameters
        "hessian_regu": 1e-2                    # Hessian regularization
    }
}





##########   TRAJECTORY ROUTINES   ##########
P2P_JOINT_TRAJECTORY        = "P2P_JOINT_TRAJECTORY"
P2P_DIRECT_TRAJECTORY       = "P2P_DIRECT_TRAJECTORY"
P2P_CONTINUOUS_TRAJECTORY   = "P2P_CONTINUOUS_TRAJECTORY"
PICK_TRAJECTORY             = "PICK_TRAJECTORY"
PLACE_TRAJECTORY            = "PLACE_TRAJECTORY"
HOMING                      = "HOMING"



##########   ROBOT STATES   ##########
ROBOT_STATE_RUN         = "run"
ROBOT_STATE_IDLE        = "idle"
ROBOT_STATE_STOP        = "stop"
ROBOT_STATE_REQUEST     = "request"
ROBOT_STATE_ERROR       = "error"


##########   ERRORS   ##########