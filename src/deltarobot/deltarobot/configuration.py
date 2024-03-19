#!/usr/bin/env python3

import numpy as np

configuration = {}

configuration["paths"] = {
    "package_path": "/home/ostifede02/dr_ws/src/deltarobot_description",
    "gui_assets_path": "/home/ostifede02/dr_ws/assets/gui/",
}

configuration["physical"] = {
    "end_effector_radius": 42
}

configuration["trajectory"] = {
    "max_acceleration": 600,
    "max_velocity": 300,

    "delta_s_high_resolution": 0.05,        # used for path length
    "mean_distance_between_set_points": 3,
    "min_distance_between_set_points": 5,

    "min_delta_time": 0.02,

    "pos_home": np.array([0, 0, -40.4])
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