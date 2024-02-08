import numpy as np

configuration = {}

configuration["paths"] = {
    "package_path": "/home/ostifede02/dr_ws/src/deltarobot_description",
    "gui_assets_path": "/home/ostifede02/dr_ws/assets/gui/",
}

configuration["physical"] = {
    "pulley": {
        "n_teeth": 20,
        "module": 3
    },
    "stepper": {
        "n_steps": 1600
    },
    "ee_radius": 40,
    "link_pitch": 50
}

configuration["trajectory"] = {
    "max_acceleration": 400,
    "max_velocity": 800,

    "delta_s_high_resolution": 0.05,        # used for path length
    "mean_distance_between_set_points": 8,
    "min_distance_between_set_points": 20,

    "joint_space_linspace_resolution": 200,

    "pos_home": np.array([0, 0, -150]),
    "pos_neutral": np.array([0, 0, -200]),

    "end_effector_x_offset": 22.5
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
PICK_TRAJECTORY_ROUTINE = "pick"
PLACE_TRAJECTORY_ROUTINE = "place"
TASK_SPACE_DIRECT_TRAJECTORY_ROUTINE = "p2p_task_space"
TASK_SPACE_SIMPLE_P2P_TRAJECTORY_ROUTINE = "simple_p2p_task_space"
JOINT_SPACE_DIRECT_TRAJECTORY_ROUTINE = "p2p_joint_space"

##########   ERRORS   ##########