configuration = {}

configuration["physical"] = {
    "pulley": {
        "n_teeth": 20,
        "module": 3
        },
    "stepper": {
        "n_steps": 800
        }
}

configuration["trajectory"] = {
    "max_acceleration": 80,
    "max_velocity": 120,
    "delta_s_high_resolution": 0.02
}

configuration["inverse_geometry"] = {
    "parameters": {
        "max_iterations": 300,
        "max_back_tracking_iterations": 30,
        "absolute_pos_threshold": 1e-3,         # absolute tolerance on position error
        "gradient_threshold": 1e-3,             # absolute tolerance on gradient's norm
        "beta": 0.1,                            # backtracking line search parameter
        "gamma": 1e-2,                          # line search convergence parameters
        "hessian_regu": 1e-2                    # Hessian regularization
    },
    "frame_ids": {
        "chain_1": 8,
        "chain_2": 14
    }

}