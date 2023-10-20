import numpy as np


z_offset = 100
# start and end point
P0 = np.array([-200, 0, -280])
P3 = np.array([200, 0, -280])
# via points
P1 = np.array([P0[0], 0, P0[2]+z_offset])
P2 = np.array([P3[0], 0, P3[2]+z_offset])


# max acceleration and velocity
max_acc = 2
max_vel = 8

# time increment in seconds
delta_t = 0.05

# overall time taken
T = 0

# states time scaling
STATE_ACC = 1
STATE_VEL = 2
STATE_DEC = 3
state = STATE_ACC   # initial state


# state change flags
# from acceleration to constant velocity OR deceleration
s_1 = 0
t_1 = 0
# from constant velocity to deceleration
s_2 = 0
t_2 = 0






# in: s -> [0, 1]   out: X
def bezier_curve(s):
    x_next = pow(1-s, 3)*P0 + 3*pow(1-s, 2)*s*P1 + 3*(1-s)*pow(s, 2)*P2 + pow(s, 3)*P3
    return x_next

# calculate the distance between two consecutive points divided by the time step
def current_velocity(t):
    # s_current = (max_acc / 2) * pow(t, 2)
    # s_next = (max_acc / 2) * pow((t+delta_t), 2)
    
    s_current = t
    s_next = t+delta_t
    
    vel = np.linalg.norm((bezier_curve(s_current) - bezier_curve(s_next)) / delta_t)
    return vel

# in: t -> [0, T]   out: s -> [0, 1]
def time_scaling(t):
    global state, t_1, s_1, t_2, s_2, T

    if state is None:
        state = STATE_ACC
        t_1, s_1, t_2, s_2, T = 0
        return None

    # acceleration
    if state == STATE_ACC:
        s = (max_acc / 2) * pow(t, 2)

        if s > 0.5:
            state = STATE_DEC
            T = 2*t
            t_1 = t_2 = t
            s_1 = s_2 = s
        elif current_velocity(t) >= max_vel:
            state = STATE_VEL
            t_1 = t
            s_1 = s

    # constant velocity
    if state == STATE_VEL:
        s = max_vel * (t - t_1) + s_1

        if s >= (1 - s_1):
            state = STATE_DEC
            T = t_1 + t
            t_2 = t
            s_2 = s

    # deceleration
    if state == STATE_DEC:
        s = s_1 - abs((max_acc / 2) * pow((t - (t_2 + t_1)), 2)) + s_2

        if t > T:
            state = None
    
    return s
