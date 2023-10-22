import numpy as np

'''
 __     __         _       _     _           
 \ \   / /_ _ _ __(_) __ _| |__ | | ___  ___ 
  \ \ / / _` | '__| |/ _` | '_ \| |/ _ \/ __|
   \ V / (_| | |  | | (_| | |_) | |  __/\__ \
    \_/ \__,_|_|  |_|\__,_|_.__/|_|\___||___/
                                             
'''

z_offset = 100
# start and end point
P0 = np.array([-400, 0, -280])
P3 = np.array([400, 0, -280])
# via points
P1 = np.array([P0[0], 0, P0[2]+z_offset])
P2 = np.array([P3[0], 0, P3[2]+z_offset])


# max acceleration and velocity
max_acc = 400        # [ mm / s2 ]
max_vel = 100      # [ mm / s ]
next_vel = 0
delta_t = 0

# states time scaling
STATE_ACC = "state const acc"
STATE_VEL = "state const vel"
STATE_DEC = "state const dec"
state = STATE_ACC   # initial state


# state change flags
# from acceleration to constant velocity OR deceleration
s_1 = 0




'''
  _____                 _   _                 
 |  ___|   _ _ __   ___| |_(_) ___  _ __  ___ 
 | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
 |  _|| |_| | | | | (__| |_| | (_) | | | \__ \
 |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
                                              
'''


# in: s -> [0, 1]   out: X
def bezier_curve(s):
    s = max(s, 0)
    s = min(s, 1)
    x_next = pow(1-s, 3)*P0 + 3*pow(1-s, 2)*s*P1 + 3*(1-s)*pow(s, 2)*P2 + pow(s, 3)*P3
    return x_next


def get_velocity_bezier_curve(s, delta_s, delta_t):  
    delta_x = np.linalg.norm(bezier_curve(s+delta_s) - bezier_curve(s))
    vel = delta_x / delta_t
    return vel

def get_roots(a, b, c, sol):
    if sol == 1:
        t1 = (-b + np.sqrt(abs(pow(b,2)-4*a*c)))/(2*a)
        return t1
    elif sol == 2:
        t2 = (-b - np.sqrt(abs(pow(b,2)-4*a*c)))/(2*a)
        return t2
    return -1

# in: t -> [0, T]   out: time_taken
def time_taken_interval(s, delta_s):
    global state, s_1, next_vel

    if round(s,3) > 1:
        state = STATE_ACC
        return None, None
     
    # acceleration
    if state == STATE_ACC:   
        x_current = bezier_curve(s)
        x_next = bezier_curve(s+delta_s)
        delta_x = np.linalg.norm(x_next - x_current)
        # x0 + v0*t + 0.5*a0*t^2 = x
        # 0.5*a0*delta_t^2 + v0*delta_t - delta_x = 0
        # a = 0.5*a0;   b = v0;  c = -delta_x
        delta_t = get_roots(0.5*max_acc, 0, -delta_x, sol=1)
        # v1 = v0 + a0*t
        prev_vel = next_vel
        next_vel = next_vel + max_acc * delta_t

        if s >= 0.5:
            state = STATE_DEC
            next_vel = prev_vel

        elif next_vel >= max_vel:            #get_velocity_bezier_curve(s, delta_s, delta_t)
            state = STATE_VEL
            next_vel = prev_vel
            s_1 = s
            

    # constant velocity
    if state == STATE_VEL:
        x_current = bezier_curve(s)
        x_next = bezier_curve(s+delta_s)

        delta_x = np.linalg.norm(x_next - x_current)
        delta_t = delta_x / next_vel

        if s >= (1 - s_1):
            print(f"s1={1-s_1}\ts={s}")
            state = STATE_DEC

    # deceleration

    if state == STATE_DEC:
        x_current = bezier_curve(s)
        x_next = bezier_curve(s+delta_s)
        delta_x = np.linalg.norm(x_next - x_current)     

        # x0 + v0*t + 0.5*a0*t^2 = x
        # -0.5*a0*delta_t^2 + v0*delta_t - delta_x = 0
        # a = -0.5*a0;   b = v0;  c = -delta_x
        delta_t = get_roots(-0.5*max_acc, 0, -delta_x, sol=2)
        # v1 = v0 + a0*t
        next_vel = next_vel - max_acc * delta_t
        print(f"next_vel={next_vel}")


    return delta_t, next_vel