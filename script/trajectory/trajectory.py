'''
This class is intended to calculate the path and trajectory planning.

The key-methods for using this class are:
    + set_trajectory_routine()
        INPUT: 
            path_routine_type:
                                "pick"  -> for a pick routine path  (Note: this path is time constrained)
                                "place" -> for a place routine path
                                "quick" -> for a straight line quick return
            pos_start:      start position (current position of the end-effector)
            pos_end:        goal position
            t_total_input:  if trajectory is time constrained  
    + get_t_next()
'''


import numpy as np

from script import configuration as conf


class Trajectory:
    def __init__(self):
        # import variables
        self.max_velocity_default = conf.configuration["trajectory"]["max_velocity"]
        self.max_acceleration_default = conf.configuration["trajectory"]["max_acceleration"]


    def get_pos_bezier_poly(self, s):
        pos = pow(1-s, 3)*self.path_poly_points[0]
        pos += 3*pow(1-s, 2)*s*self.path_poly_points[1] 
        pos += 3*(1-s)*pow(s, 2)*self.path_poly_points[2] 
        pos += pow(s, 3)*self.path_poly_points[3]
        return pos


    def __get_path_poly_points(self, path_routine_type, pos_start, pos_end):

        if path_routine_type == conf.DIRECT_TRAJECTORY_ROUTINE:
            x1_offset = (pos_end[0] - pos_start[0])*0.333
            z1_offset = (pos_end[2] - pos_start[2])*0.333
            x2_offset = (pos_start[0] - pos_end[0])*0.333
            z2_offset = (pos_start[2] - pos_end[2])*0.333

        elif path_routine_type == conf.PICK_TRAJECTORY_ROUTINE:
            x2_offset = 0
            z2_offset = (pos_start[2] - pos_end[2])*0.35
            x1_offset = ((pos_end[0]+x2_offset)-pos_start[0])*0.5
            z1_offset = ((pos_end[2]+z2_offset)-pos_start[2])*0.5

        elif path_routine_type == conf.PLACE_TRAJECTORY_ROUTINE:
            x1_offset = 0
            z1_offset = min(abs(pos_start[0] - pos_end[0])*0.4, 80)
            z1_offset = max(abs(pos_start[0] - pos_end[0])*0.4, 30)
            x2_offset = 0
            z2_offset = min(abs(pos_start[0] - pos_end[0])*0.4, 80)
            z2_offset = max(abs(pos_start[0] - pos_end[0])*0.4, 30)
        else:
            return None
        
        P1 = np.array([pos_start[0]+x1_offset, 0, pos_start[2]+z1_offset])
        P2 = np.array([pos_end[0]+x2_offset, 0, pos_end[2]+z2_offset])

        path_poly_points = np.array([pos_start, P1, P2, pos_end])

        return path_poly_points
    

    def __get_path_length(self):
        x_total = 0
        delta_s = conf.configuration["trajectory"]["delta_s_high_resolution"]
        pos_current = self.get_pos_bezier_poly(0)

        # compute curve's length
        s_instance = np.linspace(0, 1-delta_s, int(1/delta_s))
        for s in s_instance:
            pos_next = self.get_pos_bezier_poly(s+delta_s)
            x_total += np.linalg.norm(pos_next-pos_current)
            pos_current = pos_next

        return x_total


    def __get_delta_s(self, x_total):
        # if the travel distance is to short -> go directly there
        if x_total <= 10:
            delta_s = 1
        elif x_total > 10 and x_total <= 20:
            max_delta_x = 2
            delta_s = round(max_delta_x / x_total, 3)
        elif x_total > 20 and x_total <= 100:
            max_delta_x = 5
            delta_s = round(max_delta_x / x_total, 3)
        else:
            max_delta_x = 10
            delta_s = round(max_delta_x / x_total, 3)

        return delta_s
    

    def __get_const_velocity(self, t_total, x_total, const_acceleration):
        # check if is physically possible
        delta = pow(t_total * const_acceleration, 2)-4*x_total*const_acceleration
        
        if delta < 0:
            return None
        
        velocity = (t_total * const_acceleration-np.sqrt(pow(t_total * const_acceleration, 2)-4*x_total*const_acceleration))/2
        return velocity


    def __get_time_scaling_flags(self, x_total, const_velocity, const_acceleration):
    
        x_acc_flag = pow(const_velocity,2) / (2*const_acceleration)
        
        # if there isn't a constant velocity profile
        if 2*x_acc_flag >= x_total:
            x_acc_flag = x_total * 0.5

        # time flags
        t_acc_flag = np.sqrt((2*x_acc_flag)/const_acceleration)
        t_total = np.sqrt((2*x_acc_flag)/const_acceleration) + ((x_total-2*x_acc_flag)/const_velocity) + np.sqrt((2*abs(x_acc_flag)/const_acceleration))

        return x_acc_flag, t_acc_flag, t_total


    
    #    path_routine_types:
    #        "pick":     pick object path
    #        "place":    place object path
    #        "quick":    straight line path
    def trajectory_generator(self, path_routine_type, pos_start, pos_end, t_total_input=-1):
        # the cubic bezier curve is a plynomial described by 4 points
        self.path_poly_points = self.__get_path_poly_points(path_routine_type, pos_start, pos_end)
        
        self.x_total = self.__get_path_length()

        self.delta_s = self.__get_delta_s(self.x_total)     # avoid via points too close to each other
        
        # if the trajectory is time constrained -> set new max velocity, else default max velocity
        if t_total_input > 0:
            self.const_acceleration = self.max_acceleration_default
            self.const_velocity = self.__get_const_velocity(t_total_input, self.x_total, self.const_acceleration)
            if self.const_velocity is None:
                print(f"ERROR! with an acceleration of {self.const_acceleration}, the path of length {self.x_total} millimeters cannot be reached in {t_total_input} seconds.")
                return None
        else:
            self.const_velocity = self.max_velocity_default
            self.const_acceleration = self.max_acceleration_default

        
        # time scaling profile flags
        self.x_acc_flag, self.t_acc_flag, self.t_total = self.__get_time_scaling_flags(self.x_total, self.const_velocity, self.const_acceleration)
        
        return True

    def get_t_next(self, x_next):
        # acceleration profile
        if x_next < self.x_acc_flag:
            t_next = np.sqrt((2*x_next)/self.const_acceleration)

        # constant velocity profile
        if x_next >= self.x_acc_flag and x_next < (self.x_total - self.x_acc_flag):
            t_next = self.t_acc_flag + ((x_next - self.x_acc_flag) / self.const_velocity)

        # deceleration profile
        if x_next >= (self.x_total - self.x_acc_flag):
            t_next = self.t_total - np.sqrt((2*abs(self.x_total - x_next) / self.const_acceleration))

        return t_next
    


    