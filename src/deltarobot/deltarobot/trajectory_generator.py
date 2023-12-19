import rclpy
from rclpy.node import Node

import numpy as np
import time

from deltarobot_interfaces.msg import TrajectoryTask
from deltarobot_interfaces.msg import SetPointsVector
from deltarobot_interfaces.msg import SetPoint

from deltarobot import configuration as conf



class TrajectoryGenerator(Node):

    def __init__(self):
        super().__init__('trajectory_generator_node')

        self.set_points_vector_pub = self.create_publisher(
            SetPointsVector,
            'set_points_vector',
            10)
        
        self.trajectory_task_sub = self.create_subscription(
            TrajectoryTask,
            'trajectory_task',
            self.trajectory_generator_callback,
            10)
        self.trajectory_task_sub

        self.max_velocity_default = conf.configuration["trajectory"]["max_velocity"]
        self.max_acceleration_default = conf.configuration["trajectory"]["max_acceleration"]
        return
    

    def trajectory_generator_callback(self, trajectory_task_msg):
        time_start = time.time()
        
        ## unpack the message
        pos_start = np.array([trajectory_task_msg.pos_start.x, trajectory_task_msg.pos_start.y, trajectory_task_msg.pos_start.z])
        pos_end = np.array([trajectory_task_msg.pos_end.x, trajectory_task_msg.pos_end.y, trajectory_task_msg.pos_end.z])
        t_total_input = trajectory_task_msg.task_time
        path_routine_type = trajectory_task_msg.task_type.data

        # the cubic bezier curve is a plynomial described by 4 points
        self.path_poly_points = self.__get_path_poly_points(pos_start, pos_end, path_routine_type)
        x_total = self.__get_path_length()

        n_set_points = self.__get_number_set_points(x_total)     # avoid via points too close to each other
        
        # if the trajectory is time constrained (t > 0) -> set new max velocity, else default max velocity
        if t_total_input > 0:
            self.const_acceleration = self.max_acceleration_default
            self.const_velocity = self.__get_const_velocity(t_total_input, x_total)
            if self.const_velocity is None:
                self.get_logger().error(f"With an acceleration of {self.const_acceleration}, the path of length {x_total} millimeters cannot be reached in {t_total_input} seconds.")
                return None
        else:
            self.const_velocity = self.max_velocity_default
            self.const_acceleration = self.max_acceleration_default

        
        # time scaling profile flags
        self.x_acc_flag, self.t_acc_flag, t_total = self.__get_time_scaling_flags(x_total)
        
        # creating the set points vector
        set_points_vector_msg = SetPointsVector()
        set_points_vector_msg.set_points = []

        set_point_prev = self.__get_pos_bezier_poly(0)
        x_travelled = 0
        
        s_instance = np.linspace(0, 1, n_set_points)
        for s in s_instance:
            set_point_msg = SetPoint()

            set_point = self.__get_pos_bezier_poly(s)
            x_travelled = np.linalg.norm(set_point - set_point_prev)
            set_point_prev = set_point

            if s == 1:
                set_point_msg.t = self.__get_t_next(x_total, x_total, t_total)      # avoid x_travelled approximations errors 
            else:
                set_point_msg.t = self.__get_t_next(x_travelled, x_total, t_total)

            set_point_msg.x = set_point[0]
            set_point_msg.z = set_point[2]
            
            set_points_vector_msg.set_points.append(set_point_msg)

        self.set_points_vector_pub.publish(set_points_vector_msg)

        time_stop = time.time()
        self.get_logger().info(f"elapsed time: {(time_stop-time_start)*1e3} [milliseconds]")
        return
    

    def __get_path_poly_points(self, pos_start, pos_end, path_routine_type):

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
        pos_current = self.__get_pos_bezier_poly(0)

        # compute curve's length
        s_instance = np.linspace(0, 1-delta_s, int(1/delta_s))
        for s in s_instance:
            pos_next = self.__get_pos_bezier_poly(s+delta_s)
            x_total += np.linalg.norm(pos_next-pos_current)
            pos_current = pos_next

        return x_total
    

    def __get_pos_bezier_poly(self, s):
        pos = np.empty(3)

        pos = pow(1-s, 3)*self.path_poly_points[0]
        pos += 3*pow(1-s, 2)*s*self.path_poly_points[1] 
        pos += 3*(1-s)*pow(s, 2)*self.path_poly_points[2] 
        pos += pow(s, 3)*self.path_poly_points[3]
        return pos
    

    def __get_number_set_points(self, x_total):
        # if the travel distance is to short -> go directly there
        if x_total <= 25:
            n_set_points = 2
        else:
            n_set_points = int(x_total/10)

        return n_set_points


    def __get_const_velocity(self, t_total, x_total):
        # check if is physically possible
        delta = pow(t_total * self.const_acceleration, 2)-4*x_total*self.const_acceleration
        
        if delta < 0:
            return None
        
        velocity = (t_total * self.const_acceleration-np.sqrt(pow(t_total * self.const_acceleration, 2)-4*x_total*self.const_acceleration))/2
        return velocity
    

    def __get_time_scaling_flags(self, x_total):
    
        x_acc_flag = pow(self.const_velocity,2) / (2*self.const_acceleration)
        
        # if there isn't a constant velocity profile
        if 2*x_acc_flag >= x_total:
            x_acc_flag = x_total * 0.5

        # time flags
        t_acc_flag = np.sqrt((2*x_acc_flag)/self.const_acceleration)
        t_total = np.sqrt((2*x_acc_flag)/self.const_acceleration)
        t_total += ((x_total-2*x_acc_flag)/self.const_velocity) 
        t_total += np.sqrt((2*abs(x_acc_flag)/self.const_acceleration))

        return x_acc_flag, t_acc_flag, t_total
    

    def __get_t_next(self, x_travelled, x_total, t_total):
        # acceleration profile
        if x_travelled < self.x_acc_flag:
            t_next = np.sqrt((2*x_travelled)/self.const_acceleration)

        # constant velocity profile
        if x_travelled >= self.x_acc_flag and x_travelled < (x_total - self.x_acc_flag):
            t_next = self.t_acc_flag + ((x_travelled - self.x_acc_flag) / self.const_velocity)

        # deceleration profile
        if x_travelled >= (x_total - self.x_acc_flag):
            t_next = t_total - np.sqrt((2*abs(x_total - x_travelled) / self.const_acceleration))

        return t_next    




def main(args=None):
    rclpy.init(args=args)

    tg_node = TrajectoryGenerator()

    rclpy.spin(tg_node)

    tg_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()