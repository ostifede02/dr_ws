'''
This class manage all add-ons of the delta robot.
These are the main characteristics:
    +++ init
        + import robot model    ok
        + init viewer           ok
        + init ig solvers       ok
        + init trajectory       ok
        + init micro com
        + init gui
        + init camera
    
    +++ trajectory
        + configure path: based on routine_type, start and goal position
            --> reset x and t traveled
        + get_pos_next: --> keep track of x traveled
        + get_t_next:   --> keep track of t traveled

    +++ inverse geometry
        + compute inverse geometry: pos_next -> q_next  --> keep track of q current
        + check collisions
    
    +++ stepper motors interface
        + convert delta_q to steps
        + manage discretization error
        

    + manage errors
    + manage exceptions


    +++ finite state machine flow
    states:             events:
        routine             pick
                            place
                            quick

        cmd_camera          camera
        
        cmd_user            new_pos
                            break


        if get_camera_cmd:
            save pos_end
            state = set_path

        if external_cmd:
            if stop_cmd
                state = idle
    
        if set_path:
            + set_path -> input: pos_end, path_routine_type
            state = run
        
        if run:
            + get_pos_next     -> update: x_current, s
                if s_next > 1:              # arrived to goal position
                    state = routine_handler
                    continue    jump back to top
            + get_q_continuos
                if collision:
                    state = idle
                    continue    jump back to top
            + get_delta_q_discrete   -> q_remainder
            + get_delta_t      -> t_current
            state = send_cmd

        if send_cmd:
            send to micro:
                delta_t
                delta_q_discrete

        if idle:
            do nothing
        
        if quick_neutral return
            pos_end = pos_neutral
            routine_type = quick
            state = set_path

        if routine_handler:
            if routine == pick
                routine = place
                pos_end = pos_place_X
                state = set_path
            
            if routine == place
                routine = quick
                pos_end = pos_neutral
            
            if routine == quick
                routine = pick
                state = get_cmd
        
        if cmd_handler:
            if cmd == camera_cmd
                pass
            if cmd == user_input_cmd
                pass


'''

from pinocchio import RobotWrapper
import pinocchio as pin

from os.path import dirname, join, abspath

from kinematics.inverse_geometry import InverseGeometry
from trajectory.trajectory import Trajectory
import configuration as conf


class DeltaRobot:

    def __init__(self, viewer = False):
        self.initRobot()
        
        if viewer:
            self.initViewer()

        frame_id_1 = conf.configuration["inverse_geometry"]["frame_ids"]["chain_1"]
        self.ig_solver_1 = InverseGeometry(self.robot, frame_id_1)

        frame_id_2 = conf.configuration["inverse_geometry"]["frame_ids"]["chain_2"]
        self.ig_solver_2 = InverseGeometry(self.robot, frame_id_2)
        
        self.trajectory = Trajectory()
        return
   
    # ******   MODEL   ******
    def initRobot(self):
        model_dir = join(dirname(str(abspath(__file__))),"delta_robot_description")
        mesh_dir = join(model_dir,"meshes")

        # Load the URDF model
        urdf_filename = "delta_robot.urdf"
        urdf_dir = join(model_dir,"urdf")
        urdf_file_path = join(urdf_dir, urdf_filename)

        # Initialize the model
        model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_file_path, mesh_dir, geometry_types=[pin.GeometryType.COLLISION,pin.GeometryType.VISUAL])
        self.robot = RobotWrapper(model, collision_model, visual_model)
        self.robot.collision_model.addAllCollisionPairs()
        self.robot.collision_data = pin.GeometryData(self.robot.collision_model)
        return
    
    
    # ******   VIEWER   ******
    def initViewer(self):
        self.robot.initViewer()
        return
    
    def display(self, q):
        self.robot.viz.display(q)
        return
    

    # ******   TRAJECTORY   ******
    def set_trajectory_routine(self, path_routine_type, pos_start, pos_end, t_total_input=-1):
        self.trajectory.set_trajectory_routine(path_routine_type, pos_start, pos_end, t_total_input)

    

