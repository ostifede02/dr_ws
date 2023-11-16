'''
This class manage all add-ons of the delta robot.
These are the main characteristics:
    + import robot model    ok
    + init viewer           ok
    + init ig solvers       ok
    + init trajectory       ok
    
    + init micro com
    + init gui
    + init camera
    
    + configure path
    
    + get delta t
    + get t current
    + get delta x
    + get x current

    + collisions
    
    + get pos next
    + get q
    + get q in steps

    ++ keep track of x traveled
    ++ keep track of t traveled

    + manage errors
    + manage exceptions

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
        
        self.trj = Trajectory()
        return
   
    
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
    

    def initViewer(self):
        self.robot.initViewer()
        return
    

