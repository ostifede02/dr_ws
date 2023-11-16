'''
This class manage all add-ons of the delta robot.
These are the main characteristics:
    + import robot model
    + init viewer
    + init ig solvers
    + init trajectory
    
    + init micro com
    + init gui
    + init camera
    
    + configure path
    
    + get delta t
    + get t current
    + get delta x
    + get x current
    
    + get pos next
    + get q
    + get q in steps

    ++ keep track of x traveled
    ++ keep track of t traveled

    + manage errors
    + manage exceptions

'''


class DeltaRobot:
    def __init__(self):
        return