# 2 axis delta robot

## setting up the environment
### installing the pinocchio library
Follow the tutorial at the link [here](https://stack-of-tasks.github.io/pinocchio/download.html)
### installing gepetto viewer
If you are using Ubuntu 20.04 run:
~~~
sudo apt install robotpkg-py38-qt5-gepetto-viewer-corba
~~~

If you are using Ubuntu 22.04 run:
~~~
sudo apt install robotpkg-py310-qt5-gepetto-viewer-corba
~~~

## project
### inverse geomety of a closed chain robot
Since the URDF file and the ik algorithm can only solve for open chain systems, we are going to break down the problem, solving the ik for each pair of links, and let them converge to the same desired position. The ik of the third link pair (green right) is not computed, since it is equal to the other one.

### URDF file structure
#### joints
The following contraints are applied to the joints:
+ q[2] = q[1], in order to keep the end effector parallel to the ground
+ q[5] = q[4], in order to keep the two rods parallel

![plot](/script/img/joint_diagram.png)

#### open chains
The robot model is described by three open kinematic chains.
+ chain 1 (red): carriage 1 --> rod 1 --> end effector
+ chain 2 (green): carriage 2 --> rod 2
+ chain 3 (blue): carriage 2 --> rod 3

![plot](/script/img/chain_diagram.png)

### script
#### IK_solver class
The script implements a IK_solver class (see IK.solver.py). The class has two instances, one for each link chain. The **solve_GN** method uses the Gauss-Newton method in order to calculate the inverse geometry of the robot.
+ input:   (current joint position, desired end-effector position)
+ output:(new joint position)
In addition all the configuration parameters are already built-in in the class.

#### Collision detection
After computing the inverse geometry, the collision between the rods and the rails only is computed. If a collision is detect, the program stops.


### path and trajectory planning
#### path routines
Cubic Bezier curves are used for the path planning. During a pick and place routine, we can classify different paths as follows:
+ pick routine (red): start straight and get to the object from the top
+ place routine (green): start and place the object moving along a vertical path
+ return to neutral (blue): is a fast straight line from point to point (using a bezier curve too)

![plot](/script/trajectory/graphs/path_routine_subplots.png)


#### bang bang velocity profile
We can differentiate between optimal-time and time-constrained trajectories. For example, the pick routine is constrained by time as we need to synchronize the end effector with the moving object on the conveyor belt. Conversely, the place and return routines operate at maximum speed and acceleration. The time scaling position profile below illustrates the function representing position over time, reflecting the non-isometrical behavior of the Bezier curve.

+ Ist segment: constant acceleration
+ IInd segment: constant velocity
+ IIIrd segment: constant deceleration

![plot](/script/trajectory/graphs/position_time_scaling_profile_bezier.png)

### gepetto viewer

https://github.com/ostifede02/2dr/assets/113505533/8275950d-85f3-4202-8fd4-2156c668f8a0
