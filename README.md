# 2 axis delta robot

## setting up the environment
### installing the pinocchio library
Follow the tutorial at the link [here](https://stack-of-tasks.github.io/pinocchio/download.html)
### installing gepetto viewer
If you are using Ubuntu 20.04 run:
~~~
sudo apt install robotpkg-py38-pinocchio robotpkg-py38-example-robot-data robotpkg-urdfdom robotpkg-py38-qt5-gepetto-viewer-corba robotpkg-py38-quadprog robotpkg-py38-tsid
~~~

If you are using Ubuntu 22.04 run:
~~~
sudo apt install robotpkg-py310-pinocchio robotpkg-py310-example-robot-data robotpkg-urdfdom robotpkg-py310-qt5-gepetto-viewer-corba robotpkg-py310-quadprog robotpkg-py310-tsid
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


### trajectory planning
#### Bezier curves
Bezier curves are used for the path planning. They are very straight forward to implement and compute, however they have a main issue. Even though the input is isometric, the euclidean distance between each point of the output curve is not fixed. This affects the time scaling consistency.

#### time scaling function of a bezier curve
###### velocity time scaling function
As we can see the velocity profile of a bezier curve differs from the expected one.

![plot](/script/trajectory%20planning/graphs/veloity_time_scaling_profile_bezier.png)

**why is that?**

The reason for this result is because, although having an isometric input [0, 1], the euclidean distance between each point of the output curve is not fixed. This affects the irregularity in the time scaling velocity profile. As we can see from the plot below, the points at the beginning and at the end of the curve are more close to each rather than in the middle of the curve. This phenomenon is more evident as the turning radius gets smaller.

![plot](/script/trajectory%20planning/graphs/end_effector_trajectorty_linear_ts.png)


#### time scaling function of a straight line path
On the other hand if we take a straight line as path to follow, we can see from the plots below, that the position and velocity time scaling profiles are as expected of a bang-bang control.

##### position time scaling function
+ Ist segment: constant acceleration
+ IInd segment: constant velocity
+ IIIrd segment: constant deceleration

![plot](/script/trajectory%20planning/graphs/position_time_scaling_profile_segment.png)

###### velocity time scaling function

![plot](/script/trajectory%20planning/graphs/veloity_time_scaling_profile_segment.png)


### gepetto viewer

https://github.com/ostifede02/2dr/assets/113505533/8275950d-85f3-4202-8fd4-2156c668f8a0
