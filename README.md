# 2Delta robot

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

## project setup
### how to solve the problem
Since the URDF file and the ik algorithm can only solve for open chain systems, we are going to break down the problem, solving the ik for each pair of links, and let them converge to the same desired point.

### variabels
After the URDF file is imported and parsed, we are going to take into account only the Y and Z coordinates for the position of the end-effector and the Y and Z partial derivatives for the Jacobian matrix.

### script
In the script folder there are three subfolders: CHAIN_12, CHAIN_1 and CHAIN_2. This has been done for troubleshooting purposes. The problem has been break down in two simpler problems. The CHAIN_1 folder contains the left carriage and linkage (n.1 becuase has only one pair of linkages), the CHAIN_2 folder contains the right carriage and linkage (n.2 because has two pair of linkages).


### problems
The CHAIN_1 seems to be okay for small position displacements. However the Gauss-Newton algorithm converges, but with to many iterations (from 50 to 300 iterations).

![plot](./script/img/CHAIN_1_view.png)

The CHAIN_2 converges to the wrong solution.

![plot](./script/img/CHAIN_2_problem.png)