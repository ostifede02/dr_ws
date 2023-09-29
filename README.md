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
### script
In the script folder there are three subfolders: CHAIN_12, CHAIN_1 and CHAIN_2. This has been done for troubleshooting purposes. The problem has been break down in two simpler problems. The CHAIN_1 folder contains the left (has only one pair of linkages) carriage and linkage, the CHAIN_2 folder contains the right (has two pair of linkages) carriage and linkage.

#### problems
The CHAIN_1 seems to be okay, for small position displacements, the Gauss-Newthon algorithm converges in less than 30 iterations (which still can be improved).
On the other hand, the CHAIN_2 converges to the wrong solution