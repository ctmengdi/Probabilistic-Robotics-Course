# README #

Here you can find all the labs corresponding to the Probabilistic Robotics (PR) course of the Erasmus Mundus Master in computer Vision and Robotics (VIBOT).

Remember to pull the repo before starting any prelab or lab session.

### Requeriments ###

* [Ubuntu 16.04 LTS](http://releases.ubuntu.com/16.04/)
* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* Clone this repo in your catkin workspace and compile it:

~~~~
cd ~/catkin_ws/src
git clone https://bitbucket.org/gvallicrosa/probabilistic_labs.git
cd ..
catkin_make
~~~~

### Contents ###

* Lab 0: ROS introduction
* Lab 1: Turtlebot introduction
* Lab 2: Split & Merge algorithm for line extraction
* Lab 3: Particle Filter
* Lab 4: Extended Kalman Filter (EKF)
* Lab 5: Simultaneous Localization and Mapping (SLAM)


### Building the documentation ###

For the documentation of `probabilistic_basics` functions, do the following:

~~~~
sudo apt-get install python-sphinx ros-$ROS_DISTRO-rosdoc-lite
roscd probabilistic_basics
rosdoc_lite .
~~~~

The documentation lies on `doc/html/index.html`

### Problems ###

`roscore` is not starting.

* Add `export LC_ALL=C` to your `.bashrc` file

`rviz` segmentation fault in virtual machine

* Call `export LIBGL_ALWAYS_SOFTWARE=1` before funning rviz
