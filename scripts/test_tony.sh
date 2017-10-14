#!/bin/bash
source `pwd`/devel/setup.bash
catkin_make
roslaunch launch/tony.launch 
