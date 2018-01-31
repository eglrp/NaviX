#!/bin/bash
export PATH=$PATH:/opt/ros/indigo/bin:/usr/local/java/jdk1.8.0_144/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games
source /opt/ros/kinetic/setup.sh
source /opt/catkin_ws/devel/setup.bash

basepath=$(cd `dirname $0`; pwd)
#python /home/wellcasa/catkin_ws/js/init.py 
#echo "a" |sudo -S python $basepath/chmod.py
roslaunch $basepath/init.launch &
