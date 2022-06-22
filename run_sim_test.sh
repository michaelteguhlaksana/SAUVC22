#!/bin/bash

start_sim () {
	cd ~/ardupilot/ArduSub
	sim_vehicle.py -j4 &
	roslaunch mavros apm.launch fcu_url:=udp://0.0.0.0:14550@ &
}

#Start roscore
roscore &

#Start Sim

start_sim &

rosrun testing1 src/SAUVC22/testing1.py &

rostopic echo /mavros/local_position/odom/pose

