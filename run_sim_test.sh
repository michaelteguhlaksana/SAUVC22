#!/bin/bash

start_sim () {
	cd ~/ardupilot/ArduSub
	sim_vehicle.py -j4 &
	roslaunch mavros apm.launch fcu_url:=udp://0.0.0.0:14550@ &
}

#Update code from repo
echo "PULLING FROM REPO........"
git pull

wait

#Start roscore
echo "STARTING ROSCORE.........."
roscore &

#Start Sim
echo "STARTING SIM.........."
start_sim &

#START TEST SCRIPT
echo "STARTING TEST SCRIPT............"
rosrun testing1 src/SAUVC22/testing1.py &

rostopic echo /mavros/local_position/odom/pose

