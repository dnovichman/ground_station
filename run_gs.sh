#!/bin/sh

################################# Terminal 1 #######################################
# Clean and delete ros logs
#rosclean purge

# Start roscore in new terminal 
gnome-terminal --tab -e "bash -c 'roscore' "
sleep 3 #Need time for roscore to establish
################################# Terminal 2 #######################################
# Launch vicon bridge and echo tf msgs
gnome-terminal  --tab -e "bash -c 'roslaunch vicon_bridge vicon.launch' " --tab -e "bash -c 'rostopic echo tf' "
#sleep 3

################################# Terminal 3 #######################################
# Start user inputs, trajectory generator and mavlink_ros to connect to vehicle
gnome-terminal --tab -e "bash -c 'rosrun control_input control_input ' " --tab -e "bash 
-c 'rosrun lmpc_v1 lmpc_v1' " --tab -e "bash -c 'rosrun mavlink_ros mavlink_ros' "



################################# Terminal 4 #######################################
# Start phone app, visualisation software and additional software


