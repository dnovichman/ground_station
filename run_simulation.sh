#!/bin/sh

################################# Terminal 1 #######################################

# Run SITL similation
gnome-terminal -x bash -c 'cd /home/dnovichman/src/anu/Firmware;
no_sim=1 make posix_sitl_default gazebo'

# Run ROS node in terminal 2
gnome-terminal -x bash -c 'cd /home/dnovichman/src/anu/Firmware;
source integrationtests/setup_gazebo_ros.bash $(pwd);
roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/Tools/sitl_gazebo/worlds/iris_opt_flow.world'

# Run ROS node in terminal 3
sleep 5
gnome-terminal -x bash -c 'sleep 5; rosrun image_view image_view image:=/flow_camera/image_raw'


