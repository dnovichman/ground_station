/***********************************************************************************
*										   *
*	Developer: Moses Bangura						   *
*			moses.bangura@anu.edu.au				   *
*			dnovichman@hotmail.com					   *
*			Australian National University          		   *
************************************************************************************/

#include "ros/ros.h"

#include "px4_ros/px4_ros.h"
#include "mavlink_ros/Mavlink.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_datatypes.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "lmpc_V1.h"

#include "mavlink.h"
#include <glib.h>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <stdio.h>   
#include <string.h>  
#include <unistd.h>  
#include <fcntl.h>   
#include <errno.h>   
#include <termios.h> 
#ifdef __linux
#include <sys/ioctl.h>
#endif

#include <sys/time.h>
#include <time.h>

#ifndef MAV_ROS_H
#define MAV_ROS_H

using std::string;
using namespace std;

struct timeval tv;		  		/**< System time */

int baud = 115200;                  		/**< The serial baud rate */

double rec_t, rec_oldt, rec_dt;

/* General Settings */
int sysid = 1;             			/**< The unique system id of this MAV, 0-127. Has to be consistent across the system */
int compid = 50; 
int serial_compid = 0;
std::string port = "/dev/ttyUSB0";              /**< The serial port name, e.g. /dev/ttyUSB0 */


int fd_port;
double t = 0, pt = 0, dt = 0;
float trajectory[9];

bool vicon_available = false;

float global_x = 0.0f, global_y = 0.0f, global_z = 0.0f, global_vx = 0.0f, global_vy, global_vz = 0.0f;
float global_roll = 0.0f, global_pitch = 0.0f, global_yaw = 0.0f;

void get_trajectory(lmpc_v1::lmpc_V1 msg);


#define BUFFER_LENGTH 20410
uint8_t buf[BUFFER_LENGTH];
float RC[8];

mavlink_vicon_position_estimate_t pos; //vicon data
mavlink_attitude_quaternion_t att_vicon;
mavlink_set_position_target_local_ned_t traj;

ros::Subscriber mavlink_sub;
ros::Publisher mavlink_pub;
ros::Publisher imu_pub;
ros::Publisher imu_raw_pub;
ros::Publisher mag_pub;

mavlink_highres_imu_t imu_raw;
std::string frame_id("fcu");

double dtt = 0.005, dtv = 0, ptv =0, tv1 = 0, tv0;


px4_ros::px4_ros px4_msg;
ros::Publisher px4_pub;

float des_yaw;

ros::Publisher vec_state_pub;
ros::Publisher vec_vic_gps_pub;
void stuff_vehicle_state(void);
void stuff_vehicle_vic_gps(void);
std_msgs::Float64MultiArray vec_state;
std_msgs::Float64MultiArray vec_vic_gps;

mavlink_gps_raw_int_t 	gps_data;
mavlink_attitude_target_t quad_control_target;
mavlink_heartbeat_t heart_beat;
mavlink_home_position_t vehicle_home_pos;

double roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

#endif
