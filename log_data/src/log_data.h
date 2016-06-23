#include "ros/ros.h"
#include "px4_ros.h"
#include "lmpc_V1.h"
#include<time.h>
#include <cstdlib>
#include <string.h>


#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

#include <tf/transform_datatypes.h>

#include <sys/stat.h>

using namespace std;
char fileName[100];
char log_dir[100] = "/log_data/logs";//"/home/dnovichman/ANU_GroundStation/log_data/logs";
char log_tmp_dir[100];
FILE *fd_log;


#define ts 0.018182// 50Hz used as baseline

double t = 0, pt = 0, dt = 0;
double dtt = 0.005;
double dtv = 0, ptv =0, tv = 0;

double x = 0, y = 0, z = 0, px = 0, py = 0, pz = 0, x_tmp = 0, y_tmp = 0, z_tmp = 0 , px_tmp = 0, py_tmp = 0, pz_tmp = 0;

/* Data we are getting */
float RC[8];
float attitude[3];
float attitude_rate[3];
float acceleration[3];

float vicon_pos[3];
float vicon_angles[3];
float vicon_vel[3];
float trajectory[9];
float des_yaw;

double t2;

double v_max = 2,vx =0, vy = 0, vz = 0;
double vx_hat = 0, vy_hat = 0, vz_hat = 0, vx_td = 0,  vy_td = 0, vz_td = 0, dvx_hat = 0, dvy_hat = 0, dvz_hat = 0, dvx_td = 0,  dvy_td = 0, dvz_td = 0;
double k = 60;
double acc_x = 0, acc_y = 0, acc_z = 0;

/* Rotation Matrix Calculation */
double R_matrix[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
double cphi, ctheta, cpsi, sphi, stheta, spsi;

time_t t_start, t_end, t1_start;
time_t rawtime;
struct tm * timeinfo;
