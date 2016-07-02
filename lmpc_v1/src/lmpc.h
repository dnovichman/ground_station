/***********************************************************************************
*										   *
*	Developer: Moses Bangura						   *
*			moses.bangura@anu.edu.au				   *
*			dnovichman@hotmail.com					   *
*			Australian National University          		   *
************************************************************************************/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

#include <tf/transform_datatypes.h>

#include <sstream>
#include <time.h>
#include <stdio.h>
#include <iostream>
//#include <string.h>
#include <string>
#include <vector>
#include <math.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <cstdlib>

#include <Eigen/Dense>
#include "lmpc_V1.h"

#ifndef LMPC_H
#define LMPC_H

using namespace std;
using namespace Eigen;

/* Declaration of global variables */
lmpc_v1::lmpc_V1 lmpc_msg;

// For logging data 
ros::Publisher lmpc_desired;
float phone_des[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float haptic_des[4] = {0.0, 0.0, 0.0, 0.0};
double loopdt, loopt1, loopt2, fig8_t1, fig8_t2; 

int next_wpt = 0, wpt_last = 0;

// Declarations for velocity observer
double dtt = 0.005;
double dtv = 0, ptv =0, tv = 0;
double vx_hat = 0, vy_hat = 0, vz_hat = 0;
double k = 60;
double t = 0, pt = 0, dt = 0;
double x = 0, y = 0, z = 0, px = 0, py = 0, pz = 0, x_tmp = 0, y_tmp = 0, z_tmp = 0 , px_tmp = 0, py_tmp = 0, pz_tmp = 0;

double vx = 0, vy = 0, vz = 0, pvx = 0, pvy = 0, pvz = 0, pdvx = 0, pdvy = 0, pdvz = 0;

double roll = 0, pitch = 0, yaw = 0;
// End Declarations for velocity observer

MatrixXf matrix_power(MatrixXf power, int n);

int flight = -1, prev_flight = -1; //Flag for switching flight state
double time_original, time_mpc = 0;

float xd[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};


/* Rotation Matrix Calculation */
double R_matrix[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
double cphi, ctheta, cpsi, sphi, stheta, spsi;
/* End Rotation matrix initialisation */

bool xpos = true, xneg = true, drag_flag = false;
bool ypos = true, yneg = true, drag_flag_y = false;
bool cvel_flag = false;
float dragvel = 0.0f, dragvel_y = 0.3f;
int xrand, yrand;
bool changep = false, changen = false;
float z_con_min = 19;
bool rectangle = false;
int segment_l = 1;

float des_yaw = 0.0, yaw_ref = 0.0f;


/* Function declaration */
void read_haptic_input(const std_msgs::Float64MultiArray::ConstPtr& msg);
void load_veh_state(void);

#endif
