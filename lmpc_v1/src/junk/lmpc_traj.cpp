/**
*	Develeoper: Moses Bangura
*			moses.bangura@anu.edu.au
*			dnovichman@hotmail.com
*			Australian National University
*/


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
#include <string.h>
#include <vector>
#include <math.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <cstdlib>

#include <Eigen/Dense>
#include "lmpc_V1.h"



//95911029

#define pi 3.1415926
#define g 9.81
#define ts 0.018182// 50Hz used as baseline

#define m 2.16  //mass of vehicle between 2.2 and 2.3 2.2980 2.27 2.15


using namespace std;
using namespace Eigen;

lmpc_v1::lmpc_V1 lmpc_msg;

/* For logging data */
ros::Publisher lmpc_desired; // = n.advertise<std_msgs::Float64MultiArray>("LMPC_DESIRED", 100);

//! Declarations for velocity observer
double dtt = 0.005;
double dtv = 0, ptv =0, tv = 0;
double v_max = 3;
double vx_hat = 0, vy_hat = 0, vz_hat = 0, vx_td = 0,  vy_td = 0, vz_td = 0, dvx_hat = 0, dvy_hat = 0, dvz_hat = 0, dvx_td = 0,  dvy_td = 0, dvz_td = 0;
double k = 60;
double t = 0, pt = 0, dt = 0;
double x = 0, y = 0, z = 0, px = 0, py = 0, pz = 0, x_tmp = 0, y_tmp = 0, z_tmp = 0 , px_tmp = 0, py_tmp = 0, pz_tmp = 0;
double pex = 0, pey = 0, pez = 0;
double ex = 0, ey = 0, ez = 0, evx = 0, evy = 0, evz = 0, vmag = 0;
double vx = 0, vy = 0, vz = 0, pvx = 0, pvy = 0, pvz = 0, pdvx = 0, pdvy = 0, pdvz = 0;
double acc_x = 0, acc_y = 0, acc_z = 0;
double lp_ex = 0, lp_ey = 0, lp_ez = 0,lp_vx = 0, lp_vy = 0, lp_vz = 0, lp_dvx = 0, lp_dvy = 0, lp_dvz = 0;
double ix = 0, iy = 0, iz = 0, ivx = 0, ivy = 0, ivz = 0, dx = 0, dy = 0, dz = 0, dvx = 0, dvy = 0, dvz = 0;
double ix_sat = 0, iy_sat = 0, iz_sat = 0;
double ax = 0, ay = 0, az = 0, b = 0, c = 0;

double q0 = 0, q1 = 0, q2 = 0, q3 = 0;
double roll = 0, pitch = 0, yaw = 0;
// End Declarations for velocity observer

//!For yaw control
float kp_yaw = 1, ki_yaw = 1, yaw_ref = 0, e_yaw = 0, yaw_rate = 0, i_yaw = 0, sat_yaw = 0.4, pe_yaw = 0;


//Begin declarations for control outputs to RC
double z_con = m*g, x_con, y_con, yaw_con, con_in[3], Omega_x, Omega_y, Omega_z;
//End control declarations

int less_than_zero(int y);
MatrixXf matrix_power(MatrixXf power, int n);


//! Declarations for LMPC 
std_msgs::Float64MultiArray msg;
int N = 5; //Optimisation horizon
float alpha = 0.01; 
MatrixXf Q(9,9);
MatrixXf R(3,3);
MatrixXf A(9,9);
MatrixXf B(9,3);
MatrixXf Ident9(9,9);


MatrixXf Cee(9*N,3*N);
MatrixXf Q_bar(9*N,9*N);
MatrixXf R_bar(3*N,3*N);
MatrixXf M(9*N,9); //*N
MatrixXf K_LMPC;
float MPC_K = 0.9;

float MPC_L[3] = {0,0,0};
float MPC_Ldot[3] = {0,0,0};
float old_L[3] = {0, 0, 0};

int sizeA_1 = 9;
int sizeB_1 = 9;
int sizeB_2 = 3;
int sizeR_1 = 3;
int sizeR_2 = 3;
float beta = 0.01; 

int flight = -1, prev_flight = -1; //Flag for switching flight state
double time_original, time_mpc = 0;

//Array<float, 9, 1> state_error;
//state_error << {0,0,0,0,0,0,0,0,0};
VectorXf state_error(9);
//state_error << {0,0,0,0,0,0,0,0,0};
//Array<float, 9, 1> Lu; //only need last 3
VectorXf Lu(3);

VectorXf state(9);

//drag coefficient 
float cbar = 0.001; //in x, y, z = 0


float xd[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};//, -1, 0, 0, 0, 0, 0, 0}; //x,y,z,vx,vy,vz,L1,L2,L3, please cross check FIXME
//! End declarations for LMPC


/* Rotation Matrix Calculation */
double R_matrix[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
double cphi, ctheta, cpsi, sphi, stheta, spsi;
/* End Rotation matrix initialisation */

/* Declarations for MPC max limits */
float omega_max = 2, attitude_max = 0.6; //FIXME change back to 0.6
float L_max[4];

// debugging vicon
double v_dt =0, v_oldtime = 0, v_newtime = 0;
bool xpos = true, xneg = true, drag_flag = false;
bool ypos = true, yneg = true, drag_flag_y = false;
bool cvel_flag = false;
float dragvel = 0.0f, dragvel_y = 0.3f;
int xrand, yrand;
bool changep = false, changen = false;
float z_con_min = 19;
bool rectangle = false;
int segment_l = 1;
/*
void constant_velocity_flight(void)
{
	float c_vel = 0.4;
	if (cvel_flag)
	{
		xd[0] = x; 
		xd[1] = 0; 
		xd[2] = -0.7f; 
		xd[3] = c_vel;
		xd[4] = 0; 
		xd[5] = 0; 	
		xd[6] = 0; 
		xd[7] = 0; 
		xd[8] = 0; 
	}

	if (x > 1.0)
	{
}*/

void drag_flight_y(void)
{	
	//srand((unsigned int)time(NULL));
	//srand (time(NULL));	

	if ((y > 0.80f) && (ypos == true)) //&& (changep == false)
	{	
		yrand = rand()%50+25;
		dragvel_y = (float)yrand/100.0f;
		//ROS_INFO("Num is %f", dragvel);
	} else if ((y < -0.80f) && (yneg == true))// && (changen == false))
	{	
		yrand = rand()%50+25;
		dragvel_y = (float)yrand/100.0f;
	}
		
	dragvel_y = 0.3; //uncomment this for random velocities
//ROS_INFO("dragvel1 %f %f",dragvel_y, y);
ROS_INFO("x y z %f %f %f",x, y,z);
	if (drag_flag_y)
	{
		xd[0] = 0; 
		xd[1] = y_tmp; 
		xd[2] = -1.0f; 
		xd[3] = 0; //0.4 was here
		xd[4] = dragvel_y; 
		xd[5] = 0; 	
		xd[6] = 0; 
		xd[7] = 0; 
		xd[8] = 0; 
	}

	if (y_tmp > 1.0f) //y >  0.90f
	{
		ypos = false;
		yneg = true;
		changep = true;
		changen = false;
	}
	else if (y_tmp < -1.0f) //y < -0.9f
	{
		ypos = true;
		yneg = false;
		changep = false;
		changen = true;
	} 

	if ((yneg == true) && (ypos == false))
	{
		xd[0] = 0; 
		xd[1] = y_tmp; 
		xd[2] = -1.0f; 
		xd[3] = 0; //-0.4;
		xd[4] = -dragvel_y; 
		xd[5] = 0; 	
		xd[6] = 0; 
		xd[7] = 0; 
		xd[8] = 0; 
	}
	else if ((yneg == false) && (ypos == true))
	{
		xd[0] = 0; 
		xd[1] = y_tmp; 
		xd[2] = -1.0f; 
		xd[3] = 0; //0.4;
		xd[4] = dragvel_y; 
		xd[5] = 0; 	
		xd[6] = 0; 
		xd[7] = 0; 
		xd[8] = 0; 
	}
	else {
		xd[0] = 0; 
		xd[1] = y_tmp; 
		xd[2] = -1.0f; 
		xd[3] = 0; 
		xd[4] = dragvel_y; 
		xd[5] = 0; 	
		xd[6] = 0; 
		xd[7] = 0; 
		xd[8] = 0; 
	}
}

void drag_flight(void)
{	
	//srand((unsigned int)time(NULL));
	//srand (time(NULL));	

	if ((x > 1.0f) && (xpos == true)) //&& (changep == false)
	{	
		xrand = rand()%50+10;
		dragvel = (float)xrand/100.0f;
		ROS_INFO("Num is %f", dragvel);
	} else if ((x < -1.0f) && (xneg == true))// && (changen == false))
	{	
		xrand = rand()%50+10;
		dragvel = (float)xrand/100.0f;
	}
		
	dragvel = 0.2; //uncomment this for random velocities

	if (drag_flag)
	{
		xd[0] = x; 
		xd[1] = 0; 
		xd[2] = -1.0f; 
		xd[3] = 0.2; //0.4 was here
		xd[4] = 0; 
		xd[5] = 0; 	
		xd[6] = 0; 
		xd[7] = 0; 
		xd[8] = 0; 
	}

	if (x > 1.0)
	{
		xpos = false;
		xneg = true;
		changep = true;
		changen = false;
	}
	else if (x < -1.0f)
	{
		xpos = true;
		xneg = false;
		changep = false;
		changen = true;
	} 

	if ((xneg == true) && (xpos == false))
	{
		xd[0] = x; 
		xd[1] = 0; 
		xd[2] = -1.0f; 
		xd[3] = -dragvel; //-0.4;
		xd[4] = 0; 
		xd[5] = 0; 	
		xd[6] = 0; 
		xd[7] = 0; 
		xd[8] = 0; 
	}
	else if ((xneg == false) && (xpos == true))
	{
		xd[0] = x; 
		xd[1] = 0; 
		xd[2] = -1.0f; 
		xd[3] = dragvel; //0.4;
		xd[4] = 0; 
		xd[5] = 0; 	
		xd[6] = 0; 
		xd[7] = 0; 
		xd[8] = 0; 
	}
}
/*
void drag_flight(void)
{
	if (fabs(x) < 1)
	{
		if (vx > 0)
		{
			xd[0] = x; 
			xd[1] = 0; 
			xd[2] = -1.0f; 
			xd[3] = 0.2;
			xd[4] = 0; 
			xd[5] = 0; 	
			xd[6] = 0; 
			xd[7] = 0; 
			xd[8] = 0; 
		}
		else if (vx < 0)
		{
			xd[0] = x; 
			xd[1] = 0; 
			xd[2] = -1.0f; 
			xd[3] = -0.2;
			xd[4] = 0; 
			xd[5] = 0; 	
			xd[6] = 0; 
			xd[7] = 0; 
			xd[8] = 0; 
		}
	} else 	
	{
		xd[0] = 0.0; 
		xd[1] = 0; 
		xd[2] = -1.0f; 
		xd[3] = 0;
		xd[4] = 0; 
		xd[5] = 0; 	
		xd[6] = 0; 
		xd[7] = 0; 
		xd[8] = 0; 
	}
}
*/

void position_controlxyz(void)
{
	xd[0] = -1.0; 
	xd[1] = -1.0; 
	xd[2] = -0.30; 
	xd[3] = 0;
	xd[4] = 0; 
	xd[5] = 0; 	
	xd[6] = 0; 
	xd[7] = 0; 
	xd[8] = 0;
}

void rectangle_flight(void)
{
	
	if ((y>1.0) && (xd[4]>0.0))
	{
		segment_l = 2;
	}

	else if ((x>1.0) && (xd[3]>0.0))
	{
		segment_l = 3;
	}

	else if ((y<-1.0) && (xd[4]<0.0))
	{
		segment_l = 4;
	}

	else if ((x<-1.0) && (xd[3]<0.0))
	{
		segment_l = 1;
	} 
	
	if (segment_l == 2)
	{
		xd[0] = x; 
		xd[1] = 1.0; 
		xd[2] = -0.80; 
		xd[3] = 0.5;
		xd[4] = 0; 
		xd[5] = 0; 	
		xd[6] = 0; 
		xd[7] = 0; 
		xd[8] = 0;
		
	}

	else if (segment_l == 3)
	{
		xd[0] = 1.0; 
		xd[1] = y; 
		xd[2] = -0.80; 
		xd[3] = 0;
		xd[4] = -0.5; 
		xd[5] = 0; 	
		xd[6] = 0; 
		xd[7] = 0; 
		xd[8] = 0;
	}

	else if (segment_l == 4)
	{
		xd[0] = x; 
		xd[1] = -1.0; 
		xd[2] = -0.80; 
		xd[3] = -0.5;
		xd[4] = 0; 
		xd[5] = 0; 	
		xd[6] = 0; 
		xd[7] = 0; 
		xd[8] = 0;
	}

	else if (segment_l == 1)
	{
		xd[0] = -1.0; 
		xd[1] = y; 
		xd[2] = -0.80; 
		xd[3] = 0.0;
		xd[4] = 0.5; 
		xd[5] = 0; 	
		xd[6] = 0; 
		xd[7] = 0; 
		xd[8] = 0;
	} else //this is the same as segment_l = 1;
	{ //segment_l -=;
		xd[0] = -1.0; 
		xd[1] = y; 
		xd[2] = -0.80f; 
		xd[3] = 0;
		xd[4] = 0.5; 
		xd[5] = 0; 	
		xd[6] = 0; 
		xd[7] = 0; 
		xd[8] = 0; 
	}
	//rectangle = true;
	ROS_INFO("I am here %f %d",x,segment_l);
}

void position_controlx(void)
{
	//This is a hacked solution to make position controller to work
	xd[0] = 1.0; 
	xd[1] = 0; 
	xd[2] = -1.0; 
	xd[3] = 0;
	xd[4] = 0; 
	xd[5] = 0; 	
	xd[6] = 0; 
	xd[7] = 0; 
	xd[8] = 0; 	
}

void position_controlxmin(void)
{
	xd[0] = -1.0; 
	xd[1] = 0; 
	xd[2] = -1.0; 
	xd[3] = 0;
	xd[4] = 0; 
	xd[5] = 0; 	
	xd[6] = 0; 
	xd[7] = 0; 
	xd[8] = 0; 	
}

void position_control(void)
{
	//This is a hacked solution to make position controller to work
	xd[0] = 0.0; 
	xd[1] = -0.80; // -0.6
	xd[2] = -1.0;  
	xd[3] = 0;
	xd[4] = 0; 
	xd[5] = 0; 	
	xd[6] = 0; 
	xd[7] = 0; 
	xd[8] = 0; 	
}

void position_controlyp(void)
{
	//This is a hacked solution to make position controller to work
	xd[0] = 0.0; 
	xd[1] = 1.0; 
	xd[2] = -1.0; 
	xd[3] = 0;
	xd[4] = 0; 
	xd[5] = 0; 	
	xd[6] = 0; 
	xd[7] = 0; 
	xd[8] = 0; 	
}

void land_quad(void)
{	
	// Land quad at current x, y
	xd[0] = x; 
	xd[1] = y; 
	xd[2] = -0.0; 
	xd[3] = 0;
	xd[4] = 0; 
	xd[5] = 0; 	
	xd[6] = 0; //MPC_L[0]; 
	xd[7] = 0; //MPC_L[1]; 
	xd[8] = 0; //MPC_L[2]; 	
}

void mpc_trajectory(void)
{ 
	time_mpc +=dt;
	double omega = 1.0*pi/10; //2.5 was here 0.6 instead of 1
	xd[0] = 1.0*cos(omega*time_mpc); 
	xd[1] = 1.0*sin(2*omega*time_mpc); 
	xd[2] = -1.0; //-0.4;  
	xd[3] = -1.0*omega*sin(omega*time_mpc); 
	xd[4] = 2*1.0*omega*cos(2*omega*time_mpc);
	xd[5] = 0; 	
	xd[6] = m*g*cbar*xd[3]; //error here, there should be mg infront 2.27*g*
	xd[7] = m*g*cbar*xd[4]; //there is an error here, see paper. It should be mgcbar*v FIXME
	xd[8] = 0; 	

	//ROS_INFO("time %f", time_mpc);
}


void desired_trajectory (void)
{
	int i = 0;
	for (i=0; i<9; i++)
	{
		xd[i] = xd[i]; //time dependent variable
	}

	//for testing
	xd[0] = 0;
	xd[1] = 0;
	xd[2] = -1.0; //-0.6;
	xd[3] = 0;
	xd[4] = 0;
	xd[5] = 0; 	

	xd[6] = 0;
	xd[7] = 0; 
	xd[8] = 0; 
}

void control_xvelpositive(void)
{
	xd[0] = x_tmp; 
	xd[1] = 0; 
	xd[2] = -1.0; 
	xd[3] = 0.50; 
	xd[4] = 0; 
	xd[5] = 0; 	
	xd[6] = 0; 
	xd[7] = 0; 
	xd[8] = 0; 
	

	if (x_tmp > 1.30f || abs(x_tmp-1.50f) < 0.2f) 
	{
		xd[0] = 1.50; 
		xd[1] = 0.0; 
		xd[2] = -1.0; 
		xd[3] = 0.0;
		xd[4] = 0.0; 
		xd[5] = 0; 	
		xd[6] = 0; 
		xd[7] = 0; 
		xd[8] = 0; 
	}
}

void control_xvelnegative(void)
{
	xd[0] = x_tmp; 
	xd[1] = 0.0; 
	xd[2] = -1.0; 
	xd[3] = -0.45; 
	xd[4] = 0.0; 
	xd[5] = 0; 	
	xd[6] = 0; 
	xd[7] = 0; 
	xd[8] = 0; 	


	if (x_tmp < -1.20f || abs(-x_tmp-1.40f) < 0.2f) 
	{
		if (x_tmp < -1.20f) {
		xd[0] = -1.40; //1.4
		xd[1] = 0.0; 
		xd[2] = -1.0; 
		xd[3] = 0;
		xd[4] = 0.0; 
		xd[5] = 0; 	
		xd[6] = 0; 
		xd[7] = 0; 
		xd[8] = 0; 
		}
	}
}

void control_yvelpositive(void)
{
	xd[0] = 0.0; 
	xd[1] = y_tmp; 
	xd[2] = -1.0; 
	xd[3] = 0;
	xd[4] = 0.3; 
	xd[5] = 0; 	
	xd[6] = 0; 
	xd[7] = 0; 
	xd[8] = 0; 

	if (y > 0.9f || abs(y_tmp-1.0f) < 0.2f)
	{
		xd[0] = 0.0; 
		xd[1] = 1.0; 
		xd[2] = -1.0; 
		xd[3] = 0.0;
		xd[4] = 0.0; 
		xd[5] = 0; 	
		xd[6] = 0; 
		xd[7] = 0; 
		xd[8] = 0; 
	}
}

void control_yvelnegative(void)
{
	xd[0] = 0.0; 
	xd[1] = y_tmp; 
	xd[2] = -1.0; 
	xd[3] = 0;
	xd[4] = -0.3; 
	xd[5] = 0; 	
	xd[6] = 0; 
	xd[7] = 0; 
	xd[8] = 0; 

	if (y < -0.8f || abs(y_tmp-0.80f) < 0.2f)
	{
		xd[0] = 0.0; 
		xd[1] = -0.80; 
		xd[2] = -1.0; 
		xd[3] = 0;
		xd[4] = 0.0; 
		xd[5] = 0; 	
		xd[6] = 0; 
		xd[7] = 0; 
		xd[8] = 0; 
	}
}

void control_positive1(void)
{
	xd[0] = 1.0; 
	xd[1] = 1.0; 
	xd[2] = -1.0; 
	xd[3] = 0;
	xd[4] = 0; 
	xd[5] = 0; 	
	xd[6] = 0; 
	xd[7] = 0; 
	xd[8] = 0; 	
}

void control_negative1(void)
{
	xd[0] = -1.0; 
	xd[1] = -1.0; 
	xd[2] = -1.0; 
	xd[3] = 0;
	xd[4] = 0; 
	xd[5] = 0; 	
	xd[6] = 0; 
	xd[7] = 0; 
	xd[8] = 0; 	
}

void Desired_State(const std_msgs::Float64MultiArray::ConstPtr& msg2)
{
	/* For now, only position control is to be enabled */
	flight = (int)msg2->data[0];

	if (flight == 2) 
	{		
		position_controlyp();
	}

	else if (flight == 3) //POSITION Cotrol
	{		
		position_control();
	}

	else if (flight == 4)
	{
		position_controlx();
	}

	else if (flight == 5)
	{
		position_controlxmin();
	}

	else if (flight == 6) //Landing
	{
		land_quad();
	} else if (flight == 7) // Trajectory Tracking
	{	
		if (prev_flight == flight)
		{			
			mpc_trajectory();
		} else
			time_mpc = 0;
	} else if (flight == 8)
	{
		if (!drag_flag)
		{
			xd[0] = x; 
			xd[1] = 0; 
			xd[2] = -1.0f; 
			xd[3] = 0.2;
			xd[4] = 0; 
			xd[5] = 0; 	
			xd[6] = 0; 
			xd[7] = 0; 
			xd[8] = 0; 
			drag_flag = true;
		}
		drag_flight();
		ROS_INFO("Drag %f %f",vx,xd[3]);
	} else if (flight == 9)
	{
		/*if (!drag_flag_y)
		{
			xd[0] = 0; 
			xd[1] = y; 
			xd[2] = -1.0f; 
			xd[3] = 0;
			xd[4] = dragvel_y; 
			xd[5] = 0; 	
			xd[6] = 0; 
			xd[7] = 0; 
			xd[8] = 0; 
			drag_flag_y = true;
		}*/
		drag_flight_y();
		//ROS_INFO("Drag %f %f",vx,xd[3]);
	} else if (flight == 11) //control quad to xyz
	{
		position_controlxyz();
		rectangle = false;
		xd[2] = -0.8;
	} else if (flight == 12) //control quad rectangle on ground
	{
		if (!rectangle)
		{
			xd[0] = -1.0; 
			xd[1] = y; 
			xd[2] = -0.80f; 
			xd[3] = 0;
			xd[4] = 0.5; 
			xd[5] = 0; 	
			xd[6] = 0; 
			xd[7] = 0; 
			xd[8] = 0; 
			rectangle = true;
		}
		rectangle_flight();
		xd[2] = -0.8;
	}

	else if (flight == 13)
	{
		//control_yvelpositive();
		control_xvelnegative();
	}

	else if (flight == 14)
	{
		//control_yvelnegative();
		control_xvelpositive();
	}
	else if (flight == 15)
	{
		control_positive1();
	}

	else if (flight == 16)
	{
		control_negative1();
	}
	
	else
	{
		flight = -1;
		desired_trajectory();
	}

	prev_flight = flight;

	/* Stuff up publishing data */
	for (int i=0; i<9; i++)
	{
		lmpc_msg.desired_state[i] = xd[i];
	}
	lmpc_msg.desired_state[2] = -1.1; //-0.7;
	ROS_INFO("LMP des is %f %f %f %f %f",lmpc_msg.desired_state[0], lmpc_msg.desired_state[1], lmpc_msg.desired_state[2], lmpc_msg.desired_state[3], lmpc_msg.desired_state[4]);
	/* Push LMPC Message for comparison */
	lmpc_desired.publish(lmpc_msg);

}


void ViconCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	tv = ros::Time::now().toSec();
    	dtv = tv - ptv;

	/* We need minuses to bring to the first quadrant */
	x_tmp = -msg->transform.translation.y;
	y_tmp = -msg->transform.translation.x;
	z_tmp = - msg->transform.translation.z;
	/* End new Implementation */
	//ROS_INFO("vicon x y z %f %f %f",x_tmp, y_tmp, z_tmp); //check yaw
	

	tf::Quaternion q(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w);
tf::Matrix3x3 R_matrix_test(q);
	R_matrix_test.getEulerYPR(yaw, pitch, roll);

	double p1, r1;
	p1 = pitch;
	r1 = roll;

	pitch = -r1;
	roll = -p1;
	yaw = -yaw;

	cphi   = cos(roll);
	ctheta = cos(pitch);
	cpsi   = cos(yaw);

	sphi   = sin(roll);
	stheta = sin(pitch);
	spsi   = sin(yaw);

	R_matrix[0] = ctheta*cpsi; //11
	R_matrix[1] = ctheta*spsi; //12
	R_matrix[2] = -stheta;     //13

	R_matrix[3] = sphi*stheta*cpsi - cphi*spsi; //21
	R_matrix[4] = sphi*stheta*spsi + cphi*cpsi; //22
	R_matrix[5] = ctheta*sphi;		    //23

	R_matrix[6] = cphi*stheta*cpsi + sphi*spsi; //31
	R_matrix[7] = cphi*stheta*spsi - sphi*cpsi; //32
	R_matrix[8] = ctheta*cphi; 		    //33

	/* Rotate Vehicle & reference position from \frameA to \frameB */
	x = x_tmp*R_matrix[0] + y_tmp*R_matrix[1] + z_tmp*R_matrix[2]; //good
	y = x_tmp*R_matrix[3] + y_tmp*R_matrix[4] + z_tmp*R_matrix[5]; //good
	z = x_tmp*R_matrix[6] + y_tmp*R_matrix[7] + z_tmp*R_matrix[8]; //good



	xd[0] = xd[0]*R_matrix[0] + xd[0]*R_matrix[1] + xd[0]*R_matrix[2]; 
	xd[1] = xd[1]*R_matrix[3] + xd[1]*R_matrix[4] + xd[1]*R_matrix[5]; 
	xd[2] = xd[2]*R_matrix[6] + xd[2]*R_matrix[7] + xd[2]*R_matrix[8]; 

	xd[3] = xd[3]*R_matrix[0] + xd[3]*R_matrix[1] + xd[3]*R_matrix[2]; 
	xd[4] = xd[4]*R_matrix[3] + xd[4]*R_matrix[4] + xd[4]*R_matrix[5]; 
	xd[5] = xd[5]*R_matrix[6] + xd[5]*R_matrix[7] + xd[5]*R_matrix[8]; 

	
	px = px_tmp*R_matrix[0] + py_tmp*R_matrix[1] + pz_tmp*R_matrix[2];
	py = px_tmp*R_matrix[3] + py_tmp*R_matrix[4] + pz_tmp*R_matrix[5];
	pz = px_tmp*R_matrix[6] + py_tmp*R_matrix[7] + pz_tmp*R_matrix[8];

	//! We don't need to rotate L = mge_3 - F_TRe_3 because it is in body frame


// This is what is in System Identification, Estimation and Control for a Cost Effective Open-Source Quadcopter Ikyu Sa

	  // velocity observer
    if (dtv > 0.005){
    
        ptv = tv;
        dtt = dtv;
        vx_hat = (x - px) / dtt;
        vy_hat = (y - py) / dtt;
        vz_hat = (z - pz) / dtt;

        vx = vx * 0.5 + vx_hat *0.5;
        vy = vy * 0.5 + vy_hat *0.5;
        vz = vz * 0.5 + vz_hat *0.5;
	}
	
	//ROS_INFO("vx:%f\tvy:%f\tvz:%f\n",vx,vy,vz);

    px_tmp = x_tmp;
    py_tmp = y_tmp;
    pz_tmp = z_tmp;  
  
    
}



int main(int argc, char **argv)
{
	MatrixXf mod_K(3,9);

	ros::init(argc, argv, "lmpc");
	ros::NodeHandle n;

	ros::Publisher uplink_pub = n.advertise<std_msgs::Float64MultiArray>("VICON_TEST", 100);
	ros::Subscriber uplink_sub = n.subscribe<std_msgs::Float64MultiArray>("Control_Input", 10, Desired_State);
	ros::Subscriber vicon_sub = n.subscribe("vicon/MK2_I2C/MK2_I2C", 1, ViconCallback); 

	lmpc_desired = n.advertise<lmpc_v1::lmpc_V1>("lmpc_V1", 100);

	std_msgs::Float32MultiArray msg2;

	ros::Rate loop_rate(100);

	int count = 0;
	srand(time(0));

	while (ros::ok())
	{		
		t = ros::Time::now().toSec();   
		dt = t - pt;

		if (dt == 0)
		{
		dt = ts;
		}

		pt = t;

		yaw_con = yaw_con; // FIXME remove if set to 1 /2 was here
		x_con = x_con*0.4/2*0.9; //*0.4/2*2; 1.2 because gain in attitude is 0.12/0.1
		y_con = y_con*0.4/2*0.9; // FIXME currently set value on px4 is 2 and in ppm_send is 1.5 0.4/2*1.3
		msg.data.push_back(-y_con - 0.03); //roll need to change signs here given the way px4 is structured
		msg.data.push_back((x_con)/1.2); //pitch 0.01*1.2
		msg.data.push_back(z_con);
		msg.data.push_back(yaw_con); 

		/* Based on this setup, y_con 0 rad/s = 1080 which is the middle point min is -0.4 = 80 0.4=2000  */
		/* Based on this setup, x_con 0 rad/s = 1214 which is the middle point min is -0.4 = 0 0.4=1969  */

		uplink_pub.publish(msg);
		msg.data.clear();
		// %EndTag(PUBLISH)%

		// %Tag(SPINONCE)%
		ros::spinOnce();
		// %EndTag(SPINONCE)%

		// %Tag(RATE_SLEEP)%
		loop_rate.sleep();
		// %EndTag(RATE_SLEEP)%
		++count;
	}


	return 0;
}
