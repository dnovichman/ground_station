/***********************************************************************************
*										   *
*	Developer: Moses Bangura						   *
*			moses.bangura@anu.edu.au				   *
*			dnovichman@hotmail.com					   *
*			Australian National University          		   *
************************************************************************************/

#ifndef GROUND_STATION_H
#define GROUND_STATION_H

#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

//#include <Eigen/Dense>

#define pi 3.1415926
#define g 9.81

#define MAXBUF 1024
#define DELIM "="
#define config_file "/home/dnovichman/ANU_GroundStation/ground_station.config"

//using namespace Eigen;


FILE *fd;

float trans_bound = 0.10f;

char sfile[200];

float wpts[4][4]; //FIXME use dynamic array say eigen library
//MatrixXf wpts2;


/* struct to handle config file copy paste in all ros files */
struct system_config
{
	char install_path[MAXBUF];
	char vehicle_name[MAXBUF];
	char log_file_name[MAXBUF];
	char waypoints_file[MAXBUF];
	int port;
	char dev_address[MAXBUF];
	int baud_rate;
	float max_x;
	float max_y;
	float max_z;
	float max_vxy;
	float max_vz;
	float waypoint_bound;
	float yaw_bound;
};


/* Remove spaces from characters */
void remove_space(char *in, char *out);

/* Read config file */
void read_config(void);

/* Fill waypoint file */
void fill_wpt(char *s);

/* Read waypoint file */
void read_wpt_file(void);

/* Waypoint following condition */
int waypoint_following(void);

/* Check constraints */
void check_constraints(float* des, float* outp);

#endif
