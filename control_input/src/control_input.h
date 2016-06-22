/***********************************************************************************
*										   *
*	Developer: Moses Bangura						   *
*			moses.bangura@anu.edu.au				   *
*			dnovichman@hotmail.com					   *
*			Australian National University          		   *
************************************************************************************/

#include "ros/ros.h"
#include <cstdlib>

#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

#include <signal.h>
#include <termios.h>
#include <cstdlib>
#include <glib.h>

#ifndef CONTROL_INPUT_H
#define CONTROL_INPUT_H
std_msgs::Float64MultiArray msg;
int number1;
float number2, number3, number4, number5;
float output[5];

char s[200];

ros::Publisher cont_pub;
#endif
