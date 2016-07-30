/***********************************************************************************
*										   *
*	Originally linear model predictive controller trajectory generator   	   *
*	The file accepts inputs from hybrid state generator control_input    	   *
*    	To generate several trajectories, paths and des yaw as lmpc messages       *
*										   *
*	Developer: Moses Bangura						   *
*			moses.bangura@anu.edu.au				   *
*			dnovichman@hotmail.com					   *
*			Australian National University          		   *
************************************************************************************/

/***********************************************************************************	
 *				TODO						   * 
 *  										   *
************************************************************************************/

#include "lmpc.h"
#include "../../gs_source/ground_station.cpp"

/**
  * Waypoint following file
*/
int waypoint_following(void)
{
	//distance to next waypoint
	float x_ref, y_ref, z_ref;
	float xref2, yref2, zref2;
	
	xref2 = wpts[next_wpt][0];
	yref2 = wpts[next_wpt][1];
	zref2 = wpts[next_wpt][2]; 
	
	//float dist = sqrt(pow((x-wpts[next_wpt][0]),2) + pow((y-wpts[next_wpt][1]),2) + pow((z-wpts[next_wpt][2]),2));
	float dist = sqrt(pow((x-xref2),2) + pow((y-yref2),2) + pow((z-zref2),2));	
	
	float heading_error = yaw - wpts[next_wpt][3];

	//determine next waypoint
	if (dist < config.waypoint_bound && fabs(heading_error) < config.yaw_bound) 
	{
		if (next_wpt < 3) //change to dynamic
			next_wpt = next_wpt + 1;
		else
			//go to position hold this will always do a hold here
			return 0;
	}

	//define references
	x_ref = wpts[next_wpt][0];
	y_ref = wpts[next_wpt][1];
	z_ref = wpts[next_wpt][2];
	yaw_ref = wpts[next_wpt][3];

	xd[0] = x_ref;
	xd[1] = y_ref;
	xd[2] = z_ref;
	des_yaw = yaw_ref;

	return 1;
}

/**
  * Constant velocity flights for determining the drag coefficients in y 
*/
void drag_flight_y(void)
{	

	if ((y > 0.80f) && (ypos == true)) 
	{	
		yrand = rand()%50+25;
		dragvel_y = (float)yrand/100.0f;

	} else if ((y < -0.80f) && (yneg == true))
	{	
		yrand = rand()%50+25;
		dragvel_y = (float)yrand/100.0f;
	}
		
	dragvel_y = 0.3; //uncomment this for random velocities

	if (drag_flag_y)
	{
		xd[0] = 0; 
		xd[1] = y_tmp; 
		xd[2] = -1.1f; 
		xd[3] = 0; 
		xd[4] = dragvel_y; 
		xd[5] = 0; 	
		xd[6] = 0; 
		xd[7] = 0; 
		xd[8] = 0; 
	}

	if (y_tmp > 1.0f) // TODO use soft constraint
	{
		ypos = false;
		yneg = true;
		changep = true;
		changen = false;
	}
	else if (y_tmp < -1.0f) //TODO use soft constraint
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
		xd[2] = -1.1f; 
		xd[3] = 0; 
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
		xd[2] = -1.1f; 
		xd[3] = 0;
		xd[4] = dragvel_y; 
		xd[5] = 0; 	
		xd[6] = 0; 
		xd[7] = 0; 
		xd[8] = 0; 
	}
	else {
		xd[0] = 0; 
		xd[1] = y_tmp; 
		xd[2] = -1.1f; 
		xd[3] = 0; 
		xd[4] = dragvel_y; 
		xd[5] = 0; 	
		xd[6] = 0; 
		xd[7] = 0; 
		xd[8] = 0; 
	}
}

/**
  * Drag estimation trajectory generator with random velocities in x 
*/
void drag_flight(void)
{	
	if ((x > 1.0f) && (xpos == true)) 
	{	
		xrand = rand()%50+10;
		dragvel = (float)xrand/100.0f;
	} else if ((x < -1.0f) && (xneg == true))
	{	
		xrand = rand()%50+10;
		dragvel = (float)xrand/100.0f;
	}
		
	dragvel = 0.2; //uncomment this for random velocities

	if (drag_flag)
	{
		xd[0] = x; 
		xd[1] = 0; 
		xd[2] = -1.1f; 
		xd[3] = 0.2; 
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
		xd[2] = -1.1f; 
		xd[3] = -dragvel; 
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
		xd[2] = -1.1f; 
		xd[3] = dragvel; 
		xd[4] = 0; 
		xd[5] = 0; 	
		xd[6] = 0; 
		xd[7] = 0; 
		xd[8] = 0; 
	}
}

/** 
  *This is to test a newly developed application now obsolete 
*/
void position_controlxyz(void)
{
	xd[0] = -1.0; 
	xd[1] = -1.0; 
	xd[2] = -1.30; 
	xd[3] = 0;
	xd[4] = 0; 
	xd[5] = 0; 	
	xd[6] = 0; 
	xd[7] = 0; 
	xd[8] = 0;
}

/**
  * Rectangle flight
*/
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
	} else 
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
	}
}

/**
  * Control to 1.0m in x for testing transient performance only
*/
void position_controlx(void)
{
	xd[0] = 1.2;  //1.6 for TRO paper
	xd[1] = 0; 
	xd[2] = -1.10; 
	xd[3] = 0;
	xd[4] = 0; 
	xd[5] = 0; 	
	xd[6] = 0; 
	xd[7] = 0; 
	xd[8] = 0; 	
}

/**
  * Control to -1.0m in xfor testing transient performance only
*/
void position_controlxmin(void)
{
	xd[0] = -1.5; 
	xd[1] = 0; 
	xd[2] = -1.1; 
	xd[3] = 0;
	xd[4] = 0; 
	xd[5] = 0; 	
	xd[6] = 0; 
	xd[7] = 0; 
	xd[8] = 0; 	
}

/**
  * Control to -0.80m in y for testing transient performance only
*/
void position_control(void)
{
	xd[0] = 0.0; 
	xd[1] = -0.80; 
	xd[2] = -1.1;  
	xd[3] = 0;
	xd[4] = 0; 
	xd[5] = 0; 	
	xd[6] = 0; 
	xd[7] = 0; 
	xd[8] = 0; 	
}

/**
  * Control to 1.0m in y for testing transient performance only
*/
void position_controlyp(void)
{
	xd[0] = 0.0; 
	xd[1] = 1.0; 
	xd[2] = -1.1; 
	xd[3] = 0;
	xd[4] = 0; 
	xd[5] = 0; 	
	xd[6] = 0; 
	xd[7] = 0; 
	xd[8] = 0; 	
}

/**
  * Land quad at current location 
  * TODO desired yaw should be current yaw
*/
void land_quad(void)
{		
	xd[0] = x; 
	xd[1] = y; 
	xd[2] = -0.0; 
	xd[3] = 0;
	xd[4] = 0; 
	xd[5] = 0; 	
	xd[6] = 0;  
	xd[7] = 0;  
	xd[8] = 0;  	
}

/**
  * Figure 8 trajectory 
  * TODO user to specify amplitude and frequency
*/
void figure8(void)
{ 
	fig8_t2 = ros::Time::now().toSec();
	dt = fig8_t2 - fig8_t1;
	time_mpc +=dt;
	double omega = 1.0*pi/10; 
	xd[0] = 1.0*cos(omega*time_mpc); 
	xd[1] = 1.0*sin(2*omega*time_mpc); 
	xd[2] = -1.1;  
	xd[3] = -1.0*omega*sin(omega*time_mpc); 
	xd[4] = 2*1.0*omega*cos(2*omega*time_mpc);
	xd[5] = 0; 	
	xd[6] = 0; 
	xd[7] = 0; 
	xd[8] = 0; 	
	
	fig8_t1 = fig8_t2;
}

/** 
  * Desired differential flat outputs 
*/
void desired_trajectory(void)
{
	int i = 0;
	for (i=0; i<9; i++)
	{
		xd[i] = xd[i];
	}

	xd[0] = 0;
	xd[1] = 0;
	xd[2] = -1.1;
	xd[3] = 0;
	xd[4] = 0;
	xd[5] = 0; 	
	xd[6] = 0;
	xd[7] = 0; 
	xd[8] = 0; 
}

/** 
  * Positive xvel to stop at x = 1.2m
*/
void control_xvelpositive(void)
{
	xd[0] = x_tmp; 
	xd[1] = 0; 
	xd[2] = -1.1; 
	xd[3] = 0.60; 
	xd[4] = 0; 
	xd[5] = 0; 	
	xd[6] = 0; 
	xd[7] = 0; 
	xd[8] = 0; 	

	if (x_tmp > 1.20f || abs(x_tmp-1.60f) < 0.4f) //maybe make this 1.6
	{
		xd[0] = 1.60; 
		xd[1] = 0.0; 
		xd[2] = -1.1; 
		xd[3] = 0.0;
		xd[4] = 0.0; 
		xd[5] = 0; 	
		xd[6] = 0; 
		xd[7] = 0; 
		xd[8] = 0; 
	}
}

/** 
  * Negative xvel to stop at x = -1.2m
*/
void control_xvelnegative(void)
{
	xd[0] = x_tmp; 
	xd[1] = 0.0; 
	xd[2] = -1.1; 
	xd[3] = -0.45; //-0.6
	xd[4] = 0.0; 
	xd[5] = 0; 	
	xd[6] = 0; 
	xd[7] = 0; 
	xd[8] = 0; 

	if (x_tmp < -1.5f || abs(-x_tmp-1.70f) < 0.2f) 
	{
		if (x_tmp < -1.0f) {
		xd[0] = -1.70; 
		xd[1] = 0.0; 
		xd[2] = -1.1; 
		xd[3] = 0;
		xd[4] = 0.0; 
		xd[5] = 0; 	
		xd[6] = 0; 
		xd[7] = 0; 
		xd[8] = 0; 
		}
	}

	//if (x_tmp < 0.6f && x_tmp > -0.6f)
	//	xd[3] = -0.7;
}

/** 
  * Positive yvel to stop at x = 1.0m
*/
void control_yvelpositive(void)
{
	xd[0] = 0.0; 
	xd[1] = y_tmp; 
	xd[2] = -1.1; 
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
		xd[2] = -1.1; 
		xd[3] = 0.0;
		xd[4] = 0.0; 
		xd[5] = 0; 	
		xd[6] = 0; 
		xd[7] = 0; 
		xd[8] = 0; 
	}
}

/** 
  * Negative yvel to stop at x = -1.1m
*/
void control_yvelnegative(void)
{
	xd[0] = 0.0; 
	xd[1] = y_tmp; 
	xd[2] = -1.1; 
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
		xd[2] = -1.1; 
		xd[3] = 0;
		xd[4] = 0.0; 
		xd[5] = 0; 	
		xd[6] = 0; 
		xd[7] = 0; 
		xd[8] = 0; 
	}
}

/**
  * Cotrol vehicle to top right of room only for testing
*/
void control_positive1(void)
{
	xd[0] = 1.0; 
	xd[1] = 1.0; 
	xd[2] = -1.1; 
	xd[3] = 0;
	xd[4] = 0; 
	xd[5] = 0; 	
	xd[6] = 0; 
	xd[7] = 0; 
	xd[8] = 0; 	
}

/**
  * Control vehicle to bottom left of room 
*/
void control_negative1(void)
{
	xd[0] = -1.0; 
	xd[1] = -1.0; 
	xd[2] = -1.1; 
	xd[3] = 0;
	xd[4] = 0; 
	xd[5] = 0; 	
	xd[6] = 0; 
	xd[7] = 0; 
	xd[8] = 0; 	
}



void read_haptic_input(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	haptic_des[0] = msg->data[0];
	haptic_des[1] = msg->data[1];
	haptic_des[2] = msg->data[4];
	haptic_des[3] = msg->data[3];
}

/**
  * Read input from phone for phone control 
*/
void read_phone_input(const std_msgs::Float64MultiArray::ConstPtr& msg2)
{
	phone_des[0] = msg2->data[0];
	phone_des[1] = msg2->data[1];
	phone_des[2] = msg2->data[2];
	phone_des[3] = msg2->data[3];
	phone_des[4] = msg2->data[4];


	loopt2 = ros::Time::now().toSec();
	loopdt = (loopt2 - loopt1);	
}

/**
  * Read the desired command to specify outputs 
  * This is the finite state machine (FSM)... Feel free to add your states
*/
void Desired_State(const std_msgs::Float64MultiArray::ConstPtr& msg2)
{
	/* For now, only position control is to be enabled */
	flight = (int)msg2->data[0];	

	if (flight == 2) 
	{		
		position_controlyp();
		des_yaw = 0.0;
	}

	else if (flight == 3) //POSITION Cotrol
	{		
		position_control();
		des_yaw = 0.0;
	}

	else if (flight == 4)
	{
		position_controlx();
		des_yaw = 0.0;
	}

	else if (flight == 5)
	{
		position_controlxmin();
		des_yaw = 0.0;
	}

	else if (flight == 6) //Landing
	{
		land_quad();
		des_yaw = yaw;
	} else if (flight == 7) // Trajectory Tracking
	{	
		if (prev_flight == flight)
		{			
			figure8();
		} else
			time_mpc = 0;
		des_yaw = 0.0;
	} else if (flight == 8)
	{
		if (!drag_flag)
		{
			xd[0] = x; 
			xd[1] = 0; 
			xd[2] = -1.1f; 
			xd[3] = 0.2;
			xd[4] = 0; 
			xd[5] = 0; 	
			xd[6] = 0; 
			xd[7] = 0; 
			xd[8] = 0; 
			drag_flag = true;
		}
		drag_flight();
		des_yaw = 0.0;
	} else if (flight == 9)
	{
		drag_flight_y();
		des_yaw = 0.0;
	} else if (flight == 10)
	{
		if (!fd)
		{
			ROS_INFO("Waypoint File Not Specified"); //FIXME what to do about this?
		}
		wpt_last = waypoint_following();
		if (wpt_last == 0)
			{
				ROS_INFO("Waypoint Mission complete");
			}
			else
				ROS_INFO("Next Waypoint is x %f, y %f, z%f, yaw %f",wpts[next_wpt][0],wpts[next_wpt][1],wpts[next_wpt][2],wpts[next_wpt][3]);
	} else if (flight == 11) //control quad to xyz
	{
		position_controlxyz();
		rectangle = false;
		xd[2] = -0.8;
		des_yaw = 0.0;
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
		des_yaw = 0.0;
	}

	else if (flight == 13)
	{
		//control_yvelpositive();
		control_xvelnegative();
		des_yaw = 0.0;
	}

	else if (flight == 14)
	{
		//control_yvelnegative();
		control_xvelpositive();
		des_yaw = 0.0;
	}
	else if (flight == 15)
	{
		control_positive1();
		des_yaw = 0.0;
	}

	else if (flight == 16)
	{
		control_negative1();
		des_yaw = 0.0;
	}

	else if (flight == 17)
	{
		if ( phone_des[4] == 0.0f)
		{
			xd[0] = phone_des[0]; 
			xd[1] = phone_des[1]; 
			xd[2] = phone_des[2]; 
			xd[3] = 0.0f;
			xd[4] = 0.0f;
			xd[5] = 0.0f;
			xd[6] = 0.0f;
			xd[7] = 0.0f;
			xd[8] = 0.0f;
			des_yaw += phone_des[3]*pi/180.0f*1/100.0f; //assuming dt = 1/100 which is this loop
		}
		else if (phone_des[4] == 1.0f)
		{
			xd[0] = x;
			xd[1] = y;
			xd[2] = z;
			xd[3] = phone_des[0]*2.0f; 
			xd[4] = phone_des[1]*2.0f; 
			xd[5] = phone_des[2]; 
			xd[6] = 0.0f;
			xd[7] = 0.0f;
			xd[8] = 0.0f;
			des_yaw +=  phone_des[3]*pi/180.0f*1/100.0f;
		}
		
		//lmpc_msg.desired_yaw += des_yaw*pi/180.0f*loopdt;
		//lmpc_msg.desired_yaw = des_yaw;
	} else if (flight == 18)
	{
		xd[0] = x;
		xd[1] = y;
		xd[2] = z;
		xd[3] = haptic_des[0]; 
		xd[4] = haptic_des[1]; 
		xd[5] = haptic_des[2]; 
		xd[6] = 0.0f;
		xd[7] = 0.0f;
		xd[8] = 0.0f;
		des_yaw =  haptic_des[3];
	} else if (flight == 64)
	{
		xd[0] = 1.2f;
		xd[1] = 0.0f;
		xd[2] = 0.0f;
		xd[3] = 0.0f; 
		xd[4] = 0.0f; 
		xd[5] = 0.0f; 
		xd[6] = 0.0f;
		xd[7] = 0.0f;
		xd[8] = 0.0f;
		des_yaw =  0.0f;
	} else if (flight == 60)
	{
		xd[0] = 0;
		xd[1] = 0;
		xd[2] = 0;
		xd[3] = 0.0f; 
		xd[4] = 0.0f; 
		xd[5] = 0.0f; 
		xd[6] = 0.0f;
		xd[7] = 0.0f;
		xd[8] = 0.0f;
		des_yaw =  0.0f;
	} else
	{
		flight = -1;
		desired_trajectory();
		des_yaw = 0.0;
	}

	prev_flight = flight;

	/* Perform constraint checking */
	check_constraints(xd,xd);

	/* Stuff up publishing data */
	for (int i=0; i<9; i++)
	{
		lmpc_msg.desired_state[i] = xd[i];
		lmpc_msg.desired_yaw = des_yaw;
	}

	/* FIXME fix this 
	if (flight != 17)
	{
		des_yaw = 0.0f;
		lmpc_msg.desired_yaw = 0;
	}
	if (flight == 10)
		lmpc_msg.desired_yaw = yaw_ref;
	*/
	
	ROS_INFO("LMP des is %f %f %f %f %f",lmpc_msg.desired_state[0], lmpc_msg.desired_state[1], lmpc_msg.desired_state[2], lmpc_msg.desired_state[3], lmpc_msg.desired_state[4]);
	ROS_INFO("des yaw %f",lmpc_msg.desired_yaw);
	/* Push LMPC Message for comparison */
	lmpc_desired.publish(lmpc_msg);

}


/* This is the vicon callback function. It receives vicon messages 
 * and determines position and velocity in \frameB. For validation of trajectories in the future
 */
void ViconCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	tv = ros::Time::now().toSec();
    	dtv = tv - ptv;

	/* We need minuses to bring to the first quadrant */
	x_tmp = -msg->transform.translation.y;
	y_tmp = -msg->transform.translation.x;
	z_tmp = - msg->transform.translation.z;
	
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

	x = x_tmp;
	y = y_tmp;
	z = z_tmp;

/*
	//TODO check whether these matrices are in R^\top

	R_matrix[0] = ctheta*cpsi; //11
	R_matrix[1] = ctheta*spsi; //12
	R_matrix[2] = -stheta;     //13

	R_matrix[3] = sphi*stheta*cpsi - cphi*spsi; //21
	R_matrix[4] = sphi*stheta*spsi + cphi*cpsi; //22
	R_matrix[5] = ctheta*sphi;		    //23

	R_matrix[6] = cphi*stheta*cpsi + sphi*spsi; //31
	R_matrix[7] = cphi*stheta*spsi - sphi*cpsi; //32
	R_matrix[8] = ctheta*cphi; 		    //33

	/* Rotate Vehicle & reference position from \frameA to \frameB *
	x = x_tmp*R_matrix[0] + y_tmp*R_matrix[1] + z_tmp*R_matrix[2]; 
	y = x_tmp*R_matrix[3] + y_tmp*R_matrix[4] + z_tmp*R_matrix[5]; 
	z = x_tmp*R_matrix[6] + y_tmp*R_matrix[7] + z_tmp*R_matrix[8]; 



	xd[0] = xd[0]*R_matrix[0] + xd[0]*R_matrix[1] + xd[0]*R_matrix[2]; 
	xd[1] = xd[1]*R_matrix[3] + xd[1]*R_matrix[4] + xd[1]*R_matrix[5]; 
	xd[2] = xd[2]*R_matrix[6] + xd[2]*R_matrix[7] + xd[2]*R_matrix[8]; 

	xd[3] = xd[3]*R_matrix[0] + xd[3]*R_matrix[1] + xd[3]*R_matrix[2]; 
	xd[4] = xd[4]*R_matrix[3] + xd[4]*R_matrix[4] + xd[4]*R_matrix[5]; 
	xd[5] = xd[5]*R_matrix[6] + xd[5]*R_matrix[7] + xd[5]*R_matrix[8]; 

	
	px = px_tmp*R_matrix[0] + py_tmp*R_matrix[1] + pz_tmp*R_matrix[2];
	py = px_tmp*R_matrix[3] + py_tmp*R_matrix[4] + pz_tmp*R_matrix[5];
	pz = px_tmp*R_matrix[6] + py_tmp*R_matrix[7] + pz_tmp*R_matrix[8];


	// velocity observer
	if (dtv > 0.005)
	{
		ptv = tv;
		dtt = dtv;
		vx_hat = (x - px) / dtt;
		vy_hat = (y - py) / dtt;
		vz_hat = (z - pz) / dtt;

		vx = vx * 0.5 + vx_hat *0.5;
		vy = vy * 0.5 + vy_hat *0.5;
		vz = vz * 0.5 + vz_hat *0.5;
	}

	px_tmp = x_tmp;
	py_tmp = y_tmp;
	pz_tmp = z_tmp;     
*/
}

/**
  * Load the current vehicle state with more preference given to vehicle outputs
  *
*/
void load_veh_state(void)
{
	lmpc_msg.veh_state[0] = x;
	lmpc_msg.veh_state[1] = y;
	lmpc_msg.veh_state[3] = z;
	/* If vehicle is connected, use its velocities and attitude */
	lmpc_msg.veh_state[4] = vx;
	lmpc_msg.veh_state[5] = vy;
	lmpc_msg.veh_state[6] = vz;
	lmpc_msg.veh_state[7] = roll;
	lmpc_msg.veh_state[8] = pitch;
	lmpc_msg.veh_state[9] = yaw;
	lmpc_desired.publish(lmpc_msg);
}

int main(int argc, char **argv)
{
	/* Set up configuration parameters */
	read_config();
	read_wpt_file(); 
	


	ros::init(argc, argv, "lmpc");
	ros::NodeHandle n;
	
	ros::Subscriber uplink_sub = n.subscribe<std_msgs::Float64MultiArray>("Control_Input", 10, Desired_State);

	ros::Subscriber vicon_sub = n.subscribe(config.vehicle_name, 1, ViconCallback); 

	lmpc_desired = n.advertise<lmpc_v1::lmpc_V1>("lmpc_V1", 100);

	std_msgs::Float32MultiArray msg2;

	/* Frits */
	ros::Subscriber uplinks_sub = n.subscribe<std_msgs::Float64MultiArray>("phone", 100, read_phone_input);

	/* Eric */
	ros::Subscriber uplinkss_sub = n.subscribe<std_msgs::Float64MultiArray>("VICON_CON", 100, read_haptic_input);

	ros::Rate loop_rate(100);

	int count = 0;
	srand(time(0));
	

	while (ros::ok())
	{	
		// %Tag(SPINONCE)%
		ros::spinOnce();
		// %EndTag(SPINONCE)%

		/* Pack vehicle state messages and publish */
		
		load_veh_state();

		// %Tag(RATE_SLEEP)%
		loop_rate.sleep();
		// %EndTag(RATE_SLEEP)%
		++count;
		loopt1 = loopt2;
	}


	return 0;
}
