/***********************************************************************************
*										   *
*	This ros package gives the user control access to the quadrotor   	   *
*	The input to the system is mode val val val val			    	   *
*    	Please see Chapter 3 of  my thesis for some of the available states within *
*	the finite state machine (lmpc_v1 package) which this application drives   *
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

#include "control_input.h"

/**
  * Read input from terminal 
*/
void read_command(void* serial_ptr)
{

	int i = 0;
	uint8_t curval;
	uint8_t	data_ind = 0;

	while (1) 
	{
		ROS_INFO("In read mode");
		//int r = gets(s);
		fgets(s, sizeof(s), stdin);
		//scanf("%d%f%f%f%f",&number1,&number2,&number3,&number4,&number5);//getchar();getchar();
		output[0] = output[1] = output[2] = output[3] = output[4] = 0;
		
		output[data_ind] = atof(&s[0]);
		for (i=0; i<strlen(s); i++)
		{
			curval = s[i];
			if (curval == 44 && data_ind < 5 || curval == 0x20) //44 is comma and 0x20 is white space
			{
				data_ind++;
				output[data_ind] = atof(&s[i+1]);
			}
		}
		data_ind = 0;
		msg.data.clear();
		usleep(10000);
	}

}



/**
  * Publish as a rostopic
*/
void publish_on_ros(void* serial_ptr)
{
	int fd = *((int*)serial_ptr);

	while (1) 
	{
		ROS_INFO("Usage: mode(int 0-100:Hold,Height,Takeoff,Pos,Vel,Yaw,Land) float(-z) float(yaw) float(x) float(y)");
		//ROS_INFO("Command: %s",s);
		//ROS_INFO("size no1 %d %f %d %f", sizeof(s), atof(&s[0]), strlen(s), atof(&s[5]));
		ROS_INFO("Command: %d %3.3f %3.3f %3.3f %3.3f\n",(int)output[0], output[1],output[2],output[3], output[4]);

		msg.data.push_back(output[0]);
		msg.data.push_back(output[1]);
		msg.data.push_back(output[2]);
		msg.data.push_back(output[3]);
		msg.data.push_back(output[4]);
		cont_pub.publish(msg);
		usleep(10000);
	}
}


int main(int argc, char **argv)
{
	ROS_INFO("usage: position_control mode zref|vzref xref|vxref yref|vyref");

	ros::init(argc, argv, "control_input");
	ros::NodeHandle n;  	
	
	cont_pub = n.advertise<std_msgs::Float64MultiArray>("Control_Input", 10);

  	GThread* serial_thread;
  	GError* err;
  	if (!g_thread_supported())
	{
		g_thread_init(NULL);
		// Only initialize g thread if not already done
	}

	int fd = 0;
	int* fd_ptr = &fd;

	if ((serial_thread = g_thread_create((GThreadFunc)publish_on_ros, (void *)fd_ptr, TRUE, &err)) == NULL)
	{
	printf("Failed to create serial handling thread: %s!!\n", err->message);
	g_error_free(err);
	}
	
	if ((serial_thread = g_thread_create((GThreadFunc)read_command, (void *)fd_ptr, TRUE, &err)) == NULL)
	{
	printf("Failed to create serial handling thread: %s!!\n", err->message);
	g_error_free(err);
	}

	ros::spin();
	
	return 0;
}
