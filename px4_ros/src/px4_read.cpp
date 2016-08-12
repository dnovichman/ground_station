#include "ros/ros.h"
//#include "px4_ros/px4_ros.h"
//#include "lmpc_v1/lmpc_v1.h"
#include "px4_ros.h"
#include "lmpc_V1.h"
#include <cstdlib>

#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

#include "geometry_msgs/TransformStamped.h"

#include <tf/transform_datatypes.h>


#include "sensor_msgs/Imu.h"

#include "mavlink.h"
#include <glib.h>

// Standard includes
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#ifdef __linux
#include <sys/ioctl.h>
#endif

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>

using std::string;
using namespace std;

struct timeval tv;		  ///< System time

static mavlink_status_t status;
std_msgs::Float64MultiArray msgdata;
px4_ros::px4_ros px4_msg;

// Default Settings 
char ip_default[] = "127.0.0.1";
int port_number = 14550;

//! Declarations for velocity observer
double dtt = 0.005;
double dtv = 0, ptv =0, tv1 = 0, tv0;
double v_max = 3;
double vx_hat = 0, vy_hat = 0, vz_hat = 0, vx_td = 0,  vy_td = 0, vz_td = 0, dvx_hat = 0, dvy_hat = 0, dvz_hat = 0, dvx_td = 0,  dvy_td = 0, dvz_td = 0;
double vx = 0, vy = 0, vz = 0, pvx = 0, pvy = 0, pvz = 0, pdvx = 0, pdvy = 0, pdvz = 0;
double x = 0, y = 0, z = 0, px = 0, py = 0, pz = 0, x_tmp = 0, y_tmp = 0, z_tmp = 0 , px_tmp = 0, py_tmp = 0, pz_tmp = 0;
double R_matrix[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
double cphi, ctheta, cpsi, sphi, stheta, spsi;
double pposx = 0;
int vicon_update = 0;
/**********************************************************/


// Defines 
#define BUFFER_LENGTH 20410 //0
#define pi 3.1415926

// Sending Messages to PX4 and debugging or verbose
static GString* host = g_string_new("150.203.212.200");	///< host name for UDP server
//static GString* port = g_string_new("14550");		///< port for UDP server to open connection
int port = 6000;
int sysid = 3;
int compid = 111;
bool verbose = false;             ///< Enable verbose output
bool debug = false;               ///< Enable debug functions and output
bool send_data = false;           ///< Enable sending MAvlink packets to px4
bool send_control = true;	  ///< Enables control data to be sent to PX4

char target_ip[100];
int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
struct sockaddr_in gcAddr; 
struct sockaddr_in locAddr;
struct sockaddr_in fromAddr;
uint8_t buf[BUFFER_LENGTH];
ssize_t recsize;
socklen_t fromlen;

int bytes_sent;
mavlink_message_t msg;
uint16_t len;
int fd;

double t = 0, pt = 0, dt = 0;
float q0, q1, q2, q3;
float trajectory[9];

void get_trajectory(lmpc_v1::lmpc_V1 msg);


// Sending Messages to PX4
#ifndef INT16_MAX
#define INT16_MAX 0x7fff
#endif
#ifndef INT16_MIN
#define INT16_MIN (-INT16_MAX - 1)
#endif
#ifndef UINT16_MAX
#define UINT16_MAX 0xffff
#endif

#define MAVLINK_OFFBOARD_CONTROL_MODE_NONE 0
#define MAVLINK_OFFBOARD_CONTROL_MODE_RATES 1
#define MAVLINK_OFFBOARD_CONTROL_MODE_ATTITUDE 2
#define MAVLINK_OFFBOARD_CONTROL_MODE_VELOCITY 3
#define MAVLINK_OFFBOARD_CONTROL_MODE_POSITION 4
#define MAVLINK_OFFBOARD_CONTROL_FLAG_ARMED 0x10

mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t sp;
mavlink_set_roll_pitch_yaw_thrust_t sp2;
mavlink_vicon_position_estimate_t pos; //vicon data
mavlink_setpoint_8dof_t traj;
mavlink_vision_speed_estimate_t velos;
mavlink_attitude_quaternion_t att_vicon;
// End sending messages to PX4

/* Ros pubish Global Values */
ros::Publisher imu_pub;
ros::Publisher imu_raw_pub;
ros::Publisher px4_pub;

mavlink_highres_imu_t imu_raw;
std::string frame_id("fcu");



int open_udp(void)
{	
	strcpy(target_ip, host->str);

	memset(&locAddr, 0, sizeof(locAddr));
	locAddr.sin_family = AF_INET;
	locAddr.sin_addr.s_addr = INADDR_ANY;
	locAddr.sin_port = htons(port);

	/* Bind the socket to port 14550 - necessary to receive packets from qgroundcontrol */ 
	fd = bind(sock,(struct sockaddr *)&locAddr, sizeof(struct sockaddr));
	printf("fd %d",fd);

	if ((int)-1 == fd)
    	{
		perror("error bind failed");
		close(sock);
		exit(EXIT_FAILURE);
    	} 
	
	/* Attempt to make it non blocking */
	if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0)
    	{
		fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
		close(sock);
		exit(EXIT_FAILURE);
    	}
	
	
	memset(&gcAddr, 0, sizeof(gcAddr));
	gcAddr.sin_family = AF_INET;
	gcAddr.sin_addr.s_addr = inet_addr(target_ip);
	gcAddr.sin_port = htons(14550);	

	return fd;
}

/* Read abd stuff up messages into ROS */
void read_from_px4(void* serial_ptr)
{
	ROS_INFO("Yay: %d",fd);
	int i = 0;
	int fd = *((int*)serial_ptr);
	
	  while (1) {
		recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);
		usleep(60000); //20000 was here
		//if (recsize > 0)
		ROS_INFO("Size read is %d, %d, %d",recsize,fd,msg.msgid);

		if (recsize > 0)
		{
			for (unsigned int i = 0; i < recsize; i++)
			{
				mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status);
		
			}			
		}
		//usleep(100000);

		/* Decode messages and put into appropriate buffers */
		switch (msg.msgid)
	      	{
		
			case MAVLINK_MSG_ID_ATTITUDE:
			{
				mavlink_attitude_t att;
				mavlink_msg_attitude_decode(&msg, &att);
				//if (debug)
				//	ROS_INFO("roll %f, pitch %f, yaw %f",att.roll*180/pi, att.pitch*180/pi, att.yaw*180/pi);
				sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu);
				t = ros::Time::now().toSec(); 
				dt = t - pt;
				//if (debug)
				//	ROS_INFO("Time Ellapsed %f, Freq Hz %f",t, 1/dt);
				pt = t;

				// TODO: check/verify that these are body-fixed
				imu_msg->angular_velocity.x = att.rollspeed;
				imu_msg->angular_velocity.y = -att.pitchspeed;
				imu_msg->angular_velocity.z = -att.yawspeed;

				// take this from imu high res message, this is supposed to arrive before this one and should pretty much be in sync then
				imu_msg->linear_acceleration.x = imu_raw.xacc;
				imu_msg->linear_acceleration.y = -imu_raw.yacc;
		    		imu_msg->linear_acceleration.z = -imu_raw.zacc;
				imu_msg->header.frame_id = frame_id;
				imu_msg->header.seq = imu_raw.time_usec / 1000;
				imu_msg->header.stamp = ros::Time::now();

				imu_pub.publish(imu_msg);

				/* Stuff up data into px4_ros node */
				px4_msg.attitude[0] = att.roll;
				px4_msg.attitude[1] = att.pitch;
				px4_msg.attitude[2] = att.yaw;

				px4_msg.attitude_rate[0] = att.rollspeed;
				px4_msg.attitude_rate[1] = att.pitchspeed;
				px4_msg.attitude_rate[2] = att.yawspeed;

				px4_msg.linear_accel[0] = imu_raw.xacc;
				px4_msg.linear_accel[1] = -imu_raw.yacc;
				px4_msg.linear_accel[2] = -imu_raw.zacc;
				px4_pub.publish(px4_msg);
			}
			break; 

			case MAVLINK_MSG_ID_HIGHRES_IMU: //This is the same as attitude
			{
				/* decode message */
				mavlink_msg_highres_imu_decode(&msg, &imu_raw);

				std_msgs::Header header;
				header.stamp = ros::Time::now();
				header.seq = imu_raw.time_usec / 1000;
				header.frame_id = frame_id;

				sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu);

				imu_msg->angular_velocity.x = imu_raw.xgyro;
				imu_msg->angular_velocity.y = -imu_raw.ygyro;
				imu_msg->angular_velocity.z = -imu_raw.zgyro;

				imu_msg->linear_acceleration.x = imu_raw.xacc;
				imu_msg->linear_acceleration.y = -imu_raw.yacc;
				imu_msg->linear_acceleration.z = -imu_raw.zacc;

				px4_msg.linear_accel[0] = imu_raw.xacc;
				px4_msg.linear_accel[1] = -imu_raw.yacc;
				px4_msg.linear_accel[2] = -imu_raw.zacc;
				px4_pub.publish(px4_msg);


				imu_msg->header = header;
		    		imu_raw_pub.publish(imu_msg);
			}
			break; 

			case MAVLINK_MSG_ID_ESC_STATUS:
			{
				//ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
				mavlink_esc_status_t esc;
				mavlink_msg_esc_status_decode(&msg, &esc);
				for (int i=0; i<4; i++) { //there are only 4 esc's on quad
					px4_msg.esc_rpm[i] = esc.rpm[i];
					px4_msg.esc_current[i] = esc.current[i];
				}
				//if (debug)
				//	ROS_INFO("ESC is %d,%d,%d,%d",esc.rpm[0],esc.rpm[1],esc.current[0],esc.current[1]);
				px4_pub.publish(px4_msg);
				px4_pub.publish(px4_msg);
			}
			break;

			case MAVLINK_MSG_ID_ROLL_PITCH_YAW_RATES_THRUST_SETPOINT:
			{
				mavlink_roll_pitch_yaw_rates_thrust_setpoint_t tatt_sp;
				mavlink_msg_roll_pitch_yaw_rates_thrust_setpoint_decode(&msg, &tatt_sp);
				px4_msg.thrust_att_rates_sp[0] = tatt_sp.thrust;
				px4_msg.thrust_att_rates_sp[1] = tatt_sp.roll_rate;
				px4_msg.thrust_att_rates_sp[2] = tatt_sp.pitch_rate;
				px4_msg.thrust_att_rates_sp[3] = tatt_sp.yaw_rate;
			//ROS_INFO("Yes received rates %f %f %f %f",tatt_sp.roll_rate, tatt_sp.pitch_rate, tatt_sp.yaw_rate,tatt_sp.thrust);
			}
			break;

			case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
			{
				mavlink_rc_channels_raw_t rc;
				mavlink_msg_rc_channels_raw_decode(&msg, &rc);
				px4_msg.RC[0] = rc.chan1_raw;
				px4_msg.RC[1] = rc.chan2_raw;
				px4_msg.RC[2] = rc.chan3_raw;
				px4_msg.RC[3] = rc.chan4_raw;
				px4_msg.RC[4] = rc.chan5_raw;
				px4_msg.RC[5] = rc.chan6_raw;
				px4_msg.RC[6] = rc.chan7_raw;
				px4_msg.RC[7] = rc.chan8_raw;
			
				//if (debug)
				//	ROS_INFO("%d,%d,%d,%d",rc.chan1_raw,rc.chan2_raw,rc.chan3_raw,rc.chan4_raw);
				px4_pub.publish(px4_msg);
			}
			break;

			case MAVLINK_MSG_ID_RC_CHANNELS_SCALED:
			{
				mavlink_rc_channels_scaled_t rc_sc;
				mavlink_msg_rc_channels_scaled_decode(&msg, &rc_sc);
				px4_msg.RC_Scaled[0] = rc_sc.chan1_scaled;
				px4_msg.RC_Scaled[1] = rc_sc.chan2_scaled;
				px4_msg.RC_Scaled[2] = rc_sc.chan3_scaled;
				px4_msg.RC_Scaled[3] = rc_sc.chan4_scaled;
				px4_msg.RC_Scaled[4] = rc_sc.chan5_scaled;
				px4_msg.RC_Scaled[5] = rc_sc.chan6_scaled;
				px4_msg.RC_Scaled[6] = rc_sc.chan7_scaled;
				px4_msg.RC_Scaled[7] = rc_sc.chan8_scaled;
				px4_pub.publish(px4_msg);
			}
			break;
		}
		memset(buf, 0, BUFFER_LENGTH);
		}
}

/* Send data and commands to PX4 */
void send_to_px4(void* serial_ptr)
{
	int count = 0;
	ROS_INFO("Yay: sending %d",fd);
	int i = 0;
	int fd = *((int*)serial_ptr);
	while (1) 
	{
		memset(buf, 0, BUFFER_LENGTH);
		t = ros::Time::now().toSec(); 
		dt = t - pt;
		if (debug)
			ROS_INFO("Freq Hz %f, yaw %f %f %f",1/dt,pos.yaw*180/pi,pos.pitch*180/pi,pos.roll*180/pi);
		pt = t;

		mavlink_message_t msg_vicon;
		mavlink_message_t msg_des;
		mavlink_message_t msg_vel;
		mavlink_message_t msg_atvicon;
		
		traj.val1 = trajectory[0];//pos.x;
		traj.val2 = trajectory[1]; //pos.y;
		traj.val3 = trajectory[2]; //pos.z;
		traj.val4 = trajectory[3];
		traj.val5 = trajectory[4];
		traj.val6 = trajectory[5];
		traj.val7 = trajectory[6];
		traj.val8 = trajectory[7];
		mavlink_msg_setpoint_8dof_encode(sysid, compid, &msg_des, &traj);
	
		len = mavlink_msg_to_send_buffer(buf, &msg_des);
		bytes_sent = sendto(sock, (char*)buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		//usleep(20000);
		memset(buf, 0, BUFFER_LENGTH);

		/*if (vicon_update==0 && pposx != pos.x)
		//{	ROS_INFO("SHIT");
			pposx = pos.x;
			vicon_update = 1;
			mavlink_msg_vicon_position_estimate_encode(sysid, compid, &msg_vicon, &pos);
			count = count + 1;
			len = mavlink_msg_to_send_buffer(buf, &msg_vicon);
			bytes_sent = sendto(sock, (char*)buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
			if (debug)
			ROS_INFO("Bytes sent is %d,%d\n",bytes_sent,len);

		memset(buf, 0, BUFFER_LENGTH);
		*/

		/* Send Vicon data as quaternion with time stamps */
		att_vicon.q1 = (tv1 -tv0)*1000000;
		att_vicon.q2 = pos.x;
		att_vicon.q3 = pos.y;
		att_vicon.q4 = pos.z;
		att_vicon.rollspeed = pos.roll;
		att_vicon.pitchspeed = pos.pitch;
		att_vicon.yawspeed = pos.yaw;
		mavlink_msg_attitude_quaternion_encode(sysid, compid, &msg_atvicon, &att_vicon);
		len = mavlink_msg_to_send_buffer(buf, &msg_atvicon);
			bytes_sent = sendto(sock, (char*)buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		ROS_INFO("Time %f %f",att_vicon.q1, att_vicon.q2);
			//if (debug)
			//ROS_INFO("Bytes sent is %d,%d\n",bytes_sent,len);
		//}
		/* Send VICON Data */
		//if (debug)
		//	ROS_INFO("x,y,z %f,%f,%f",pos.x,pos.y,pos.z);
		//pos.x = count; //t;
		//pos.y = count/10000;
		//pos.z = -0.3;
		//pos.roll = 0.7*0; //count*2;
		//pos.pitch = 0.1*0;
		//pos.yaw = 0.2*0;
		
		usleep(60000);
		
		
		/* Send Vicon Velocity messages using vision speed info*/
		velos.x = vx;
		velos.y = vy;
		velos.z = vz;
		//ROS_INFO("Vx %f %f %f",vx,vy,vz);
		//mavlink_msg_vision_speed_estimate_encode(sysid, compid, &msg_vel, &velos);
		//len = mavlink_msg_to_send_buffer(buf, &msg_vel);
		//bytes_sent = sendto(sock, (char*)buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		memset(buf, 0, BUFFER_LENGTH);	
		
		
		}
}


void ViconCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	tv1 = ros::Time::now().toSec();
    	dtv = tv1 - ptv;
	ptv = tv1;
	//pos.x = -msg->transform.translation.y;
	//pos.y = -msg->transform.translation.x;
	//pos.z = - msg->transform.translation.z;
	x_tmp = pos.x;
	y_tmp = pos.y;
	z_tmp = pos.z;
	
	/* My Implementation with corrected angles*/   
	q0 = msg->transform.rotation.x;
	q1 = msg->transform.rotation.y;
	q2 = msg->transform.rotation.z;
	q3 = msg->transform.rotation.w;	 
	double roll, pitch, yaw;

	tf::Quaternion q(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w);
tf::Matrix3x3 R_matrix_test(q);
	R_matrix_test.getEulerYPR(yaw, pitch, roll);

	double p1, r1;
	p1 = pitch;
	r1 = roll;	
	
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

	
	px = px_tmp*R_matrix[0] + py_tmp*R_matrix[1] + pz_tmp*R_matrix[2];
	py = px_tmp*R_matrix[3] + py_tmp*R_matrix[4] + pz_tmp*R_matrix[5];
	pz = px_tmp*R_matrix[6] + py_tmp*R_matrix[7] + pz_tmp*R_matrix[8];

	//! We don't need to rotate L = mge_3 - F_TRe_3 because it is in body frame


// This is what is in System Identification, Estimation and Control for a Cost Effective Open-Source Quadcopter Ikyu Sa

	  // velocity observer
    if (dtv > 0.005){
	pos.x = -msg->transform.translation.y;
	pos.y = -msg->transform.translation.x;
	pos.z = - msg->transform.translation.z;
	pos.pitch = -r1;
	pos.roll = -p1;
	pos.yaw = -yaw;
	vicon_update = 0;
    
        
        dtt = dtv;
        vx_hat = (x - px) / dtt;
        vy_hat = (y - py) / dtt;
        vz_hat = (z - pz) / dtt;

        vx = vx * 0.5 + vx_hat *0.5;
        vy = vy * 0.5 + vy_hat *0.5;
        vz = vz * 0.5 + vz_hat *0.5;
	}
	//ROS_INFO("Yes I am in vicon %f",dtv);
	//ROS_INFO("vx:%f\tvy:%f\tvz:%f\n",vx,vy,vz);

    px_tmp = x_tmp;
    py_tmp = y_tmp;
    pz_tmp = z_tmp;  
}

void get_trajectory(lmpc_v1::lmpc_V1 msg)
{	
	for (int i =0; i<9; i++)
	{
		trajectory[i] = msg.desired_state[i];
	}
	ROS_INFO("Traj %f %f %f %f %f %f",trajectory[0], trajectory[1], trajectory[2], trajectory[3], trajectory[5]);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "px4_ros");


	// Handling Program options
	static GOptionEntry entries[] =
	{
			{ "sysid", 'a', 0, G_OPTION_ARG_INT, &sysid, "ID of this system", NULL },
			{ "compid", 'c', 0, G_OPTION_ARG_INT, &compid, "ID of this component", NULL },
			{ "host", 'h', 0, G_OPTION_ARG_STRING, host, "Remote host", host->str },
			{ "port", 'p', 0, G_OPTION_ARG_INT, &port, "Remote port", NULL },
			{ "verbose", 'v', 0, G_OPTION_ARG_NONE, &verbose, "Be verbose", NULL },
			{ "send_data", 's', 0, G_OPTION_ARG_NONE, &send_data, "Be send_data", NULL },
			{ "debug", 'd', 0, G_OPTION_ARG_NONE, &debug, "Debug mode, changes behaviour", NULL },
			{ NULL }
	};

	GError *error = NULL;
	GOptionContext *context;

	context = g_option_context_new ("- translate between MAVLINK broadcast bus and ground control ROS link");
	g_option_context_add_main_entries (context, entries, NULL);
	//g_option_context_add_group (context, NULL);
	if (!g_option_context_parse (context, &argc, &argv, &error))
	{
		g_print ("Option parsing failed: %s\n", error->message);
		exit (1);
	}

  	
	ros::NodeHandle n;
	tv0 = ros::Time::now().toSec();
	px4_pub = n.advertise<px4_ros::px4_ros>("px4_ros", 100); //10 was heerS
	ros::Subscriber vicon_sub = n.subscribe("vicon/MK2_POWER/MK2_POWER", 100, ViconCallback); // wand/MK2_I2C MK2_POWER
	ros::Subscriber lmpc_m_sub = n.subscribe<lmpc_v1::lmpc_V1>("lmpc_v1", 10, get_trajectory); //MBangura

	ros::NodeHandle nh("fcu");
  	imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 10);
	imu_raw_pub = nh.advertise<sensor_msgs::Imu>("imu", 10);

	
	/* Open UDP port */	
	// Print the basic configuration
	printf("Connecting to host %s:%d\n", host->str, port);
	fd = open_udp();

	if (debug)
		ROS_INFO("fd %d",fd);

	GThread* serial_thread;
	GError* err;

	int* fd_ptr = &fd;

	if (!g_thread_supported())
	{
		g_thread_init(NULL);
	// Only initialize g thread if not already done
	}
	
	if ((serial_thread = g_thread_create((GThreadFunc)send_to_px4, (void *)fd_ptr, TRUE, &err)) == NULL)
	{
	printf("Failed to create serial handling thread: %s!!\n", err->message);
	g_error_free(err);
	}
	
	if ((serial_thread = g_thread_create((GThreadFunc)read_from_px4, (void *)fd_ptr, TRUE, &err)) == NULL)
	{
	printf("Failed to create serial handling thread: %s!!\n", err->message);
	g_error_free(err);
	}

	ros::spin();

  	close(fd);
}
