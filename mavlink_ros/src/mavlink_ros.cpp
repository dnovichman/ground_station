/***********************************************************************************
*										   *
* @file										   *
*   @brief The serial interface process						   *
*										   *	
*   This process connects any external MAVLink UART device to ROS		   *
*										   *
*   @author Lorenz Meier, <mavteam@student.ethz.ch>
*										   *
************************************************************************************/

/***********************************************************************************
*	Developer: Moses Bangura						   *
*			moses.bangura@anu.edu.au				   *
*			dnovichman@hotmail.com					   *
*			Australian National University          		   *
************************************************************************************/

/***********************************************************************************	
*				TODO						   * 
*  										   *
************************************************************************************/

#include "mav_ros.h"
#include "../../gs_source/ground_station.cpp"

/**
  * file to publish vehicle state
*/
void stuff_vehicle_state(void)
{
	/* vec_state = [x y z pthi theta psi Vx Vy Vz] */
	vec_state.data.push_back(global_x);
	vec_state.data.push_back(global_y);
	vec_state.data.push_back(global_z);
	vec_state.data.push_back(global_roll);
	vec_state.data.push_back(global_pitch);
	vec_state.data.push_back(global_yaw); 
	vec_state.data.push_back(global_vx);
	vec_state.data.push_back(global_vy);
	vec_state.data.push_back(global_vz);

	vec_state_pub.publish(vec_state);
	vec_state.data.clear();
}

/**
  * file to publish gps and vicon status
*/
void stuff_vehicle_vic_gps(void)
{
	/* vec_vic_gps = [int(vicon/gps) nSats lat lon alt] */
	float sta, body_filter;
	if (vicon_available)
		sta = 1.0f;
	else
		sta = 0.0f;

	//if (int(sta) == 0 && gps_data.satellites_visible < 5)
	//	sta = 2.0f;

	if (quad_control_target.thrust >= 0.3f && heart_beat.system_status >= 4)
		body_filter = 1.0f;
	else
		body_filter = 0.0f;

	vec_vic_gps.data.push_back(sta);
	vec_vic_gps.data.push_back(float (gps_data.satellites_visible));
	vec_vic_gps.data.push_back(body_filter);
	vec_vic_gps.data.push_back(float (gps_data.lat * 1e-7 * pi/180.0f)); // in radians
	vec_vic_gps.data.push_back(float (gps_data.lon * 1e-7 * pi/180.0f)); // in radians
	vec_vic_gps.data.push_back(float (gps_data.alt)); 

	vec_vic_gps_pub.publish(vec_vic_gps);
	vec_vic_gps.data.clear();
}

/**
 * Returns the file descriptor on success or -1 on error.
 */
int open_port(std::string& port)
{
  int fd_port; /* File descriptor for the port */

  // Open serial port
  // O_RDWR - Read and write
  // O_NOCTTY - Ignore special chars like CTRL-C
  fd_port = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_port == -1)
  {
    /* Could not open the port. */
    return (-1);
  }
  else
  {
    fcntl(fd_port, F_SETFL, 0);
  }

  return (fd_port);
}

bool setup_port(int fd_port, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
  //struct termios options;

  struct termios config;
  if (!isatty(fd_port))
  {
    fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd_port);
    return false;
  }
  if (tcgetattr(fd_port, &config) < 0)
  {
    fprintf(stderr, "\nERROR: could not read configuration of fd_port %d\n", fd_port);
    return false;
  }
  //
  // Input flags - Turn off input processing
  // convert break to null byte, no CR to NL translation,
  // no NL to CR translation, don't mark parity errors or breaks
  // no input parity check, don't strip high bit off,
  // no XON/XOFF software flow control
  //
  config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
  //
  // Output flags - Turn off output processing
  // no CR to NL translation, no NL to CR-NL translation,
  // no NL to CR translation, no column 0 CR suppression,
  // no Ctrl-D suppression, no fill characters, no case mapping,
  // no local output processing
  //
  config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

#ifdef OLCUC
  config.c_oflag &= ~OLCUC;
#endif

#ifdef ONOEOT
  config.c_oflag &= ~ONOEOT;
#endif

  //
  // No line processing:
  // echo off, echo newline off, canonical mode off,
  // extended input processing off, signal chars off
  //
  config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
  //
  // Turn off character processing
  // clear current char size mask, no parity checking,
  // no output processing, force 8 bit input
  //
  config.c_cflag &= ~(CSIZE | PARENB);
  config.c_cflag |= CS8;
  //
  // One input byte is enough to return from read()
  // Inter-character timer off
  //
  config.c_cc[VMIN] = 1;
  config.c_cc[VTIME] = 10; // was 0

  // Get the current options for the port
  //tcgetattr(fd_port, &options);

  switch (baud)
  {
    case 1200:
      if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
      {
        fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
        return false;
      }
      break;
    case 1800:
      cfsetispeed(&config, B1800);
      cfsetospeed(&config, B1800);
      break;
    case 9600:
      cfsetispeed(&config, B9600);

      cfsetospeed(&config, B9600);
      break;
    case 19200:
      cfsetispeed(&config, B19200);
      cfsetospeed(&config, B19200);
      break;
    case 38400:
      if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
      {
        fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
        return false;
      }
      break;
    case 57600:
      if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
      {
        fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
        return false;
      }
      break;
    case 115200:
      if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
      {
        fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
        return false;
      }
      break;

      // These two non-standard (by the 70'ties ) rates are fully supported on
      // current Debian and Mac OS versions (tested since 2010).
    case 460800:
      if (cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0)
      {
        fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
        return false;
      }
      break;
    case 921600:
      if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0)
      {
        fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
        return false;
      }
      break;
    default:
      fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
      return false;

      break;
  }

  //
  // Finally, apply the configuration
  //
  if (tcsetattr(fd_port, TCSAFLUSH, &config) < 0)
  {
    fprintf(stderr, "\nERROR: could not set configuration of fd_port %d\n", fd_port);
    return false;
  }
  return true;
}

void close_port(int fd_port)
{
  close(fd_port);
}

/**
  * Read trajectory input data 
*/
void get_trajectory(lmpc_v1::lmpc_V1 msg)
{	
	for (int i =0; i<9; i++)
	{
		trajectory[i] = msg.desired_state[i];
	}
	des_yaw = msg.desired_yaw;

	// Hadle CMU stuff
	for (int i=0; i<3; i++)
	{
		cmu_mcmd.pos[i] = (int16_t)trajectory[i]*1e3;
		cmu_mcmd.vel[i] = (int16_t)trajectory[2+i]*1e3;
		cmu_mcmd.acc[i] = (int16_t)trajectory[5+i]*1e3;
		cmu_mcmd.jerk[i] = 0;
	}
	cmu_mcmd.pos[2] = (int16_t)(1.1*1e3); //fix my convention with labs
	cmu_mcmd.heading[0] = (int16_t)(des_yaw*1e4);
	cmu_mcmd.heading[1] = 0;
	cmu_mcmd.heading[2] = 0;
	cmu_mcmd.target_system = sysid;

	cmu_motor_state.state = 1;
	cmu_motor_state.target_system = sysid;	
}

/** 
   * Vicon position estimates
*/
void ViconCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{	
	tv1 = ros::Time::now().toSec();
    	dtv = tv1 - ptv;
	ptv = tv1;	

	/* Correct angles to match north-east-down standard*/   
	
	tf::Quaternion q(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w);
tf::Matrix3x3 R_matrix_test(q);
	R_matrix_test.getEulerYPR(yaw, pitch, roll);

	double p1, r1;
	p1 = pitch;
	r1 = roll;	
	
	pos.x = -msg->transform.translation.y;
	pos.y = -msg->transform.translation.x;
	pos.z = - msg->transform.translation.z;
	pos.pitch = -r1;
	pos.roll = -p1;
	pos.yaw = -yaw;

	dtt = dtv;	

	global_x = pos.x;
	global_y = pos.y;
	global_z = pos.z;

	global_roll 	= pos.roll;
	global_pitch 	= pos.pitch;
	global_yaw 	= pos.yaw;
	//stuff_vehicle_state();

	vicon_available = true;

	//Handle CMU stuff	
	cmu_mpose.npose = 1;
	cmu_mpose.ids[0] = sysid;
	cmu_mpose.pose[0] = (int16_t)(msg->transform.translation.x*1000);	
	cmu_mpose.pose[1] = (int16_t)(msg->transform.translation.y*1000);
	cmu_mpose.pose[2] = (int16_t)(msg->transform.translation.z*1000);
	cmu_mpose.pose[3] = (int16_t)(yaw*10000);

	// CMU controller gains
	for (int i=0; i<3; i++)
	{
		cmu_att_cmd_gains.kR[i] = 0.0f;
		cmu_att_cmd_gains.kOm[i] = 0.0f;
		cmu_pos_cmd_gains.kp[i] = 0.0f;
		cmu_pos_cmd_gains.kd[i] = 0.0f;
	}
	/* Position gains */
	cmu_pos_cmd_gains.kp[0] = 15.0f;
	cmu_pos_cmd_gains.kp[1] = 15.0f;
	cmu_pos_cmd_gains.kp[2] = 30.0f;

	cmu_pos_cmd_gains.kd[0] = 7.746f;
	cmu_pos_cmd_gains.kd[1] = 7.746f;
	cmu_pos_cmd_gains.kd[2] = 10.9545f;

	/* Attitude control gains */
	cmu_att_cmd_gains.kR[0] = 130.0f;
	cmu_att_cmd_gains.kR[1] = 137.80f;
	cmu_att_cmd_gains.kR[2] = 100.0f;

	cmu_att_cmd_gains.kOm[0] = 22.8035;
	cmu_att_cmd_gains.kOm[1] = 23.4776;
	cmu_att_cmd_gains.kOm[2] = 20.0f;


	cmu_att_cmd_gains.target_system = sysid;
	cmu_pos_cmd_gains.target_system = sysid;
}


/**
  * write to uart/serial both position and desired trajectory 
*/
void write_to_mavlink(void* serial_ptr)
{
	float freq;
	int bytes_sent;
	uint16_t len;

	float old_vt = 0, vic_freq = 0;
	
	int count = 0;
	while (1)
	{

		t = ros::Time::now().toSec(); 

		/* write vicon data */
		mavlink_message_t msg_vicon;
		mavlink_message_t msg_des;

		mavlink_message_t cmu_multiple_pos, cmu_cascaded_att_gains, cmu_cascaded_pos_gains, cmu_pos_command, cmu_motor;
		

		/* Write desired position/trajectory setpoint this should really be hardcoded in px4 */
		traj.x = trajectory[0];
		traj.y = trajectory[1]; 
		traj.z = trajectory[2]; 
		traj.vx = trajectory[3];
		traj.vy = trajectory[4];
		traj.vz = trajectory[5];
		traj.afx = trajectory[6];
		traj.afy = trajectory[7];
		traj.afz = trajectory[8];
		traj.yaw = des_yaw;
		traj.target_system = sysid;
		traj.target_component = compid;
		traj.coordinate_frame = 8;
		
		ROS_INFO("x,y,z %u,%u,%u,%u",cmu_mpose.pose[0],cmu_mpose.pose[1],cmu_mpose.pose[2],cmu_mpose.pose[3]); 
		//usleep(25000);
		usleep(12500);
		memset(buf, 0, BUFFER_LENGTH);
				
		tv1 = ros::Time::now().toSec();
		pos.usec = (tv1 -tv0)*1000000;
		traj.time_boot_ms = (uint32_t)pos.usec;

		mavlink_msg_vicon_position_estimate_encode(sysid, compid, &msg_vicon, &pos);

		dt = t - pt;
		freq = 1/dt;
cmu_mpose.time_usec = uint32_t(pos.usec);
cmu_mcmd.time_usec = uint32_t(pos.usec);
cmu_att_cmd_gains.time_usec = uint32_t(pos.usec);
cmu_pos_cmd_gains.time_usec = uint32_t(pos.usec);
cmu_motor_state.time_usec = uint32_t(pos.usec);
		traj.time_boot_ms = uint32_t(pos.usec);

		vic_freq = 1/(att_vicon.q1 - old_vt)*1000000;
		ROS_INFO("dt and Freq bytes time %f %f %f",dt,freq, vic_freq);

		// TODO only publish when data is available
		float check_time = pos.usec ;
		float old_time_check = 0.0f;
		stuff_vehicle_vic_gps();

		// Publish CMU stuff
		mavlink_msg_mocap_multi_pose_encode(sysid, compid, &cmu_multiple_pos, &cmu_mpose);
		mavlink_msg_cascaded_cmd_gains_encode(sysid, compid, &cmu_cascaded_att_gains, &cmu_att_cmd_gains);
		mavlink_msg_mocap_position_cmd_gains_encode(sysid, compid, &cmu_cascaded_pos_gains, &cmu_pos_cmd_gains);
		mavlink_msg_mocap_position_cmd_encode(sysid, compid, &cmu_pos_command, &cmu_mcmd);
		mavlink_msg_mocap_motor_state_encode(sysid, compid, &cmu_motor, &cmu_motor_state);

		if (dt !=dt || dt > 0.01f)
		{
			stuff_vehicle_state();
			mavlink_msg_set_position_target_local_ned_encode(sysid, compid, &msg_des, &traj);
			len = mavlink_msg_to_send_buffer(buf, &msg_des);
			bytes_sent += write(fd_port, (char*)buf, len);
			memset(buf, 0, BUFFER_LENGTH);

			//if (vicon_available)
			//{
				//len = mavlink_msg_to_send_buffer(buf, &msg_vicon);
				//bytes_sent += write(fd_port, (char*)buf, len);
				//ROS_INFO("true yaw pitch roll %f %f %f\n",pos.yaw*180/pi,pos.pitch*180/pi,pos.roll*180/pi); 
				vicon_available = false;
				ROS_INFO("time check %f %f\n",(check_time - old_time_check)/1000000,1000000/(check_time - old_time_check));
				old_time_check=check_time;

				// Send CMU stuff
				len = mavlink_msg_to_send_buffer(buf, &cmu_multiple_pos);
				bytes_sent += write(fd_port, (char*)buf, len);
				memset(buf, 0, BUFFER_LENGTH);
				len = mavlink_msg_to_send_buffer(buf, &cmu_cascaded_att_gains);
				bytes_sent += write(fd_port, (char*)buf, len);
				memset(buf, 0, BUFFER_LENGTH);
				len = mavlink_msg_to_send_buffer(buf, &cmu_cascaded_pos_gains);
				bytes_sent += write(fd_port, (char*)buf, len);
				memset(buf, 0, BUFFER_LENGTH);
				len = mavlink_msg_to_send_buffer(buf, &cmu_pos_command);
				bytes_sent += write(fd_port, (char*)buf, len);
				memset(buf, 0, BUFFER_LENGTH);

				len = mavlink_msg_to_send_buffer(buf, &cmu_motor);
				bytes_sent += write(fd_port, (char*)buf, len);
				memset(buf, 0, BUFFER_LENGTH);
			//}
			pt = t;	
			count++;
		}					
		old_vt = att_vicon.q1;		
	}
}

/**
  * Read stuff from serial and convert to local ROS msg types 
*/
void* serial_wait(void* serial_ptr)
{
  int fd_port = *((int*)serial_ptr);

  mavlink_status_t lastStatus;
  lastStatus.packet_rx_drop_count = 0;

  // Blocking wait for new data
	while (1)
	{
		uint8_t cp;
		mavlink_message_t message;
		mavlink_status_t status;
		uint8_t msgReceived = false;
		
		rec_t = ros::Time::now().toSec(); 

		if (read(fd_port, &cp, 1) > 0)
		{
		      /* Check if a message could be decoded, return the message in case yes */
		      msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
		      if (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count)
		      {
			  printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
			  unsigned char v = cp;
			  fprintf(stderr, "%02x ", v);
		      }

		      lastStatus = status;
		}
		else
		{
			// Not sure what should go in here
		}

		/* Decode mavlink received msgs */
		if (msgReceived)
		{

			ROS_INFO("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid,
				 message.compid);

			mavlink_ros::Mavlink rosmavlink_msg;

			rosmavlink_msg.len = message.len;
			rosmavlink_msg.seq = message.seq;
			rosmavlink_msg.sysid = message.sysid;
			rosmavlink_msg.compid = message.compid;
			rosmavlink_msg.msgid = message.msgid;

			sysid = message.sysid;

			mavlink_pub.publish(rosmavlink_msg);

			switch (message.msgid)
			{
			
				case MAVLINK_MSG_ID_ATTITUDE:
				{	

				 	 mavlink_attitude_t att;
					mavlink_msg_attitude_decode(&message, &att);
					
					sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu);
		
					px4_msg.attitude[0] = att.roll;
					px4_msg.attitude[1] = att.pitch;
					px4_msg.attitude[2] = att.yaw;

					px4_msg.attitude_rate[0] = att.rollspeed;
					px4_msg.attitude_rate[1] = att.pitchspeed;
					px4_msg.attitude_rate[2] = att.yawspeed;

					global_roll = att.roll;
					global_pitch = att.pitch;
					global_yaw = -att.yaw; //need this to make sure yaw on vehicle is as defined in vicon callback (negative) 

					px4_pub.publish(px4_msg);
				}
				break;

				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					mavlink_highres_imu_t imu_raw2;

					/* decode message */
					mavlink_msg_highres_imu_decode(&message, &imu_raw2);

	
					/* Stuff up data into px4_ros node */
					px4_msg.linear_accel[0] = imu_raw2.xacc;
					px4_msg.linear_accel[1] = -imu_raw2.yacc;
					px4_msg.linear_accel[2] = -imu_raw2.zacc;
					px4_pub.publish(px4_msg);

					std_msgs::Header header;
					header.stamp = ros::Time::now();
					header.seq = imu_raw.time_usec / 1000;
					header.frame_id = frame_id;

					if (imu_raw_pub.getNumSubscribers() > 0)
					{

						sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu);

						imu_msg->angular_velocity.x = imu_raw.xgyro;
						imu_msg->angular_velocity.y = -imu_raw.ygyro;
						imu_msg->angular_velocity.z = -imu_raw.zgyro;

						imu_msg->linear_acceleration.x = imu_raw.xacc;
						imu_msg->linear_acceleration.y = -imu_raw.yacc;
						imu_msg->linear_acceleration.z = -imu_raw.zacc;

						for (sensor_msgs::Imu::_angular_velocity_covariance_type::iterator it =
						imu_msg->angular_velocity_covariance.begin(); it != imu_msg->angular_velocity_covariance.end(); ++it)
						*it = 0;

						for (sensor_msgs::Imu::_linear_acceleration_covariance_type::iterator it =
						imu_msg->linear_acceleration_covariance.begin(); it != imu_msg->linear_acceleration_covariance.end();
						++it)
						*it = 0;

						imu_msg->orientation_covariance[0] = -1;

						imu_msg->header = header;

						imu_raw_pub.publish(imu_msg);

						px4_pub.publish(px4_msg);
					}

				}
			  	break;

				/* TODO create our own local velocity: This is velocity in body fixed frame */
				case MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE:
				{
					mavlink_vision_speed_estimate_t onboard_vel;
					mavlink_msg_vision_speed_estimate_decode(&message, &onboard_vel);
					global_vx = onboard_vel.x;
					global_vy = onboard_vel.y;
					global_vz = onboard_vel.z;

				}
				break;

				/* Read Home Position */
				case MAVLINK_MSG_ID_HOME_POSITION:
				{
					mavlink_msg_home_position_decode(&message, &vehicle_home_pos);
				}
				break;

				/* Read system status */
				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					//mavlink_sys_status_decode(&message, &vehicle_onboard_state);
				}
				break;

				/* Read GPS data */
				case MAVLINK_MSG_ID_GPS_RAW_INT:
				{
					//mavlink_gps_raw_int_t gps_data;
					mavlink_msg_gps_raw_int_decode(&message, &gps_data);
				}
				break;

				/* Read attitude setpoint */
				case MAVLINK_MSG_ID_ATTITUDE_TARGET:
				{
					mavlink_msg_attitude_target_decode(&message, &quad_control_target);
				}
				break;

				/* Read vehicle state */
				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					mavlink_msg_heartbeat_decode(&message, &heart_beat);
					//ROS_INFO("state %d %d %d\n", heart_beat.base_mode, heart_beat.custom_mode, heart_beat.system_status);
				}
				break;

				case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
				{
					mavlink_rc_channels_raw_t rc;
					mavlink_msg_rc_channels_raw_decode(&message, &rc);
					RC[0] = rc.chan1_raw;
					RC[1] = rc.chan2_raw;
					RC[2] = rc.chan3_raw;
					RC[3] = rc.chan4_raw;
					RC[4] = rc.chan5_raw;
					RC[5] = rc.chan6_raw;
					RC[6] = rc.chan7_raw;
					RC[7] = rc.chan8_raw;

					px4_pub.publish(px4_msg);
				}
				break;

			}
			
		}
	}
	return NULL;
}


int main(int argc, char **argv)
{
	GError *error = NULL;
	int* fd_ptr = &fd_port;
	bool setup;
	GThread* serial_thread;
	GError* err;

	ros::init(argc, argv, "mavlink_ros_serial");

	/* Read configuration file */
	read_config();
	baud = config.baud_rate;
	port = config.dev_address;

	/* Open and configure serial port */
	printf("Trying to connect to %s.. ", port.c_str());

	fd_port = open_port(port);
			

	if (fd_port == -1)
	{
		fprintf(stderr, "failure, could not open port.\n");
		exit(EXIT_FAILURE);
	} else
	{
		printf("success.\n");
	}
	
	printf("Trying to configure %s.. ", port.c_str());

	setup = setup_port(fd_port, baud, 8, 1, false, false);
	if (!setup)
	{
		fprintf(stderr, "failure, could not configure port.\n");
		exit(EXIT_FAILURE);
	} else
	{
		printf("success.\n");
	}

	// SETUP ROS
	ros::NodeHandle mavlink_nh("/mavlink"); 

	ros::NodeHandle n;
	tv0 = ros::Time::now().toSec();
	ros::Subscriber vicon_sub = n.subscribe(config.vehicle_name, 1, ViconCallback); 
	ros::Subscriber lmpc_m_sub = n.subscribe<lmpc_v1::lmpc_V1>("lmpc_V1", 10, get_trajectory);

	px4_pub = n.advertise<px4_ros::px4_ros>("px4_ros", 100);

	/* These two nodes are for future use */
	ros::NodeHandle nh("fcu");
	imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 10);

	ros::NodeHandle raw_nh("fcu/raw");
	imu_raw_pub = nh.advertise<sensor_msgs::Imu>("imu", 10);

	vec_state_pub  	= n.advertise<std_msgs::Float64MultiArray>("vehicle_state", 10);
	vec_vic_gps_pub = n.advertise<std_msgs::Float64MultiArray>("vicon_gps_state", 10);

	/* Only initialize g thread if not already done */
	if (!g_thread_supported())
	{
		g_thread_init(NULL);		
	}

	/* Run indefinitely while the ROS and serial threads handle the data */
	printf("\nREADY, waiting for serial/ROS data.\n");

	if ((serial_thread = g_thread_create((GThreadFunc)serial_wait, (void *)fd_ptr, TRUE, &err)) == NULL)
	{
		printf("Failed to create serial handling thread: %s!!\n", err->message);
		g_error_free(err);
	}
	
	if ((serial_thread = g_thread_create((GThreadFunc)write_to_mavlink, (void *)fd_ptr, TRUE, &err)) == NULL)
	{
		printf("Failed to create serial handling thread: %s!!\n", err->message);
		g_error_free(err);
	}

	int noErrors = 0;

	if (fd_port == -1 || fd_port == 0)
	{
		fprintf(stderr, "Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", port.c_str(), baud);
		exit(EXIT_FAILURE);
	}
	else
	{
		fprintf(stderr, "\nConnected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", port.c_str(), baud);
	}


	if (fd_port == -1 || fd_port == 0)
	{
		exit(noErrors);
	}


	/**
	* Now pump callbacks (execute mavlinkCallback) until CTRL-c is pressed
	*/
	ros::spin();

	close_port(fd_port);

	return 0;
}
