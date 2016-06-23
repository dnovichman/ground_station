/****************************************************************************************
*											*
*		Ground station logging system						*
*	Logs data from PX4 and desired setpoints and velocity measured on the ground	*
*											*
*	Develeoper: Moses Bangura							*
*			moses.bangura@anu.edu.au					*
*			dnovichman@hotmail.com						*
*			Australian National University					*
****************************************************************************************/

/****************************************************************************************
*			TODO								*
*	1) Fix velocity estimation based on code on quad				*
*	2) The velocities should be both v and V and log data				*
*	3) Create a second logging system similar to PX4 FlightPlot			*
****************************************************************************************/

#include "log_data.h"
#include "../../gs_source/ground_station.cpp"	

char newf[100];

int my2_strcmp(char *s1, char *s2)
{     
	//int i = 0;
	int len = (int)strlen(s1); //sizeof(s1)/sizeof(s1[0]);
	int len2 = (int)strlen(s2); //sizeof(s2)/sizeof(s2[0]);

	if (len != len2+1)
		return 1;

	
	for (int i=0; i < len-1; i++)
	{
		if (s1[i] == s2[i])
		{
			
		}
		else
		{
			return 1;
		}
	}
	return 0;
}

/**
  * Create log file Name 
*/
void create_log_file(void)
{
	time ( &rawtime );
	timeinfo = localtime ( &rawtime );	

	int yy = my2_strcmp(config.log_file_name,"log.txt");

	if (yy == 0)
	{
		if (timeinfo->tm_sec<55)
		{
			sprintf ( newf ,"/%4d_%02d_%02d_%02d%02d.txt",  timeinfo->tm_year+1900, timeinfo->tm_mon+1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min);
		}
		else
		{
			sprintf ( newf ,"/%4d_%02d_%02d_%02d%02d.txt",  timeinfo->tm_year+1900, timeinfo->tm_mon+1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min+1);
		}
	} else
	{
		char newf2[100];
		strcpy(newf2,config.log_file_name);
		char slash[20];
		strcpy(slash,"/");
		char* ss2 = strcat(slash,newf2);

		strncpy(newf, ss2, sizeof newf - 1);
		int ind = strlen(newf)-1;
		newf[ind] = '\0';
		
	}

	char* ss = strcat(log_tmp_dir,newf);
	strncpy(fileName, ss, sizeof fileName - 1);
	
	fileName[99] = '\0';

	fd_log = fopen(fileName, "w");
}

/**
  * Decode vicon data
  * TODO do velocity estimation correctly.. see quad code
*/
void decode_vicon(const geometry_msgs::TransformStamped::ConstPtr& msg)
{	
	tv = ros::Time::now().toSec();
    	dtv = tv - ptv;

	double yaw, pitch, roll;
	/* We need minuses to bring to the first quadrant */
	vicon_pos[0] = -msg->transform.translation.y;
	vicon_pos[1] = -msg->transform.translation.x;
	vicon_pos[2] = -msg->transform.translation.z;
	x_tmp = vicon_pos[0];
	y_tmp = vicon_pos[1];
	z_tmp = vicon_pos[2]; 


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

	vicon_angles[0] = roll;
	vicon_angles[1] = pitch;
	vicon_angles[2] = yaw;

	vicon_vel[0] = vx;
	vicon_vel[1] = vy;
	vicon_vel[2] = vz;
	
}

/**
  * decode PX4FMU/Pixhawk data
*/
void decode_px4_data(px4_ros::px4_ros msg)
{
	RC[0] = msg.RC[0];
	RC[1] = msg.RC[1];
	RC[2] = msg.RC[2];
	RC[3] = msg.RC[3];

	attitude[0] = msg.attitude[0];
	attitude[1] = msg.attitude[1];
	attitude[2] = msg.attitude[2];

	attitude_rate[0] = msg.attitude_rate[0];
	attitude_rate[1] = msg.attitude_rate[1];
	attitude_rate[2] = msg.attitude_rate[2];

	acceleration[0] = msg.linear_accel[0];
	acceleration[1] = msg.linear_accel[1];
	acceleration[2] = msg.linear_accel[2];
}

/**
  * Decode desired trajectory data 
*/
void get_trajectory(lmpc_v1::lmpc_V1 msg)
{
	for (int i =0; i<9; i++)
	{
		trajectory[i] = msg.desired_state[i];
	}
	des_yaw = msg.desired_yaw;
}


/**
  * Write to File 
*/
void write_file(void)
{
	struct timeval tv;
	gettimeofday(&tv, NULL);

	/* time, attitude[3], attitude_rate[3],linear_accel[3],RC[4], vicon_pos[3], vicon_attitude */
	fprintf(fd_log, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f, %f,%f, %f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",t2, attitude[0], attitude[1], attitude[2], attitude_rate[0], attitude_rate[1], attitude_rate[2], acceleration[0],acceleration[1],acceleration[2],RC[0], RC[1], RC[2], RC[3],vicon_pos[0],vicon_pos[1],vicon_pos[2],vicon_angles[0],vicon_angles[1],vicon_angles[2],vicon_vel[0],vicon_vel[1],vicon_vel[2],trajectory[0], trajectory[1], trajectory[2], trajectory[3],trajectory[4], trajectory[5], trajectory[6], trajectory[7], trajectory[8],des_yaw);

}

int main(int argc, char **argv)
{
	/* Read configuration file */
	read_config();

	ros::init(argc, argv, "log_data");
	ros::NodeHandle n;
	px4_ros::px4_ros px4_msg;

	ros::Subscriber px4_sub = n.subscribe<px4_ros::px4_ros>("px4_ros", 100, decode_px4_data);
	ros::Subscriber vicon_sub = n.subscribe(config.vehicle_name, 1, decode_vicon); 
	ros::Subscriber lmpc_m_sub = n.subscribe<lmpc_v1::lmpc_V1>("lmpc_V1", 10, get_trajectory);

	/* check and create log directory */
	char* ss = strcat(config.install_path,log_dir);
	strncpy(log_tmp_dir, ss, sizeof fileName - 1);
	log_tmp_dir[99] = '\0';

	if (mkdir(log_tmp_dir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) > -1)
		ROS_INFO("Directory %s was created",log_tmp_dir);
	else
		ROS_INFO("Directory %s exists",log_tmp_dir);

	/* Create Log File */
	create_log_file();
	int count = 0;
	ros::Rate loop_rate(100);

	/* Continuously write file */
	while (ros::ok())
	{
		write_file();

		// %EndTag(PUBLISH)%

		// %Tag(SPINONCE)%
		ros::spinOnce();
		// %EndTag(SPINONCE)%

		// %Tag(RATE_SLEEP)%
		loop_rate.sleep();
		// %EndTag(RATE_SLEEP)%
		++count;
	}
	fclose(fd_log);
}
