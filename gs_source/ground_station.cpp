#include "ground_station.h"

struct system_config config;

/* Remove white spaces after reading a string */
void remove_space(char *in, char *out)
{
	int str = (int)strlen(in);
	for (int j = 0; j < str-1; j++ )
    	{
	if ((in[j] != 44) || (in[j] != 0x0A) || (in[j] != 0x20) || (in[j] != 0x2c) || (in[j] != 0x2b))
	{
	 	out[j] = in[j];
	}
	}
}

/* Open Config file */
void read_config(void)
{	
        FILE *file = fopen (config_file, "r");
	char pp[MAXBUF];

	if (file != NULL)
        {
                char line[MAXBUF];
                int i = 0;

                while(fgets(line, sizeof(line), file) != NULL)
                {
                        char *cfline;
                        cfline = strstr((char *)line,DELIM);
                        cfline = cfline + strlen(DELIM);
   			if (i == 0)
			{
                                memcpy(config.install_path,cfline,strlen(cfline)-1);
				remove_space(&config.install_path[0],&config.install_path[0]);
                                printf("%s\n",config.install_path);
                        } else if (i == 1)
			{
                                memcpy(config.vehicle_name,cfline,strlen(cfline)-1);
				remove_space(&config.vehicle_name[0],&config.vehicle_name[0]);
                                printf("%s\n",config.vehicle_name);
                        } else if (i == 2)
			{
                                memcpy(config.log_file_name,cfline,strlen(cfline));
                        } else if (i == 3)
			{
                                memcpy(config.waypoints_file,cfline,strlen(cfline)-1);
				remove_space(&config.waypoints_file[0],&config.waypoints_file[0]);
				printf("%s\n",config.waypoints_file);
                        } else if (i == 4)
			{			
                                memcpy(pp,cfline,strlen(cfline));
				config.port = atoi(pp);
                                printf("%d\n",config.port);
                        } else if (i == 5)
			{
                                memcpy(config.dev_address,cfline,strlen(cfline)-1);   
				remove_space(&config.dev_address[0],&config.dev_address[0]);
				printf("%s\n",config.dev_address);                             
                        } else if (i == 6)
			{
                                memcpy(pp,cfline,strlen(cfline)); 
				config.baud_rate = atoi(pp);  
				printf("%d\n",config.baud_rate);                             
                        } else if (i == 7)
			{
                                memcpy(pp,cfline,strlen(cfline)); 
				config.max_x = atof(pp);  
				printf("%f\n",config.max_x);                             
                        } else if (i == 8)
			{
                                memcpy(pp,cfline,strlen(cfline)); 
				config.max_y = atof(pp);  
				printf("%f\n",config.max_y);                             
                        } else if (i == 9)
			{
                                memcpy(pp,cfline,strlen(cfline)); 
				config.max_z = atof(pp);  
				printf("%f\n",config.max_z);                             
                        } else if (i == 10)
			{
                                memcpy(pp,cfline,strlen(cfline)); 
				config.max_vxy = atof(pp);  
				printf("%f\n",config.max_vxy);                             
                        } else if (i == 11)
			{
                                memcpy(pp,cfline,strlen(cfline)); 
				config.max_vz = atof(pp);  
				printf("%f\n",config.max_vz);                             
                        }else if (i == 12)
			{
                                memcpy(pp,cfline,strlen(cfline)); 
				config.waypoint_bound = atof(pp);  
				printf("%f\n",config.waypoint_bound);                             
                        } else if (i == 13)
			{
                                memcpy(pp,cfline,strlen(cfline)); 
				config.yaw_bound = atof(pp);  
				printf("%f\n",config.yaw_bound);                             
                        } 
                        i++;
                } 
        } else
	{
		printf("Failed to open config file\n");
	}
       
        fclose(file);   
}

/** Function to fill out waypoints array */
void fill_wpt(char *s)
{
	int i, j = 0, h_count = 0;
	char curval, data_ind = 0;

	wpts[j][data_ind] = atof(&s[0]);

	for (i=0; i<(int)strlen(s); i++)
	{
		curval = s[i];
		if (((curval == 35) || (curval == 0x23)) && (h_count == 0))
		{
			if ((s[i+1] == 87) || (s[i+1] == 0x57))
			{
				printf("Using WSGS\n");
				h_count = h_count +1;
			} else if ((s[i+1] == 67) || (s[i+1] == 0x43))
			{
				printf("Using cartesian\n"); //WSGS or CART
			// TODO is first letter is cartesian then CART
			}
				
		}

		if ((curval == 35) || (curval == 0x23))
		{
			if (i > 0 && h_count < 2) 
			{
				//trans_bound =atof(&s[i+1]);
				//if (trans_bound < config.waypoint_bound)
				//	config.waypoint_bound = trans_bound;
				//printf("ao %f %x %u\n",trans_bound,curval,i); //this is the bounds
				h_count++;
			}
			i = i+4;
			wpts[j][data_ind] = atof(&s[i+1]);
		}

		if (h_count >= 2)
		{
			if (curval == 0x0A)
			{
				//printf("End of Line\n");
				data_ind = 0;
				j++;
				wpts[j][data_ind] = atof(&s[i+1]);
				//i=i+1;
			}

			if (((curval == 44) || (curval == 0x20)) && (curval != 0x0A)) //44 is comma and 0x20 is white space 0x0A 0x76
			{
				data_ind++;
				wpts[j][data_ind] = atof(&s[i+1]);
				//printf("wpts %f %d %d\n",wpts[j][data_ind],i,strlen(s));
			}
		}				
	}

	for (i=0; i<4; i++)
	{
		printf("%f %f %f %f\n",wpts[i][0],wpts[i][1],wpts[i][2],wpts[i][3]);
	}
	
}

void read_wpt_file(void)
{
	int v;

	fd = fopen(config.waypoints_file, "r"); //fileName config.waypoints_file
	if (fd)
	{
		v = fread(sfile, 1, 200, fd);
		if (v)
		{
        		fill_wpt(sfile);
		} else
		{
			printf("Waypoint file invalid\n");
		}
	}
	fclose(fd);
}

void check_constraints(float* des, float* outp)
{
	//memset(outp, 0, sizeof outp);

	/* Check x */
	if (des[0] > config.max_x)
		outp[0] = config.max_x;
	else if (des[0] < -config.max_x)
		outp[0] = -config.max_x;
	else
		outp[0] = des[0];

	/* Check y */
	if (des[1] > config.max_y)
		outp[1] = config.max_y;
	else if (des[1] < -config.max_y)
		outp[1] = -config.max_y;
	else
		outp[1] = des[1];

	/* Check z */
	if (-des[2] > config.max_z)
		outp[2] = -config.max_z;
	else if (-des[2] < 0.0f)
		outp[2] = 0.0f;
	else
		outp[2] = des[2];

	/* Check vx */
	if (des[3] > config.max_vxy)
		outp[3] = config.max_vxy;
	else if (des[3] < -config.max_vxy)
		outp[3] = -config.max_vxy;
	else
		outp[3] = des[3];

	/* Check vy */
	if (des[4] > config.max_vxy)
		outp[4] = config.max_vxy;
	else if (des[4] < -config.max_vxy)
		outp[4] = -config.max_vxy;
	else
		outp[4] = des[4];

	/* Check vz */
	if (des[5] > config.max_vz)
		outp[5] = config.max_vz;
	else if (des[5] < -config.max_vz)
		outp[5] = -config.max_vz;
	else
		outp[5] = des[5];

	outp[6] = 0.0f;
	outp[7] = 0.0f;
	outp[8] = 0.0f;
}
