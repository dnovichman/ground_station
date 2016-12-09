#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#include <GL/glut.h>
#include <pthread.h>

#define PI 3.1415926

using namespace std;

char *window_title = "Filter Indicator";

int vicon = 0;
int gps_sat = 0;
int bff_filter = 0;
float mag_vel = 0.0f;

typedef enum {
   MODE_BITMAP,
   MODE_STROKE
} mode_type;

static mode_type mode;
static int font_index;

void* bitmap_fonts[7] = {
      GLUT_BITMAP_9_BY_15,
      GLUT_BITMAP_8_BY_13,
      GLUT_BITMAP_TIMES_ROMAN_10,
      GLUT_BITMAP_TIMES_ROMAN_24,
      GLUT_BITMAP_HELVETICA_10,
      GLUT_BITMAP_HELVETICA_12,
      GLUT_BITMAP_HELVETICA_18     
   };

   char* bitmap_font_names[7] = {
      "GLUT_BITMAP_9_BY_15",
      "GLUT_BITMAP_8_BY_13",
      "GLUT_BITMAP_TIMES_ROMAN_10",
      "GLUT_BITMAP_TIMES_ROMAN_24",
      "GLUT_BITMAP_HELVETICA_10",
      "GLUT_BITMAP_HELVETICA_12",
      "GLUT_BITMAP_HELVETICA_18"     
   };

   void* stroke_fonts[2] = {
      GLUT_STROKE_ROMAN,
      GLUT_STROKE_MONO_ROMAN
   };

   char* stroke_font_names[2] = {
      "GLUT_STROKE_ROMAN",
      "GLUT_STROKE_MONO_ROMAN"
   };


struct tharg
{
	int th_argc;
	char** th_argv;	
}th_arg;

void
print_bitmap_string(void* font, char* s)
{
   if (s && strlen(s)) {
      while (*s) {
         glutBitmapCharacter(font, *s);
         s++;
      }
   }
}

void
print_stroke_string(void* font, char* s)
{
   if (s && strlen(s)) {
      while (*s) {
         glutStrokeCharacter(font, *s);
         s++;
      }
   }
}
void 
my_init()
{
   mode = MODE_BITMAP;
   font_index = 0;
}
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

void handleKeypress(unsigned char key, int x, int y) {
	switch (key) {
		case 27: //Escape key
			exit(0); //quit the loop
	}
}

void
gluPerspective(GLdouble fovy, GLdouble aspect, GLdouble zNear, GLdouble zFar)
{
   GLdouble xmin, xmax, ymin, ymax;

   ymax = zNear * tan(fovy * M_PI / 360.0);
   ymin = -ymax;
   xmin = ymin * aspect;
   xmax = ymax * aspect;


   glFrustum(xmin, xmax, ymin, ymax, zNear, zFar);
}



//Initializes 3D rendering
void initRendering() {
	glEnable(GL_DEPTH_TEST);
}

//Called when the window is resized
void handleResize(int w, int h) {

   GLdouble aspect;	
glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
/* Make the world and window coordinates coincide so that 1.0 in */
   /* model space equals one pixel in window space.                 */
//   glScaled(aspect, aspect, 1.0);

   /* Now determine where to draw things. */
  // glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluPerspective(90.0, (double)w / (double)h, 1.0, 1000.0);
}


//Draws the 3D scene
void drawScene() {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	glMatrixMode(GL_MODELVIEW); //Switch to the drawing perspective
	glLoadIdentity(); //Reset the drawing perspective
//	glRotatef(-_cameraAngle, 0.0f, 1.0f, 0.0f); //Rotate the camera
	glTranslatef(0.0f, 0.0f, -200.0f); //Move forward 5 units

	glColor3f(0.0, 0.0, 0.0);
    glRasterPos3f(-217, 43,0);
    print_bitmap_string(bitmap_fonts[2], "A");
    glRasterPos3f(-207, 41,0);
    print_bitmap_string(bitmap_fonts[3], "^");
    glRasterPos3f(-210, 30,0);
    print_bitmap_string(bitmap_fonts[3], "V");


	glColor3f(1.0, 1.0, 1.0);
    glRasterPos3f( -12, 43,0);
    print_bitmap_string(bitmap_fonts[2], "B");

    glRasterPos3f(-2, 41,0);
    print_bitmap_string(bitmap_fonts[3], "^");

    glRasterPos3f( -5, 30,0);
    print_bitmap_string(bitmap_fonts[3], "V");


	glBegin(GL_POLYGON);
	switch(vicon){
		case 0: {
					if(gps_sat < 5){
						glColor3f(1,0,0);
					}else{
						glColor3f(0,float(gps_sat-5)/10 + 0.5,0);
					}
					break;
				}
		case 1: {
					glColor3f(0,1,0);
					break;
				}
		default: {
					glColor3f(1,0,0);		
				}
	}

	glVertex2f(-300, 200);
	glVertex2f(-100, 200);
	glVertex2f(-100, -200);
	glVertex2f(-300, -200);	
	glEnd();

// BFF filter
	glBegin(GL_POLYGON);
	switch(bff_filter){
		case 0: {
					glColor3f(1,0,0);
					break;
				}
		case 1: {
					glColor3f(0,0,1);
					break;
				}
		default: {
					glColor3f(1,0,0);		
				}
	}

	glVertex2f(-100, 200);
	glVertex2f(100, 200);
	glVertex2f(100, -200);
	glVertex2f(-100, -200);	
	glEnd();


// VAA filter
	if (vicon&&bff_filter){
		glColor3f(0,0,0);
	    glRasterPos3f(173, 41,0);
	    print_bitmap_string(bitmap_fonts[3], "^");
	    glRasterPos3f(170, 30,0);
	    print_bitmap_string(bitmap_fonts[3], "V =");
		glRasterPos3f(213, 43,0);
    	print_bitmap_string(bitmap_fonts[2], "A");
    	glRasterPos3f(223, 41,0);
    	print_bitmap_string(bitmap_fonts[3], "^");
    	glRasterPos3f(220, 30,0);
	    print_bitmap_string(bitmap_fonts[3], "V");
	}else if(vicon){
		glColor3f(0,0,0);
	    glRasterPos3f(173, 41,0);
	    print_bitmap_string(bitmap_fonts[3], "^");
	    glRasterPos3f(170, 30,0);
	    print_bitmap_string(bitmap_fonts[3], "V =");
		glRasterPos3f(213, 43,0);
	    print_bitmap_string(bitmap_fonts[2], "A");
	    glRasterPos3f(223, 41,0);
	    print_bitmap_string(bitmap_fonts[3], "^");
	    glRasterPos3f(220, 30,0);
	    print_bitmap_string(bitmap_fonts[3], "V");
	}else if(bff_filter){
		glColor3f(1,1,1);
	    glRasterPos3f(173, 41,0);
	    print_bitmap_string(bitmap_fonts[3], "^");
	    glRasterPos3f(170, 30,0);
	    print_bitmap_string(bitmap_fonts[3], "V =");
		glRasterPos3f(213, 43,0);
	    print_bitmap_string(bitmap_fonts[2], "B");
	    glRasterPos3f(223, 41,0);
	    print_bitmap_string(bitmap_fonts[3], "^");
	    glRasterPos3f(220, 30,0);
	    print_bitmap_string(bitmap_fonts[3], "V");
	}else{
		glColor3f(0.0, 0.0, 0.0);
	    glRasterPos3f(173, 41,0);
	    print_bitmap_string(bitmap_fonts[3], "^");
	    glRasterPos3f(170, 30,0);
	    print_bitmap_string(bitmap_fonts[3], "|V| =");
	    glRasterPos3f(215, 30,0); 
	    char output[50];
	    snprintf(output, 50, " %2.1f", mag_vel);

	    print_bitmap_string(bitmap_fonts[3], output);
	} 

	glBegin(GL_POLYGON);
	if (vicon&&bff_filter){
		glColor3f(1,0.6,0);
	}else if(vicon){
		glColor3f(0,1,0);
	}else if(bff_filter){
		glColor3f(0,0,1);
	}else{
		glColor3f(1,0,0);
	}

	glVertex2f(100, 200);
	glVertex2f(300, 200);
	glVertex2f(300, -200);
	glVertex2f(100, -200);	
	glEnd();


	glutSwapBuffers();
}



void update(int value) {

	glutPostRedisplay(); //Tell GLUT that the display has changed

	//Tell GLUT to call update again in 25 milliseconds
	glutTimerFunc(40, update, 0);
}


void* test(void* threadarg) {
        struct tharg *arg;
        arg=(struct tharg *) threadarg;
	
        //Initialize GLUT

	glutInit(&(arg->th_argc), arg->th_argv);
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
        glutInitWindowSize(600, 400);

        //Create the window
        glutCreateWindow(window_title);
        initRendering();

        //Set handler functions
        glutDisplayFunc(drawScene);
	glutKeyboardFunc(handleKeypress);
        glutReshapeFunc(handleResize);

	glutTimerFunc(40, update, 0); //Add a timer

        glutMainLoop();
        return 0;
}

void GPSCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	vicon = (int)msg->data[0];//0; 1
	gps_sat = (int)msg->data[1];//
	bff_filter = (int)msg->data[2];//0;1
	if (gps_sat>=5){
		vicon = 1;
	}else{
		vicon = 0;
	}
	printf("vicon: %d, GPS_sat: %d, bff_filter: %d;\n", vicon, gps_sat, bff_filter);	
}

void VehicleStateCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	float vx = msg->data[6];
	float vy = msg->data[7];
	float vz = msg->data[8];
	mag_vel = sqrt(vx*vx + vy*vy + vz*vz);
}

int main(int argc, char **argv)
{
	th_arg.th_argc=argc;
	th_arg.th_argv=argv;
	pthread_t thread;

   int rc=pthread_create(&thread, NULL, test, (void*) &th_arg);  

  ros::init(argc, argv, "GPS_indicator");
  my_init();

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("vicon_gps_state", 1, GPSCallback); 
  ros::Subscriber state_sub = n.subscribe("vehicle_state",1,VehicleStateCallback);

  ros::spin();
 
        return 0;
}
