

#include<stdbool.h> //for bool
//#include<unistd.h> //for usleep
//#include <math.h>

#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

//#include "limits.h" // for INT_MAX

//double joint_torque[4][200000];

// char s2d[20];
// char *ptr;
// long long int iteration =0;
// long long int end_iteration =0;
// double qinit[2] = {0,1.25};

//related to writing data to a file
double qD_dot[4];

// double joint_acc[4];
FILE *fid1;
//, *fid2, *fid3, *fid4, *fid5;
//FILE *fid11;
//FILE *qD_dot;	//desired joint angular velocity values
int loop_index = 0;
const int data_frequency = 1; //frequency at which data is written to a file

// C:\Users\vssc\Documents\Received Files\mujoco200_win64\mujoco200_win64\myproject\space_man
char filename[] = "../myproject/space_man/hello.xml";
char path[] = "../myproject/space_man/";
char xmlfile[] = "model.xml";
char datafile1[] = "../myproject/space_man/sensed_data.csv";
// char datafile2[] = "../myproject/space_man/joint1_torques_ran.txt";
// char datafile3[] = "../myproject/space_man/joint2_torques_ran.txt";
// char datafile4[] = "../myproject/space_man/joint3_torques_ran.txt";
// char datafile5[] = "../myproject/space_man/joint4_torques_ran.txt";
//char datafile6[] = "../myproject/space_man/jt.txt";

//char qD_dot_path[] = "../myproject/space_man/ifile_qDdot.txt";

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// holders of one step history of time and position to calculate dertivatives
mjtNum position_history = 0;
mjtNum previous_time = 0;

// controller related variables
float_t ctrl_update_freq = 100;
mjtNum last_update = 0.0;
mjtNum ctrl;
mjtNum qfrc_applied;
mjtNum xfrc_applied;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

//****************************
//This function is called once and is used to get the headers
// void init_save_data()
// {
  // //write name of the variable here (header)
   // fprintf(fid1,"t ");
   // fprintf(fid1,"base_x  base_y  base_z  base_quat1  base_quat2  base_quat3  base_quat4 base_vel_x base_vel_y  base_vel_z  base_pitch  base_yaw  base_roll base_linacc_x base_linacc_y base_linacc_z base_angacc_x base_angacc_y base_angacc_z fx_eef  fy_eef  fz_eef  Tx_eef  Ty_eef  Tz_eef  EEF_x EEF_y EEF_z EEF_quat1 EEF_quat2 EEF_quat3 EEF_quat4 q1  q2  q3  q4, q1_dot  q2_dot  q3_dot  q4_dot");

   // //Don't remove the newline
   // fprintf(fid1,"\n");
// }

//***************************
//This function is called at a set frequency, put data here
void save_data(const mjModel* m, mjData* d)
{
  //data here should correspond to headers in init_save_data()
  //seperate data by a space %f followed by space
  
  int i;
  fprintf(fid1,"%f,\t",d->time);
  for(i=0;i<36;i++)
  fprintf(fid1,"%f,\t",d->sensordata[i]);
  // for(i=1;i<5;i++)
  // fprintf(fid,"%f, %f,",d->qpos[i], d->qvel[i]);
  //Don't remove the newline
  fprintf(fid1,"\n");
}

// void load_data()
// {
	
	// fid2 = fopen(datafile2, "r");
	// fid3 = fopen(datafile3, "r");
	// fid4 = fopen(datafile4, "r");
	// fid5 = fopen(datafile5, "r");
// //	fid11 = fopen(datafile6, "w");
    // long long int i=0;
    // while(fscanf(fid2, "%s", s2d)==1) {
		// joint_torque[0][i] = atof(s2d);	
		// i++; end_iteration++;
		
    // }
	// i=0;
	// fclose(fid2);
	// while(fscanf(fid3, "%s", s2d)==1) {
		// joint_torque[1][i] = atof(s2d);	
		// i++;	
    // }
	// i=0;
	// fclose(fid3);
	// while(fscanf(fid4, "%s", s2d)==1) {
		// joint_torque[2][i] = atof(s2d);	
		// i++;	
    // }
	// i=0;
	// fclose(fid4);
	// while(fscanf(fid5, "%s", s2d)==1) {
		// joint_torque[3][i] = atof(s2d);	
// //		fprintf(fid11,"%f\n",joint_torque[3][i]);
		// i++;	
    // }
	// fclose(fid5);
	
	
// }

//*************************************

// main function
int main(int argc, const char** argv)
{

    // activate software
    mj_activate("mjkey.txt");
    char xmlpath[100]= "";
    char datapath[100]= "";

    strcat(xmlpath,path);
    strcat(xmlpath,xmlfile);

//    strcat(datapath,path);
//    strcat(datapath,datafile);

	//Reading desired joint angular velocity values from input file
	

    // load and compile model
    char error[1000] = "Could not load binary model";
    
	// loading torques values
	//load_data();
	
	
	
    // check command-line arguments
    if( argc<2 )
        m = mj_loadXML(filename, 0, error, 1000);

    else
        if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);


    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    double arr_view[] = {89.608063, -11.588379, 5, 0.000000, 0.000000, 1.000000};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];
    
    // use the first while condition if you want to simulate for a period.
    //mjcb_control= mycontroller;
    fid1 = fopen(datafile1,"w");
	
    //init_save_data();
    //mycontroller(m,d);
    while( !glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
	
        while( d->time - simstart < 1.0/30.0 )
        {
            mj_step(m, d);
			
			
			
			if ((d->time)<10) { qD_dot[0]=0.1; qD_dot[1]=0; qD_dot[2]=0; qD_dot[3]=0;}
			else if ((d->time)<12) { qD_dot[0]=0; qD_dot[1]=0; qD_dot[2]=0; qD_dot[3]=0;}
			else if ((d->time)<32) { qD_dot[0]=-0.1; qD_dot[1]=0; qD_dot[2]=0; qD_dot[3]=0;}
			else if ((d->time)<34) { qD_dot[0]=0; qD_dot[1]=0; qD_dot[2]=0; qD_dot[3]=0;}
			else if ((d->time)<44) { qD_dot[0]=0.1; qD_dot[1]=0; qD_dot[2]=0; qD_dot[3]=0;}
			else if ((d->time)<46) { qD_dot[0]=0; qD_dot[1]=0; qD_dot[2]=0; qD_dot[3]=0;}
			else if ((d->time)<56) { qD_dot[0]=0; qD_dot[1]=0.1; qD_dot[2]=0; qD_dot[3]=0;}
			else if ((d->time)<58) { qD_dot[0]=0; qD_dot[1]=0; qD_dot[2]=0; qD_dot[3]=0;}
			else if ((d->time)<78) { qD_dot[0]=0; qD_dot[1]=-0.1; qD_dot[2]=0; qD_dot[3]=0;}
			else if ((d->time)<80) { qD_dot[0]=0; qD_dot[1]=0; qD_dot[2]=0; qD_dot[3]=0;}
			else if ((d->time)<90) { qD_dot[0]=0; qD_dot[1]=0.1; qD_dot[2]=0; qD_dot[3]=0;}
			else if ((d->time)<92) { qD_dot[0]=0; qD_dot[1]=0; qD_dot[2]=0; qD_dot[3]=0;}
			else if ((d->time)<102) { qD_dot[0]=0; qD_dot[1]=0; qD_dot[2]=0.1; qD_dot[3]=0;}
			else if ((d->time)<104) { qD_dot[0]=0; qD_dot[1]=0; qD_dot[2]=0; qD_dot[3]=0;}
			else if ((d->time)<124) { qD_dot[0]=0; qD_dot[1]=0; qD_dot[2]=-0.1; qD_dot[3]=0;}
			else if ((d->time)<126) { qD_dot[0]=0; qD_dot[1]=0; qD_dot[2]=0; qD_dot[3]=0;}
			else if ((d->time)<136) { qD_dot[0]=0; qD_dot[1]=0; qD_dot[2]=0.1; qD_dot[3]=0;}
			else if ((d->time)<138) { qD_dot[0]=0; qD_dot[1]=0; qD_dot[2]=0; qD_dot[3]=0;}
			else if ((d->time)<148) { qD_dot[0]=0; qD_dot[1]=0; qD_dot[2]=0; qD_dot[3]=0.1;}
			else if ((d->time)<150) { qD_dot[0]=0; qD_dot[1]=0; qD_dot[2]=0; qD_dot[3]=0;}
			else if ((d->time)<170) { qD_dot[0]=0; qD_dot[1]=0; qD_dot[2]=0; qD_dot[3]=-0.1;}
			else if ((d->time)<172) { qD_dot[0]=0; qD_dot[1]=0; qD_dot[2]=0; qD_dot[3]=0;}
			else if ((d->time)<182) { qD_dot[0]=0; qD_dot[1]=0; qD_dot[2]=0; qD_dot[3]=0.1;}
			else if ((d->time)<184) { qD_dot[0]=0; qD_dot[1]=0; qD_dot[2]=0; qD_dot[3]=0;}
			
			
			// qD_dot profile 2
			// if ((d->time)<10) { qD_dot[0]=0.1; qD_dot[1]=0.1; qD_dot[2]=0.1; qD_dot[3]=0.1;}
			// else if ((d->time)<12) { qD_dot[0]=0; qD_dot[1]=0; qD_dot[2]=0; qD_dot[3]=0;}
			// else if ((d->time)<32) { qD_dot[0]=-0.1; qD_dot[1]=-0.1; qD_dot[2]=-0.1; qD_dot[3]=-0.1;}
			// else if ((d->time)<34) { qD_dot[0]=0; qD_dot[1]=0; qD_dot[2]=0; qD_dot[3]=0;}
			// else if ((d->time)<44) { qD_dot[0]=0.1; qD_dot[1]=0.1; qD_dot[2]=0.1; qD_dot[3]=0.1;}
			// else if ((d->time)<46) { qD_dot[0]=0; qD_dot[1]=0; qD_dot[2]=0; qD_dot[3]=0;}
			
			// qD_dot profile 3
			// if ((d->time)<10) { qD_dot[0]=0.1; qD_dot[1]=-0.1; qD_dot[2]=0.1; qD_dot[3]=-0.1;}
			// else if ((d->time)<12) { qD_dot[0]=0; qD_dot[1]=0; qD_dot[2]=0; qD_dot[3]=0;}
			// else if ((d->time)<32) { qD_dot[0]=-0.1; qD_dot[1]=0.1; qD_dot[2]=-0.1; qD_dot[3]=0.1;}
			// else if ((d->time)<34) { qD_dot[0]=0; qD_dot[1]=0; qD_dot[2]=0; qD_dot[3]=0;}
			// else if ((d->time)<44) { qD_dot[0]=0.1; qD_dot[1]=-0.1; qD_dot[2]=0.1; qD_dot[3]=-0.1;}
			// else if ((d->time)<46) { qD_dot[0]=0; qD_dot[1]=0; qD_dot[2]=0; qD_dot[3]=0;}
			
			//printf("%d\n", d->sensordata);
			
			for (int i=0;i<4;i++)
			{
				//if (iteration<end_iteration) 
					// d->ctrl[i]=joint_torque[i][iteration];
				
				//else 
					//d->ctrl[i]=0;
				//printf("%f %lld ",d->ctrl[i],iteration);
				
				d->ctrl[i]=qD_dot[i]-d->qvel[i+6];
			}
			//printf("\n");
			//iteration++;
			
			if ( loop_index%data_frequency==0)
            {
              save_data(m,d);
            }
            loop_index = loop_index + 1;
			
        }

       // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

          // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        //printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();

    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}
