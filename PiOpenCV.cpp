// ****************************************************************************
//
//  Open CV Line Follower - (c) 2017 Phil Willis
//
//
//         
//	Depends:
//		wiringPi - http://wiringpi.com/
//		joystick_pi 
//
// ***************************************************************************
//
#include <iostream>
#include <string>
#include <vector>
#include <stdio.h>
#include <math.h>

#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

// Wiring Pi a nice C lib for accessing the GPIO 
#include <wiringPi.h>
#include <softPwm.h>
#include <joystick_pi/joystick_pi.h>
#include <zeroborg/zeroborg.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include "opencv2/video/tracking.hpp"

#include "raspicam/raspicam_cv.h"

// Define CYTRON if using the CYTRON control board or not if using ZeroBorg
#define CYTRON

#ifdef CYTRON
//#define LEFT_MOTOR_DIR     	8
//#define RIGHT_MOTOR_DIR    	10
//#define LEFT_MOTOR_DRIVE   	9
//#define RIGHT_MOTOR_DRIVE  	11
#define LEFT_MOTOR_DIR     	10
#define RIGHT_MOTOR_DIR    	12
#define LEFT_MOTOR_DRIVE   	13
#define RIGHT_MOTOR_DRIVE  	14
#define FWD 0  //1
#define BWD 1  //0
#endif

#define LED 				3  // GPIO 22

using namespace std;
using namespace cv;
using namespace raspicam;

int lowThreshold = 90;
unsigned int highSpeed = 160;
unsigned int lowSpeed = 120;
int imageWidth = 320;
int minWidth = 30;

static volatile bool running = true;

int zb;

// Get a key press from a raw terminal stream
char getKey() 
{
	fd_set set;
	struct timeval tv;

	tv.tv_sec = 0;
	tv.tv_usec = 10;

	FD_ZERO( &set );
	FD_SET( fileno( stdin ), &set );

	int res = select( fileno( stdin )+1, &set, NULL, NULL, &tv );

	if( res > 0 )
	{
		char c;
		printf( "Input seen\n" );
		read( fileno( stdin ), &c, 1 );
		return c;
	}
	else if( res < 0 )
	{
		fprintf(stderr,"Select error\n" );
		return 0;
	}
	return 0;
}

void SetRightM(int speed, float gear)
{
	speed = speed / gear;
	if( speed < 0 ) 
	{
		speed = min(speed * -1, 255);
		softPwmWrite (RIGHT_MOTOR_DRIVE, speed );
		digitalWrite (RIGHT_MOTOR_DIR, BWD );
	}
	else
	{
		softPwmWrite (RIGHT_MOTOR_DRIVE, speed );
		digitalWrite (RIGHT_MOTOR_DIR, FWD );
	}
}

void SetLeftM(int speed, float gear)
{
	speed = speed / gear;
	if( speed < 0 ) 
	{		
		speed = min(speed * -1, 255);		
		softPwmWrite (LEFT_MOTOR_DRIVE, speed );
		digitalWrite (LEFT_MOTOR_DIR, BWD );
	}
	else
	{
		speed = min(speed, 255);
		softPwmWrite (LEFT_MOTOR_DRIVE, speed );
		digitalWrite (LEFT_MOTOR_DIR, FWD );
	}
}

void SetRightMotor(float speed)
{	
	SetMotor( 1, speed);
	SetMotor( 2, speed);
}

void SetLeftMotor(float speed)
{
	SetMotor( 3, speed);
	SetMotor( 4, speed);	
}

// All stop!!
void Stop() 
{    
	#ifdef CYTRON	
	SetRightM(0,0);
    SetLeftM(0,0);
    #else
    SetRightMotor(0);
    SetLeftMotor(0);
    #endif
}

unsigned int CalcSpeed( double diff ) {
	double lowLimit = lowSpeed;
	double highLimit = highSpeed - lowSpeed;
	
	diff = abs(diff);
	
	// ( diff / width == speed / highLimit ) + lowLimit    
    int speed = int( (highLimit * ( diff / imageWidth )) + lowLimit );
    //printf("Speed %d \n",speed);
    return speed;    
}


//TBDL - Take notice of return values
unsigned int TurnRight(double diff) {
    unsigned int ret = 0;
     
    int speed = CalcSpeed( diff ); 
    printf("Turn Right Speed %d \n",speed);
    SetLeftM(speed, 1);
    SetRightM(0, 1);
    return ret;  
}

unsigned int TurnLeft(double diff) {
    unsigned int ret = 0;
    
    int speed = CalcSpeed( diff ); 
    printf("Turn Left Speed %d \n",speed);
    SetLeftM(0, 1);
    SetRightM(speed * -1, 1);
    return ret; 
}

unsigned int Straight(){
    unsigned int ret = 0;
    printf("Straight\n");
    SetRightM(lowSpeed * -1, 1);
    SetLeftM(lowSpeed, 1);
    return ret; 
}

double GetPosition(Mat camera, bool viewing)
{
    Mat outImg;
    Mat roiImg;
      
    //IplImage* image = raspiCamCvQueryFrame(capture);
    //Mat camera(image);    
    
    //Extract a region of interest from the grey scale frame
    Rect roi(0,190,640,100);
    
    camera(roi).copyTo(roiImg);       
    
    threshold(roiImg, roiImg, lowThreshold , 255, 0);
    
    // negative image
    bitwise_not(roiImg, roiImg);    
    Mat erodeElmt = getStructuringElement(MORPH_RECT, Size(3, 3));
    Mat dilateElmt = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(roiImg, roiImg, erodeElmt);
    dilate(roiImg, roiImg, dilateElmt);
   
    roiImg.copyTo(outImg);
    
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(roiImg, contours, hierarchy, CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE, Point(0,0));
  
    for (size_t i = 0; i < contours.size(); i++) {
       float area = contourArea(contours[i]);       
       if( area > 60000 ) {
           // Stop motors and wait for positive hit
           Stop();
           return 9999.0;
       }
       else if (area > 2000) {
          Moments mu;
          mu = moments(contours[i], false);
          
          Point2f center(mu.m10 / mu.m00, 50);
          circle(outImg, center, 5, Scalar(128, 128, 128), -1, 8, 0);

		  // Show the viewers at home what we can see
		  if( viewing )
			imshow("C", outImg);
		  else
			imwrite ("outImage.jpg",outImg );		
		  
          double position = mu.m10/mu.m00;
          double diff = imageWidth - position;
          
          //printf("Centre = %f Diff = %f Area %f\n", position, diff, area);
          
          return diff;
      }
    } 
    return 0;
}

int main(int argc, char** argv) 
{
	// GPIO access needs to be root 
	if(geteuid() != 0)
	{
		fprintf(stderr, "Error: requires root to run (sudo?).\n");   
		exit(-1);
	}    	
	
	
	
	// Set up Wiring Pi to use native GPIO numbering
	wiringPiSetup();
	//wiringPiSetupGpio();
	
	#ifdef CYTRON
	pinMode (RIGHT_MOTOR_DIR, OUTPUT);
	pinMode (LEFT_MOTOR_DIR, OUTPUT);
	int a = softPwmCreate(RIGHT_MOTOR_DRIVE, 0, 255);	
	int b = softPwmCreate(LEFT_MOTOR_DRIVE,  0, 255);	
	int result = a + b;	
	if( result != 0 ) {
	  fprintf(stderr, "Error setting PWM.\n");	  
	  exit(-1);
	} 
	printf("Soft PWM enabled\n");	
	#else
	if( ZeroBorgInit(1) < 0 )
	{
		fprintf(stderr, "Failed to init zeroborg.\n");   
		exit(-1);
	}    
	#endif	

	RaspiCam_Cv Camera;	
    Camera.set (CV_CAP_PROP_FORMAT, CV_8UC1 );
    Camera.set (CV_CAP_PROP_GAIN, 50 );    
    Camera.set (CV_CAP_PROP_FRAME_WIDTH, 640 );
    Camera.set (CV_CAP_PROP_FRAME_HEIGHT, 480 );
    Camera.set (CV_CAP_ROTATION, 180 );
    Camera.set (CV_CAP_PROP_BRIGHTNESS, 60 );
	Camera.set (CV_CAP_PROP_CONTRAST, 70 );	 
	Camera.set (CV_CAP_PROP_GAIN, 50 );
    
    if ( !Camera.open() ) {
        fprintf(stderr, "Failed to init open camera\n");   
		exit(-1);
    }
    
    bool viewing = false;
    if( argc > 1 )
    {
		namedWindow("C", CV_WINDOW_AUTOSIZE);
		viewing = true;
	}
		
	// To allow us to simply press a key without Enter we set our terminal to RAW
	// and we save the original settings to restore them on exit
	struct termios oldSettings, newSettings;
	tcgetattr( fileno( stdin ), &oldSettings );
	newSettings = oldSettings;
	newSettings.c_lflag &= (~ICANON & ~ECHO);
	tcsetattr( fileno( stdin ), TCSANOW, &newSettings );  
	
	int fd, rc;
	int leftStick = 0;
	int rightStick = 0;
				
    struct js_event jse;

	fd = open_joystick();
	if (fd < 0) {
		printf("open failed.\n");
		exit(1);
	}
	
	float gear = 2.0;
		
    bool follow = false;
    pinMode (LED, OUTPUT);
    digitalWrite (LED, follow );
    
    Stop();
    		
	Mat image;		
	while( running )
	{		
		if( follow )
		{
			Camera.grab();
			Camera.retrieve(image);
		
			double diff = GetPosition(image, viewing);
			//printf( " Diff %f \n",diff);
		
			if(diff == 9999.0)
				Stop();
			else if(diff > minWidth)
				TurnRight(diff);			
			else if(diff < -minWidth)			
				TurnLeft(diff);					
			else
				Straight();		
		}	
			
		rc = read_joystick_event(&jse);		
		if (rc == 1) {
			//printf("Event: time %8u, value %8hd, type: %3u, axis/button: %u\n", jse.time, jse.value, jse.type, jse.number);
			
			// Left stick position
			if( jse.number == 1 && jse.type == 2) 
				leftStick = jse.value;
				
			// Right stick position
			if( jse.number == 3 && jse.type == 2) 
				rightStick = jse.value;
				
			if( jse.type == 1 && jse.value == 1 )
			{
				//printf("Event: time %8u, value %8hd, type: %3u, axis/button: %u\n", jse.time, jse.value, jse.type, jse.number);
				if( jse.number == 5 )
					gear -= 0.25;
				
				if( jse.number == 4 )
					gear += 0.25;
				
				if( gear < 1 )
					gear = 1;
				
				if( gear > 3 )
					gear = 3;
					
				if( jse.number == 8 )
					running = false;
				
				if( jse.number == 9 )
					system("sudo halt");
					
				if( jse.number == 3 )
					follow = true;							
					
				if( jse.number == 1 )
					follow = false;							
				
				digitalWrite (LED, follow );	
			}
				
			
			if( !follow )
			{	
				//printf(" leftStick %d, rightStick %d \n", leftStick, rightStick);	
						
				// Invert the right			
				float leftSpeed = (float)(leftStick ) / 32767.0;
				float rightSpeed = (float)(rightStick * -1) / 32767.0;
				
				
				// For safety
				if( leftSpeed == 0.0 && rightSpeed == 0.0 )
				{
					Stop();
				}
				else
				{
					//printf(" LeftSpeed %f, RightSpeed %f \n", leftSpeed, rightSpeed);	
					#ifdef CYTRON
					int left = (leftStick) / 128;
					int right = (rightStick * -1) / 128;
					printf(" LeftSpeed %d, RightSpeed %d \n", (int)(left / gear), (int)(right / gear));	
					SetLeftM( left, gear );
					SetRightM( right, gear );
					#else
					SetLeftMotor( leftSpeed / gear );
					SetRightMotor( rightSpeed / gear );
					#endif
				}	
			}					
		}
		
		//usleep(1000);	
		waitKey(1);
		if(getKey() == 'x') 
			running = false;

	} 
	imwrite ("image.jpg",image );
	Camera.release();
	// Put back the terminal settings as they were
	tcsetattr( fileno( stdin ), TCSANOW, &oldSettings );
	exit(0);
}
