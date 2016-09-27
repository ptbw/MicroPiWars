// ****************************************************************************
//
//  Micro Pi Noon - (c) 2016 Phil Willis
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
#include <joystick_pi/joystick_pi.h>
#include <zeroborg/zeroborg.h>

using namespace std;

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

// Set the right motor speed independently of the left, used directly by the PID engine for the line follower and in tank control mode
void SetRightMotor(float speed)
{
	SetMotor( 1, speed);
	SetMotor( 2, speed);
}

// Set the left motor speed independently of the right, used diectly by the PID engine for the line follower and in tank control mode
void SetLeftMotor(float speed)
{
	SetMotor( 3, speed);
	SetMotor( 4, speed);	
}

// All stop!!
void Stop() 
{    
    SetRightMotor(0);
    SetLeftMotor(0);
}

int main(int argc, char** argv) 
{
	// GPIO access needs to be root 
	if(geteuid() != 0)
	{
		fprintf(stderr, "Error: requires root to run (sudo?).\n");   
		exit(-1);
	}    	
	
	if( ZeroBorgInit(1) < 0 )
	{
		fprintf(stderr, "Failed to init zeroborg.\n");   
		exit(-1);
	}    	

	// Set up Wiring Pi to use native GPIO numbering
	wiringPiSetup();
		
	// To allow us to simply press a key without Enter we set our terminal to RAW
	// and we save the original settings to restore them on exit
	struct termios oldSettings, newSettings;
	tcgetattr( fileno( stdin ), &oldSettings );
	newSettings = oldSettings;
	newSettings.c_lflag &= (~ICANON & ~ECHO);
	tcsetattr( fileno( stdin ), TCSANOW, &newSettings );  
	
	int fd, rc;
	int leftStick;
	int rightStick;
				
    struct js_event jse;

	fd = open_joystick();
	if (fd < 0) {
		printf("open failed.\n");
		exit(1);
	}
	
	float gear = 2.0;
				
	while( running )
	{
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
					
			}
				
				
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
				SetLeftMotor( leftSpeed / gear );
				SetRightMotor( rightSpeed / gear );
			}			
		}
		
		usleep(1000);	
		if(getKey() == 'x') 
			running = false;

	} 
	
	// Put back the terminal settings as they were
	tcsetattr( fileno( stdin ), TCSANOW, &oldSettings );
	exit(0);
}
