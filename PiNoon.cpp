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

using namespace std;

static volatile bool running = true;

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
void SetRightMotor(int speed)
{
	
}

// Set the left motor speed independently of the right, used diectly by the PID engine for the line follower and in tank control mode
void SetLeftMotor(int speed)
{
	
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
					
			int leftSpeed = (leftStick * -1) / 128;
			int rightSpeed = (rightStick * -1) / 128;
			printf(" LeftSpeed %d, RightSpeed %d \n", leftSpeed, rightSpeed);	
			
			// For safety
			if( leftSpeed == 0 && rightSpeed == 0 )
			{
				Stop();
			}
			else
			{
				SetLeftMotor( leftSpeed );
				SetRightMotor( rightSpeed );
			}			
		}
		
		usleep(100);	
		if(getKey() == 'x') 
			running = false;

	} 
	
	// Put back the terminal settings as they were
	tcsetattr( fileno( stdin ), TCSANOW, &oldSettings );
	exit(0);
}
