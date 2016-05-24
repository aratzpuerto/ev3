#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <signal.h>

int _pushed_left, _pushed_right;
double _left_us_value, _right_us_value;

/*************************************************************************
 * AUXILIARY FUNCTIONS
 *************************************************************************
*/
/*
 * Function to publish the speed to be set in a motor
 * @in:
 * 		chatter_motor: The pubisher of the topic in 
 * 						which the speed to be set will be written
 * 		duty_cycle:	The speed to be set
 */ 
void move(ros::Publisher chatter_motor, int duty_cycle)
{
	std_msgs::String msg;
	std::stringstream ss;
	
	ss << duty_cycle;
	msg.data = ss.str();
	chatter_motor.publish(msg);
}

/*
 * Function to handle the ctrl+c signal
 * @in
 * 		sig: Signal identificator
 */
void mySigintHandler(int sig)
{
	ros::NodeHandle nh;

	ros::Publisher chatter_outA = nh.advertise<std_msgs::String>("ev3_outA", 10); 
	ros::Publisher chatter_outB = nh.advertise<std_msgs::String>("ev3_outB", 10);
   	ros::Publisher chatter_outC = nh.advertise<std_msgs::String>("ev3_outC", 10);
	ros::Publisher chatter_outD = nh.advertise<std_msgs::String>("ev3_outD", 10);

	std_msgs::String msg;
	std::stringstream ss;
	
	// Publish 0 as speed in the topics for the motor ports so as to stop the motors 
	ss << 0;
	msg.data = ss.str();
	chatter_outA.publish(msg);
	chatter_outB.publish(msg);
	chatter_outC.publish(msg);
	chatter_outD.publish(msg);
	
	// Shut down the node
	ros::shutdown();
}

/*************************************************************************/


/*************************************************************************
 * CALLBACK FUNCTIONS
 *************************************************************************
*/
// Callback to retrieve the value of the left touch sensor from the topic
void chatterCallback_push_left(const std_msgs::String::ConstPtr& msg_l)
{
	_pushed_left = atoi(msg_l->data.c_str());
	ros::spinOnce();
}
// Callback to retrieve the value of the right touch sensor from the topic
void chatterCallback_push_right(const std_msgs::String::ConstPtr& msg_r)
{
	_pushed_right = atoi(msg_r->data.c_str());
	ros::spinOnce();
}

// Callback to retrieve the value of the left ultrasonic sensor from the topic
void chatterCallback_left_us(const std_msgs::String::ConstPtr& msg_us)
{
	_left_us_value = strtod(msg_us->data.c_str(),NULL);	
	ros::spinOnce();
}

// Callback to retrieve the value of the right ultrasonic sensor from the topic
void chatterCallback_right_us(const std_msgs::String::ConstPtr& msg_us)
{
  _right_us_value=strtod(msg_us->data.c_str(),NULL);
  ros::spinOnce();
}

/*************************************************************************/




int main(int argc, char *argv[])
{
   	std::string left_motor, right_motor; 
    
    // Initialize the node
    ros::init(argc, argv, "ev3pkg_avoid_obstacles");
    
    ros::NodeHandle nh;
     
    ros::Rate loop_rate(10);
    
    // Publishers which will write the speed each motor has to take in topics  
    ros::Publisher chatter_left_motor = nh.advertise<std_msgs::String>("ev3_outA", 10);
    ros::Publisher chatter_right_motor = nh.advertise<std_msgs::String>("ev3_outD", 10);
    
    // Subscribers to read the values of the sensors from topics
    ros::Subscriber sub_left_us = nh.subscribe("ev3_in1",1, chatterCallback_left_us);
    ros::Subscriber sub_push_left = nh.subscribe("ev3_in2", 1, chatterCallback_push_left);
    ros::Subscriber sub_push_right = nh.subscribe("ev3_in3", 1, chatterCallback_push_right);
    ros::Subscriber sub_right_us = nh.subscribe("ev3_in4", 1, chatterCallback_right_us);

	// Signal handler for <ctrl+c>
	signal(SIGINT, mySigintHandler);
	
	while(ros::ok())
	{	
	  // If either ultrasonic sensor finds an object closer than 10cm
	  if ( (_left_us_value <= 0.10) or (_right_us_value <= 0.10)  )		
	  {
		  // Turn the robot backwards to the left
		  move(chatter_left_motor, 0);
		  move(chatter_right_motor, -75);
		  sleep(1);
	  } 

	  
	  if (!_pushed_left && !_pushed_right)	// No touch sensor pushed
	  {
		  // Move straight on
		  move(chatter_left_motor, 75);
		  move(chatter_right_motor, 75);		  
	  }
	  else if  (_pushed_right)		      	// Right touch sensor pushed  
	  {
			// Turn right backwards to the right
			move(chatter_left_motor, -75);
			move(chatter_right_motor, 0);				
			sleep(1);
	  }
	  else 									// Left or both touch sensor(s) pushed      				
		{	
			//		Turn left backwards
			move(chatter_left_motor, 0);
			move(chatter_right_motor, -75);					
			sleep(1);
		}
		
		ros::spinOnce();
		
		loop_rate.sleep();
	}

	exit(0);

}
/*************************************************************************/
