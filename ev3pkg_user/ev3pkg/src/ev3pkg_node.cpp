#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "../include/ev3pkg/definitions.h"

#include <pthread.h>
#include <sstream>

#include <signal.h>
#include <semaphore.h>

struct param *_params;
sem_t mutex;

/*************************************************************************
 * AUXILIARY FUNCTIONS
 *************************************************************************
*/
/*
 * Function to empty the content of a buffer
 * @in
 * 		buf: The buffer to be emptied
 */
void wipe_buffer(char * buf) {
    memset(buf, 0, strlen(buf));
}


void mySigintHandler(int sig)
{	
	ros::shutdown();
}
/*************************************************************************/




/*************************************************************************
 * CALLBACK FUNCTIONS
 *************************************************************************
*/

/*
 * Callback function to retrieve the speed to be set in the out port A of 
 */
void chatterCallback_outA(const std_msgs::String::ConstPtr& msg_outA)
{	
	sem_wait(&mutex);
	_params->outA=atoi(msg_outA->data.c_str());
	if(!_params->isset_outA) _params->isset_outA=true;
	sem_post(&mutex);
	
	ros::spinOnce();
}

/*
 * Callback function to retrieve the speed to be set in the out port B of 
 */
void chatterCallback_outB(const std_msgs::String::ConstPtr& msg_outB)
{
	sem_wait(&mutex);
	_params->outB=atoi(msg_outB->data.c_str());
	if(!_params->isset_outB) _params->isset_outB=true;
	sem_post(&mutex);
	
	ros::spinOnce();
}

/*
 * Callback function to retrieve the speed to be set in the out port C of 
 */
void chatterCallback_outC(const std_msgs::String::ConstPtr& msg_outC)
{
	sem_wait(&mutex);
	_params->outC=atoi(msg_outC->data.c_str());
	if(!_params->isset_outC) _params->isset_outC=true;
	sem_post(&mutex);
	
	ros::spinOnce();
}

/*
 * Callback function to retrieve the speed to be set in the out port D of 
 */
void chatterCallback_outD(const std_msgs::String::ConstPtr& msg_outD)
{
	sem_wait(&mutex);
	_params->outD=atoi(msg_outD->data.c_str());
	if(!_params->isset_outD)_params->isset_outD=true;
	sem_post(&mutex);
	
	ros::spinOnce();
}
/*************************************************************************/




/*************************************************************************
 * THREADS
 *************************************************************************
*/
/*
 * Thread to comunicate with the EV3 and retrieve sensor information
 * @in
 * 		ip_void_ptr: Pointer to the ip address of the EV3
 */
void* ev3_in(void * ip_void_ptr)
{
	int n, port, sock;
	double value;
	
	char * token;
	const char separator[2]="@";
    char buf[MAX_BUF];
     
    char * ip_ptr = (char *) ip_void_ptr;
       
	struct sockaddr_in serv_addr;
	socklen_t addr_size = sizeof(struct sockaddr_in);
	
	std_msgs::String msg;
	std::stringstream ss;
	
	ros::NodeHandle nh;

	// Create the socket
	if((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		perror("Error while creating the socket");
		exit(1);
	}
	
	// Set an address for the socket
	memset(&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(EV3_PORT);
	if(inet_aton(ip_ptr,&serv_addr.sin_addr) <= 0)
	{
		fprintf(stderr,"Wrong IP: %s\n", ip_ptr);
		exit(1);
	}
	
	// Connect
	if(connect(sock, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
		perror("Connection error");
		exit(1);
	}

	// Create the Publishers which will public the sensor information in the corresponding topic
    ros::Publisher chatter_in1 = nh.advertise<std_msgs::String>("ev3_in1", 10);
    ros::Publisher chatter_in2 = nh.advertise<std_msgs::String>("ev3_in2", 10);
    ros::Publisher chatter_in3 = nh.advertise<std_msgs::String>("ev3_in3", 10);
    ros::Publisher chatter_in4 = nh.advertise<std_msgs::String>("ev3_in4", 10);

    ros::Rate loop_rate(10);

	/* 	Communicate with the server and let it know the client is ready to retrieve
		sensor information */
    sprintf(buf,"INP");
	if(write(sock,buf,3)<3)
	{	
		perror("Error while sending data");
		close(sock);
		exit(1);
	}
	
    while(1)
    {
		// Empty the buffer to avoid trash when retrieving sensor information
		wipe_buffer(buf);

		// Get sensor information from the server
		if((n=read(sock,buf,MAX_BUF))>0)
		{	
			// Insert a string terminator to avoid tokenization problems
			buf[n]='\0';
			
			// Tokenize the port the sensor is connected to
			token = strtok(buf+3, separator);			
			while(token != NULL)
			{
				// Store the port as int
    			port = atoi(token);					

				// Value
				token = strtok( NULL, separator);	// Tokenize the value
				value = strtod(token,NULL);			// Store the value as double

				
				ss.str("");
				ss << value;						// Store the sensor value in the string stream
				msg.data = ss.str();				// Store the sensor value in a message

				/*	Publish the message in its corresponding topic
					depending on the port the sensor which provided the value
					is connected
				*/
				switch(port)
				{
						case 1:
							chatter_in1.publish(msg);
							break;
						case 2:
							chatter_in2.publish(msg);
							break;
						case 3:
							chatter_in3.publish(msg);
							break;
						case 4:
							chatter_in4.publish(msg);
							break;
				}

				ros::spinOnce();

				// Tokenize the port the sensor is connected to
				token = strtok(NULL, separator);		
			} // end while

			// Empty the buffer to avoid trash when sending an answer to the server
			wipe_buffer(buf);

			// Send the answer to the server
			sprintf(buf,"OK");
			if(write(sock,buf,2)<2)
			{	
				perror("Error while sending data");
				close(sock);
				exit(1);
			}
		} // end if read

	loop_rate.sleep();
	} // end while (1)

	close(sock);
	pthread_exit(NULL);
	
	exit(0);
}

/*
 * Thread to comunicate with the EV3 and send the speed to be set in the motors
 * @in
 * 		ip_void_ptr: Pointer to the ip address of the EV3
 */
void* ev3_out(void * param_void_ptr)
{
	int n, sock;
    char buf[MAX_BUF];
    
	struct param *param_ptr = (struct param*) param_void_ptr;
   
	struct sockaddr_in serv_addr;
	socklen_t addr_size = sizeof(struct sockaddr_in);

	// Create the socket
	if((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		perror("Error while creating the socket");
		exit(1);
	}

	// Give the socket an address
	memset(&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(EV3_PORT);
	if(inet_aton(param_ptr->ip,&serv_addr.sin_addr) <= 0)
	{
		fprintf(stderr,"Wrong IP: %s\n", param_ptr->ip);
		exit(1);
	}

	// Connect
	if(connect(sock, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
		perror("Connection error");
		exit(1);
	}

	/* Initialize all the motor statuses to not connected
	 * sem for the synchronization between threads*/
	sem_wait(&mutex);
	_params->isset_outA=false;
	_params->isset_outB=false;
	_params->isset_outC=false;
	_params->isset_outD=false;
	sem_post(&mutex);
	
	/* 	Communicate with the server and let it know the client is ready to send
		the speed list to be set in the motors */
	sprintf(buf,"OUT");
	if(write(sock,buf,3)<3)
	{	
		perror("Error while sending data");
		close(sock);
		exit(1);
	}

	// Wait for server answer
    wipe_buffer(buf);
    if((n=read(sock, buf, MAX_BUF))>=0)
    {
		// Check if the server has returned OK
		if(strncmp(buf,"OK",2))
        {
			perror("Error while receiving data");
            close(sock);
            exit(1);	
	    }	
     }

    while(1)
    {
		/* Retrieve the speed from the callback and insert the port it has
		 * to be set at + the speed to be set in the end of the buffer */
		sem_wait(&mutex);
		if(_params->isset_outA)
				sprintf(buf,"%soutA@%d@",buf,_params->outA);
		sem_post(&mutex);
		
		sem_wait(&mutex);
		if(_params->isset_outB)
				sprintf(buf,"%soutB@%d@",buf,_params->outB);
		sem_post(&mutex);
		
		sem_wait(&mutex);
		if(_params->isset_outC)
				sprintf(buf,"%soutC@%d@",buf,_params->outC);
		sem_post(&mutex);
		
		sem_wait(&mutex);
		if(_params->isset_outD)
				sprintf(buf,"%soutD@%d@",buf,_params->outD);
		sem_post(&mutex);
		
		
		if(strlen(buf)>=9)	// 9 is the minimum size of the buffer OUTout#@_@
		{
			// Send the buffer to the server
			if(write(sock,buf,strlen(buf))<strlen(buf))
			{	
				perror("Error while sending data");
				close(sock);
				exit(1);
			}

			// Receive server answer
			wipe_buffer(buf);
			if((n=read(sock,buf,MAX_BUF))>=0)
			{
				if(strncmp(buf,"OK",2))
				{
					perror("Error while receiving data");
					close(sock);
					exit(1);
				}
			}

			// Reinitialize the buffer inserting only the command
			wipe_buffer(buf);
			sprintf(buf,"OUT");
		}
		
	}

	close(sock);
	pthread_exit(NULL);
	exit(0);
}

/*
 * Thread to subscribe to the topics containing the speed to be set in the motors
 * @in
 * 		ip_void_ptr: Pointer to the ip address of the EV3
 */
void* ev3_motor(void * param_void_ptr)
{

	ros::NodeHandle n;

	ros::Subscriber sub_outA = n.subscribe("ev3_outA",10, chatterCallback_outA);
	ros::Subscriber sub_outB = n.subscribe("ev3_outB",10, chatterCallback_outB);
	ros::Subscriber sub_outC = n.subscribe("ev3_outC",10, chatterCallback_outC);
	ros::Subscriber sub_outD = n.subscribe("ev3_outD",10, chatterCallback_outD);

	ros::spin();
	
	pthread_exit(NULL);
	exit(0);
}


/*
 * Thread to comunicate with the EV3 and retrieve odometry information
 * @in
 * 		ip_void_ptr: Pointer to the ip address of the EV3
 */
void* ev3_out_odometry(void * ip_void_ptr)
{
	int n, sock;
	char * token;
	int port,tick;
	
	const char separator[2]="@";
    char buf[MAX_BUF];
     
	char *ip_ptr = (char *) ip_void_ptr;
       
	struct sockaddr_in serv_addr;
	socklen_t addr_size = sizeof(struct sockaddr_in);

	std_msgs::String msg_tick;
	std::stringstream ss;

    ros::NodeHandle nh;

	// Create the socket
	if((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		perror("Error while creating the socket");
		exit(1);
	}
	
	// Set an address for the socket
	memset(&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(EV3_PORT);
	if(inet_aton(ip_ptr,&serv_addr.sin_addr) <= 0)
	{
		fprintf(stderr,"Wrong IP: %s\n", ip_ptr);
		exit(1);
	}

	// Connect 
	if(connect(sock, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
		perror("Connection error");
		exit(1);
	}

	/* 	Communicate with the server and let it know the client is ready to retrieve
		odometry information */
	sprintf(buf,"ODO");
	if(write(sock,buf,strlen(buf))<strlen(buf))
	{	
		perror("Error while sending data");
		close(sock);
		exit(1);
	}

	// Create the Publishers which will public the sensor information in the corresponding topic
    ros::Publisher chatter_odoA = nh.advertise<std_msgs::String>("ev3_odoA", 10);
    ros::Publisher chatter_odoB = nh.advertise<std_msgs::String>("ev3_odoB", 10);
    ros::Publisher chatter_odoC = nh.advertise<std_msgs::String>("ev3_odoC", 10);
    ros::Publisher chatter_odoD = nh.advertise<std_msgs::String>("ev3_odoD", 10);

    while(1)
    {
		// Empty the buffer to avoid trash when retrieving odometry information
		wipe_buffer(buf);

		// Get odometry information from the server
		if((n=read(sock,buf,MAX_BUF))>0)
		{	
			// Insert a string terminator to avoid tokenization problems
			buf[n]='\0';

			// Tokenize motor port the motor is connected to
			token = strtok(buf+3, separator);			
			while(token != NULL)
			{
				// Store the port as int
				port = atoi(token);
				// Tokenize odometry information
				token = strtok(NULL, separator);
				// Store odometry information as int
				tick = atoi(token);
				
				
				ss.str("");
				ss << tick;						// Store the odometry value in the string stream
				msg_tick.data = ss.str();		// Store the odometry value in a message


				/*	Publish the message in its corresponding topic
					depending on the port the motor which provided the value
					is connected
				*/
				switch(port)
				{
						case 1:
							chatter_odoA.publish(msg_tick);
							break;
						case 2:
							chatter_odoB.publish(msg_tick);
							break;
						case 3:
							chatter_odoC.publish(msg_tick);
							break;
						case 4:
							chatter_odoD.publish(msg_tick);
							break;							
				}
				
				// Tokenize the port the motor is connected to
				token = strtok(NULL, separator);	
			}
		
			// Empty the buffer to avoid trash when sending an answer to the server
			wipe_buffer(buf);

			// Send the answer to the server
			sprintf(buf,"OK");
			if(write(sock,buf,2)<2)
			{	
				perror("odo2::Error while sending data");
				close(sock);
				exit(1);
			}

		} // end if read


	} // end while (1)

	close(sock);
	pthread_exit(NULL);
	
	exit(0);
}
/*************************************************************************/

/*
 * Main function 
 */
int main(int argc, char *argv[])
{
	pthread_t ev3_in_thread, ev3_out_thread, ev3_out_motor, ev3_out_odo;

	// Struct to store the motor information
	_params = (param*) malloc(sizeof(struct param));

	signal(SIGINT, mySigintHandler);

	// Sem for the synchronization of threads
	sem_init(&mutex, 0, 1);

	// Initialize de ros node
    ros::init(argc, argv, "ev3pkg_node");

    ros::NodeHandle nh;    
    std::string ip;
    ros::Rate loop_rate(10);

	// Get the ip from launchar parameters
	ros::param::get("/ev3_driver/ip", ip );
	strcpy(_params->ip,ip.c_str());

	// Execute the thread to retrieve sensor information and publish it in topics
	if(	pthread_create(&ev3_in_thread, NULL, &ev3_in, (void *) _params->ip) ){
		fprintf(stderr, "Error creating thread\n");
		return 1;
	}

	/*	Execute the thread to subscribe to the topics containing the speed to be set
		in the motors and retrieve it */
	if(	pthread_create(&ev3_out_motor, NULL, &ev3_motor, NULL) ){
		fprintf(stderr, "Error creating thread\n");
		return 1;
	}

	// 	Execute the thread to send the speed to be set in the motors to the server
	if(	pthread_create(&ev3_out_thread, NULL, &ev3_out, (void *) _params) ){
		fprintf(stderr, "Error creating thread\n");
		return 1;
	}

	// Execute the thread to retrieve odometry information and publish it in topics
	if(	pthread_create(&ev3_out_odo, NULL, &ev3_out_odometry, (void *) _params->ip) ){
		fprintf(stderr, "Error creating thread\n");
		return 1;
	}

 	pthread_join(ev3_out_motor,NULL);
	pthread_join(ev3_out_odo,NULL);
	pthread_join(ev3_out_thread,NULL);
	pthread_join(ev3_in_thread,NULL);
	
	exit(0);
}
/*************************************************************************/

