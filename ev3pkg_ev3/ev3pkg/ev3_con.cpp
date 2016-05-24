#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <netinet/in.h>

#include <signal.h>

#include "include/definitions.h"
#include "ev3dev-lang/cpp/ev3dev.h"

#include <pthread.h>

#include <sstream>

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

/* Function to store the port value and speed of the motors
 * 
 * @in:
 * 		param: Pointer to the struct to store the tokenized information
 * 		port: The port the motor is connected to
 * 		duty_cycle: The speed to be set in the motor 
 */
void assign_values(param * params, char * port, int duty_cycle)
{
	if(!strcmp(port,"outA"))
		params->outA=duty_cycle;
	if(!strcmp(port,"outB"))
		params->outB=duty_cycle;	
	if(!strcmp(port,"outC"))
		params->outC=duty_cycle;	
	if(!strcmp(port,"outD"))
		params->outD=duty_cycle;
}

/* Function to tokenize the information received from the client
 * 
 * @in:
 * 		param: Pointer to the struct to store the tokenized information
 * 		buf: Buffer containing the information received from the client 
 */
void tokenize_params(param * params, char * buf)
{
	char * token;
	const char separator[2]="@";
	char port[5];
	int	duty_cycle;

	// Tokenize the port the motor is connected to
	token=strtok(buf, separator);

	while(token != NULL)
	{
		// Store the port the motor is connect to
		strcpy(port,token);
		
		// Tokenize the speed to be set in the motor
		token=strtok(NULL, separator);	
		// Store the speed as int
		duty_cycle=atoi(token);
		
		// Make sure the values given by the user are not out of bounds
		if (duty_cycle<-100) duty_cycle=-100;
		else if (duty_cycle>100) duty_cycle=100;
		
		// Store the information in the struct
		assign_values(params, port, duty_cycle);

		// Tokenize the next port the motor is connected to
		token=strtok(NULL, separator);
	}
	
}

/* Function to reset the connected motors
 * 
 */
void reset_out_ports()
{
	int i;
	ev3dev::motor m[4] = { {ev3dev::OUTPUT_A} , {ev3dev::OUTPUT_B}, {ev3dev::OUTPUT_C}, {ev3dev::OUTPUT_D} };

	for (i=0; i<4; i++)
	{
		// Reset each connected motor
		if(m[i].connected()) m[i].reset();	
	}

}
/*************************************************************************/





/*************************************************************************
 * THREADS
 *************************************************************************
*/
/*
 * Thread to handle EV3 sensor. Retrieve the values the sensor provide and
 * send them to the client
 * 
 * @in:
 * 		con_void_ptr: Pointer to the socket for the connection with the client
 */
void* ev3_sensor(void * con_void_ptr)
{	
	int i,n, sensorPort;
	char buf[MAX_BUF];
	double sensorValue;
	
	char sensor_type[MAX_BUF];

	int *con_ptr = (int *) con_void_ptr;
	std::string s_type;

	while(1)
   	 {
		ev3dev::sensor s[4] = { {ev3dev::INPUT_1}, {ev3dev::INPUT_2}, {ev3dev::INPUT_3}, {ev3dev::INPUT_4} };     	 
		
		// Empty the buffer to avoid trash when storing sensor information
		wipe_buffer(buf);
		
		// Insert the command in the buffer
		sprintf(buf,"INP");
		
        for (i=0; i<4; i++)
		{	
			// For each connected sensor
			if (s[i].connected())
			{	
				// Store the port the sensor is connected to
				sensorPort = i+1;
				// Store the float value of the reading the sensor is currently returning								
				sensorValue = s[i].float_value(0);
				// Store the sensor type
				s_type = s[i].type_name();				
				strcpy(sensor_type,s_type.c_str());
				
				/* If the sensor is ultrasonic the reading is being given in centimeters
				 * convert it to meters before sending it to the client*/
				if(!strcmp(sensor_type,"EV3 ultrasonic"))
					sensorValue=sensorValue*0.01;
				
				// Save sensor port + sensor value in the end of the existing buffer
				sprintf(buf,"%s%d@%lg@",buf,sensorPort,sensorValue);
			}

		} // end for

		// Send buffer to the client
        if(write(*con_ptr,buf,strlen(buf))<strlen(buf))
        {
			perror("Error while sending the buffer");
         	close(*con_ptr);
       	    exit(1);
        }	
		
		// Empty the buffer to avoid trash when retrieving client answer
        wipe_buffer(buf);
        
        // Receive client answer
        if((n=read(*con_ptr, buf, MAX_BUF))>=0)
        {
			// Check if the client has returned OK
			if(strncmp(buf,"OK",2))
	        {
				perror("Error while receiving data");
                close(*con_ptr);
	            exit(1);	
	        }	
         }
	}//end while(1)

}


/*
	Function to handle ev3 motors. Set the speed provided by the user in the motors
	
	@in: v_void_ptr: 	Buffer with the out port name the motors to be affected are connected to +
						the speed to be set
 
*/
void* ev3_motor(void * param_void_ptr)
{

	struct param *param_ptr = (struct param*)  param_void_ptr;

	ev3dev::motor m[4] = { {ev3dev::OUTPUT_A} , {ev3dev::OUTPUT_B}, {ev3dev::OUTPUT_C}, {ev3dev::OUTPUT_D} };
	
	// If there is a motor connected to the out port A, set the speed
	if(m[0].connected() && param_ptr->outA!=101)
	{
	   m[0].set_duty_cycle_sp(param_ptr->outA);
	   m[0].run_forever();
	   
	   // Set out of bounds value to know when the user has set a value
	   param_ptr->outA=101;	
	}

	// If there is a motor connected to the out port B, set the speed
	if(m[1].connected() && param_ptr->outB!=101)
	{
	   m[1].set_duty_cycle_sp(param_ptr->outB);
	   m[1].run_forever();
	   
	   // Set out of bounds value to know when the user has set a value
	   param_ptr->outB=101;	
	}

	// If there is a motor connected to the out port C, set the speed
	if(m[2].connected() && param_ptr->outC!=101)
	{
	   m[2].set_duty_cycle_sp(param_ptr->outC);
	   m[2].run_forever();
	   
	   // Set out of bounds value to know when the user has set a value
	   param_ptr->outC=101;	
	}
	
	// If there is a motor connected to the out port D, set the speed
	if(m[3].connected() && param_ptr->outD!=101)
	{
	   m[3].set_duty_cycle_sp(param_ptr->outD);
	   m[3].run_forever();
	   
	   // Set out of bounds value to know when the user has set a value
	   param_ptr->outD=101;	
	}	
	
}

/*
 * Thread to handle the odometry information. Retrieve the current oposition of the motors and
 * send them to the client
 * 
 * @in:
 * 		con_void_ptr: Pointer to the socket for the connection with the client
 */
void* ev3_out_odometry(void * con_void_ptr)
{	
	char buf[MAX_BUF];
	int n,i;
	int *con_ptr = (int *) con_void_ptr;
		
	while(1)
    {
		ev3dev::motor m[4] = { {ev3dev::OUTPUT_A}, {ev3dev::OUTPUT_B}, {ev3dev::OUTPUT_C}, {ev3dev::OUTPUT_D} };     
				
		// Empty the buffer to avoid trash when storing sensor information
		wipe_buffer(buf);

		// Insert the command in the buffer
        sprintf(buf,"ODO");
	    for(i=0; i<4; i++)
	    {	
			// For each connected motor    
			if(m[i].connected())
				// Store the port the motor is connected to and its current position at the end of the buffer
				sprintf(buf,"%s%d@%d@", buf, i+1, m[i].position());
		}

		// Send buffer to the client	    		
        if(write(*con_ptr,buf,strlen(buf))<strlen(buf))
        {
			perror("Error while sending the buffer");
         	close(*con_ptr);
       	    exit(1);
        }	
		
		// Empty the buffer to avoid trash when retrieving client answer
        wipe_buffer(buf);
        
        // Receive client answer
        if((n=read(*con_ptr, buf, MAX_BUF))>=0)
        {
			// Check if the client has returned OK
			if(strncmp(buf,"OK",2))
	        {

				perror("Error while receiving data");
                close(*con_ptr);
	            exit(1);	
	        }	
         }
	}//end while(1)

}
/*************************************************************************/



/*
 * Main function 
 */
int main()
{
   int sock, con,n;
   
   struct sockaddr_in serv_addr;

   char buf[MAX_BUF], tmp[MAX_BUF];
 
   struct param *params;
   params = (param*) malloc(sizeof(struct param));

   pthread_t ev3_in_thread, ev3_out_thread, ev3_odometry_thread;

   // Create socket
   if((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
   {
      perror("Error while creating the socket");
      exit(1);
   }

   // Give the socket an address
   memset(&serv_addr, 0, sizeof(serv_addr)); serv_addr.sin_family = AF_INET; 
   serv_addr.sin_addr.s_addr = htonl(INADDR_ANY); 
   serv_addr.sin_port = htons(EV3_PORT);

   if(bind(sock, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
   {
      perror("Error while assigning addres to the socket");
      exit(1);
   }

   // Set the socket as passive
   if(listen(sock,5) < 0)
   {
      perror("Error while marking the socket as a passive socket");
      exit(1);
   }

   // Ignore child signals to avoid zombies
   signal(SIGCHLD, SIG_IGN);

   // Initialize out port speed struct fields with out of bounds values
   // to control user input speeds
   params->outA=101;	
   params->outB=101;	
   params->outC=101;	
   params->outD=101;	


   while(1)
   {
	  // Connect
      if((con = accept(sock, NULL, NULL))<0)
      {
         perror("Conexion error");
         exit(1);
      }
      
      switch(fork())
      {
         case 0:														// Child proccess
            close(sock);
            // Reset motor ports
			reset_out_ports();
			
			while(1){

				// Retrieve the command from the client
				wipe_buffer(buf);
				if((n=read(con,buf,MAX_BUF))<=0)
				{
					perror("Error while receiving data");
					close(con);
					exit(1);
				}
				
				// Insert a string terminator to avoid tokenization problems
				buf[n]='\0';

				
				if(!strncmp(buf,"INP",3))	// in ports thread
				{  
					if( pthread_create(&ev3_in_thread, NULL, &ev3_sensor, (void * ) &con) )
					{
						perror("Error creating thread\n");
						exit(1);
					}
					pthread_join(ev3_in_thread,NULL);		   
				}
				else if(!strncmp(buf,"OUT",3))	// Motor thread
				{
					// Send answer to the client
					wipe_buffer(tmp);
					sprintf(tmp,"OK");
					if(write(con,tmp,2)<2)
					{	
						perror("Error while sending data");
						close(sock);
						exit(1);
					}

					// Get the buffer from the client
					wipe_buffer(buf);
					if((n=read(con,buf,MAX_BUF))<=0)
					{
						perror("main: Error while receiving data");
						close(con);
						exit(1);
					}			
					buf[n]='\0';

					// 	Tokenize and store the information received from the client
					tokenize_params(params,buf+3);

					if(pthread_create(&ev3_out_thread, NULL, &ev3_motor, (void *)  params) )
					{ 
						perror("Error creating thread\n");
						exit(1);
					} 
					
					// Send answer to the client
					wipe_buffer(tmp);
					sprintf(tmp,"OK");
					if(write(con,tmp,2)<2)
					{	
						perror("Error while sending data");
						close(sock);
						exit(1);
					}
					
					pthread_join(ev3_out_thread,NULL);
				}
				else if(!strncmp(buf,"ODO",3))	// Odometry thread
				{

					if( pthread_create(&ev3_odometry_thread, NULL, &ev3_out_odometry, (void * ) &con) )
					{
						perror("Error creating thread\n");
						exit(1);
					}
					pthread_join(ev3_odometry_thread,NULL);	
				}
			}//end while 1		
	
			close(con);
			exit(0);
		default:
			close(con);
		} // end switch
	} // end while(1)

}


/*************************************************************************/

