/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavlink_control.cpp
 *
 * @brief An example offboard control process via mavlink
 *
 * This process connects an external MAVLink UART device to send an receive data
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */



// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "mavlink_control.h"


// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int
top (int argc, char **argv)
{

	// --------------------------------------------------------------------------
	//   PARSE THE COMMANDS
	// --------------------------------------------------------------------------

	// Default input arguments
#ifdef __APPLE__
	char *uart_name = (char*)"/dev/tty.usbmodem1";
#else
	char *uart_name = (char*)"/dev/ttyTHS1";
#endif
	int baudrate = 57600;
	//int baudrate = 921600;

	// do the parse, will throw an int if it fails
	parse_commandline(argc, argv, uart_name, baudrate);
	


	// --------------------------------------------------------------------------
	//   PORT and THREAD STARTUP
	// --------------------------------------------------------------------------

	/*
	 * Instantiate a serial port object
	 *
	 * This object handles the opening and closing of the offboard computer's
	 * serial port over which it will communicate to an autopilot.  It has
	 * methods to read and write a mavlink_message_t object.  To help with read
	 * and write in the context of pthreading, it gaurds port operations with a
	 * pthread mutex lock.
	 *
	 */
	Serial_Port serial_port(uart_name, baudrate);


	/*
	 * Instantiate an autopilot interface object
	 *
	 * This starts two threads for read and write over MAVlink. The read thread
	 * listens for any MAVlink message and pushes it to the current_messages
	 * attribute.  The write thread at the moment only streams a position target
	 * in the local NED frame (mavlink_set_position_target_local_ned_t), which
	 * is changed by using the method update_setpoint().  Sending these messages
	 * are only half the requirement to get response from the autopilot, a signal
	 * to enter "offboard_control" mode is sent by using the enable_offboard_control()
	 * method.  Signal the exit of this mode with disable_offboard_control().  It's
	 * important that one way or another this program signals offboard mode exit,
	 * otherwise the vehicle will go into failsafe.
	 *
	 */
	Autopilot_Interface autopilot_interface(&serial_port);
	
	/*
	 * Setup interrupt signal handler
	 *
	 * Responds to early exits signaled with Ctrl-C.  The handler will command
	 * to exit offboard mode if required, and close threads and the port.
	 * The handler in this example needs references to the above objects.
	 *
	 */
	serial_port_quit         = &serial_port;
	//printf("!!!\n");
	autopilot_interface_quit = &autopilot_interface;
	//printf("!!!\n");
	signal(SIGINT,quit_handler);

	/*
	 * Start the port and autopilot_interface
	 * This is where the port is opened, and read and write threads are started.
	 */
	 
	serial_port.start();
	//printf("!!!\n");
	autopilot_interface.start();
	//printf("!!!\n");
	

	// --------------------------------------------------------------------------
	//   RUN COMMANDS
	// --------------------------------------------------------------------------

	/*
	 * Now we can implement the algorithm we want on top of the autopilot interface
	 */
	commands(autopilot_interface);


	// --------------------------------------------------------------------------
	//   THREAD and PORT SHUTDOWN
	// --------------------------------------------------------------------------

	/*
	 * Now that we are done we can stop the threads and close the port
	 */
	autopilot_interface.stop();
	serial_port.stop();


	// --------------------------------------------------------------------------
	//   DONE
	// --------------------------------------------------------------------------

	// woot!
	return 0;

}


// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------

void
commands(Autopilot_Interface &api)
{
	//flags for judging pre-conditions of flight
	bool flag_isFlying = false;
	bool flag_isWaiting = false;

	
	// initialize command data strtuctures
	mavlink_set_position_target_local_ned_t ip = api.initial_position;


	while (1)
	{
		//printf("[% .4f, %.4f, %.4f]\n", api.current_messages.attitude.pitch,api.current_messages.attitude.roll,api.current_messages.attitude.yaw);
		// -------------------------------
		// keep sending stay put command to pixhawk
		// -------------------------------
		api.update_setpoint(ip);
		// -------------------------------
		if (api.current_messages.heartbeat.custom_mode == 393216)		//switch to offboard mode
		{
			printf("offboard!\n");
			if ( execute_flight_task(api) == 0 )
			{
				api.disable_offboard_control();				
			}
				
		}				
		usleep( 200 * 1E3);
	}


	// --------------------------------------------------------------------------
	//   END OF COMMANDS
	// --------------------------------------------------------------------------

	return;

}

// ------------------------------------------------------------------------------
// Flight Task
// ------------------------------------------------------------------------------
int 
execute_flight_task(Autopilot_Interface &api)
{

	mavlink_global_position_int_t home = { 0 };
	
	if ( flight_takeoff(api, home) == 0 )
		return 0;
	else if ( flight_ascend(api, 5.0) == 0 )
		return 0;
	else if	( flight_level(api) == 0 )
		return 0;
	else if ( flight_land(api, home) == 0 )
		return 0;
	else 
		return 1;
}

// ------------------------------------------------------------------------------
// kinds of tasks
// ------------------------------------------------------------------------------

int
flight_takeoff(Autopilot_Interface &api, mavlink_global_position_int_t &gps)
{
	// Prepare command for off-board mode
	Mavlink_Messages message_in = api.current_messages;
	float height = (float) message_in.global_position_int.alt / 1E3 + 1.0;
	gps = api.current_messages.global_position_int;
	
	mavlink_command_long_t cmd = { 0 };
	cmd.target_system    	= api.system_id;
	cmd.target_component 	= api.autopilot_id;
	cmd.command          	= MAV_CMD_NAV_TAKEOFF;
	cmd.confirmation     	= true;
	cmd.param4           	= message_in.attitude.yaw;   	//set yaw angle
	cmd.param5				= (float) message_in.global_position_int.lat / 1E7;
	cmd.param6				= (float) message_in.global_position_int.lon / 1E7;
	cmd.param7				= height;

	// Encode
	mavlink_message_t message_out;
	mavlink_msg_command_long_encode(api.system_id, api.companion_id, &message_out, &cmd);

	// Send the message
	int len = api.serial_port->write_message(message_out);
	
	while (1)
	{
		// Done!
		if (api.current_messages.heartbeat.custom_mode == 65536)
			return 0;
		if ( fabs( ( (float) api.current_messages.global_position_int.alt / 1E3)  - height) < 0.2 )
			return 1;	
		usleep( DT * 1E3 );
	}	
}

int
flight_ascend(Autopilot_Interface &api, float height)
{
	// Get initial position and set target
	Mavlink_Messages message_in = api.current_messages;
	mavlink_set_position_target_local_ned_t sp;	
	sp.x        = message_in.local_position_ned.x;
	sp.y        = message_in.local_position_ned.y;
	sp.z        = message_in.local_position_ned.z - height;
	sp.yaw      = message_in.attitude.yaw;
	
	set_position(sp.x, sp.y, sp.z, sp);
	set_yaw(sp.yaw, sp);
	api.update_setpoint(sp);
	
	while (1)
	{
		// Done!
		if (api.current_messages.heartbeat.custom_mode == 65536)
			return 0;
		if ( fabs(api.current_messages.local_position_ned.z - sp.z) < 0.1 )
			return 1;
		usleep( DT * 1E3 );		
	}
}

int
flight_level(Autopilot_Interface &api)
{
	float distance;
	// Get initial position and set target
	Mavlink_Messages message_in = api.current_messages;
	mavlink_set_position_target_local_ned_t ip;
	mavlink_set_position_target_local_ned_t sp;

	ip.x        = message_in.local_position_ned.x;
	ip.y        = message_in.local_position_ned.y;
	ip.z        = message_in.local_position_ned.z;
	ip.yaw      = message_in.attitude.yaw; 
	
	sp.x        = ip.x + 3.0;
	sp.y        = ip.y - 3.0;
	sp.z        = ip.z;
	sp.yaw      = ip.yaw;
	
	set_position(sp.x, sp.y, sp.z, sp);
	set_yaw(sp.yaw, sp);
	api.update_setpoint(sp);
	
	while (1)
	{
		// Done!
		if (api.current_messages.heartbeat.custom_mode == 65536)
			return 0;
		distance = pow(api.current_messages.local_position_ned.x - sp.x, 2) + pow(api.current_messages.local_position_ned.y - sp.y, 2);
		if ( distance < 0.2 )
			break;
		usleep( DT * 1E3 );		
	}

	api.update_setpoint(ip);

	while (1)
	{
		if (api.current_messages.heartbeat.custom_mode == 65536)
			return 0;
		distance = pow(api.current_messages.local_position_ned.x - ip.x, 2) + pow(api.current_messages.local_position_ned.y - ip.y, 2);
		if ( distance < 0.2 )
			return 1;
		usleep( DT * 1E3 );		
	}
	
}

int
flight_land(Autopilot_Interface &api, mavlink_global_position_int_t land_position)
{
	// Prepare command for off-board mode
	mavlink_command_long_t cmd = { 0 };
	cmd.target_system    	= api.system_id;
	cmd.target_component 	= api.autopilot_id;
	cmd.command          	= MAV_CMD_NAV_LAND;
	cmd.confirmation     	= true;
	cmd.param4           	= api.current_messages.attitude.yaw;   	//set yaw angle
	cmd.param5				= (float) land_position.lat / 1E7;
	cmd.param6				= (float) land_position.lon / 1E7;
	cmd.param7				= (float) land_position.alt / 1E3;

	// Encode
	mavlink_message_t message_out;
	mavlink_msg_command_long_encode(api.system_id, api.companion_id, &message_out, &cmd);

	// Send the message
	int len = api.serial_port->write_message(message_out);
	
	while (1)
	{
		// Done!
		if (api.current_messages.heartbeat.custom_mode == 65536)
			return 0;
		if ( fabs( ( api.current_messages.global_position_int.alt - land_position.alt ) < 100 ))
			return 1;	
		usleep( DT * 1E3 );
	}	
}


// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

	// string for command line usage
	const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";

	// Read input arguments
	for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"
	
		printf("argc=%d\n",argc);
		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			throw EXIT_FAILURE;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

	}
	// end: for each input argument

	// Done!
	return;
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

	// autopilot interface
	try {
		autopilot_interface_quit->handle_quit(sig);
	}
	catch (int error){}

	// serial port
	try {
		serial_port_quit->handle_quit(sig);
	}
	catch (int error){}

	// end program here
	exit(0);

}


// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main(int argc, char **argv)
{
	// This program uses throw, wrap one big try/catch here
	try
	{
		int result = top(argc,argv);
		return result;
	}

	catch ( int error )
	{
		fprintf(stderr,"mavlink_control threw exception %i \n" , error);
		return error;
	}

}


