#include <inttypes.h>
#include <fcntl.h>
#include <termios.h>
#include <lib/crc/crc.h>

#include <float.h>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/log.h>
#include "mk_uwb.hpp"
#include "uwb_commands.hpp"
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_uwb.h>

extern "C" __EXPORT int mk_uwb_main(int argc, char *argv[])
{
        PX4_INFO("Hello MK UWB");
	if (argc>1){
            if (strcmp(argv[1], "Start") == 0)
	    {
		mk_uwb::Robotics_protocol_manager(CMD_INIT_GET_PLATFORM_INFO);
	    }
	    else if(strcmp(argv[1], "StartRanging") == 0)
	    {
		mk_uwb::Robotics_protocol_manager(CMD_START_RANGING);
	    }
	    else if(strcmp(argv[1], "StopRanging") == 0)
	    {
		mk_uwb::Robotics_protocol_manager(CMD_STOP_RANGING);
	    }

	    else if(strcmp(argv[1], "SetConfig") == 0)
	    {
		if(argc>2){
			mk_uwb::Configure(argv[2],argv[3], argv[4]);
		};
		mk_uwb::Robotics_protocol_manager(CMD_SET_CONFIGS);
	    }

	    else if(strcmp(argv[1], "GetConfig") == 0)
	    {
		if(argc>2){
			mk_uwb::GetConfig(argv[2],argv[3]);
		};
		mk_uwb::Robotics_protocol_manager(CMD_GET_CONFIGS);
	    };
        }



















	else{
		// Subscirbe to "sensor_uwb", then set a polling interval of 200ms
		int uwb_sub = orb_subscribe(ORB_ID(sensor_uwb));
		orb_set_interval(uwb_sub, 200);

		// Configure a POSIX POLLIN system to sleep the current thread until
		// data appears on the topic
		px4_pollfd_struct_t fds_uwb;
		fds_uwb.fd = uwb_sub;
		fds_uwb.events = POLLIN;

		int counter = 20;
		printf("%02i | sessionid | \n", counter);
		printf("-----------------------------------\n");

			// Loop a specified number of times
		for(int i = 1; i <= counter; i++)
		{
			// Allow the POSIX POLLIN system to poll for data, with 1000ms timeout
			int poll_ret = px4_poll(&fds_uwb, 1, 1000);

			// If px4_poll returns 0, then the poll system timed out! Throw an error.
			if(poll_ret == 0)
			{
			PX4_ERR("Got no command within a second");
			}

			// If it didn't return 0, we got data!
			else
			{
			// Double check that the data we recieved was in the right format (I think - need to check)
			if(fds_uwb.revents & POLLIN)
			{

				// Create a sensor_gyro_s struct to store the data we recieved
				struct sensor_uwb_s uwb;

				// Copy the data over to the struct
				orb_copy(ORB_ID(sensor_uwb), uwb_sub, &uwb);

				if(uwb.uwb_anchor_cmd==CMD_SET_CONFIGS)
				{

					//mk_uwb::Configure(uwb.group_id, uwb.config_id,  uwb.config_value);
					mk_uwb::Robotics_protocol_manager(uwb.uwb_anchor_cmd);
				}
				else
				{
					mk_uwb::Robotics_protocol_manager(uwb.uwb_anchor_cmd);
				}
			}
			}
		}
	};
        return 0; // return of main function
}
