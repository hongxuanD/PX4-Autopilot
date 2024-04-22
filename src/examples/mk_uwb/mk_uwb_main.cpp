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
	if (argc>1){
            if (strcmp(argv[1], "Start") == 0)
	    {
		mk_uwb::Robotics_protocol_manager(CMD_START_RANGING);
	    }
	    else if(strcmp(argv[1], "StartRanging") == 0)
	    {
		if(argc>=2){
			mk_uwb::TargetAnchor(argv[2]);
			mk_uwb::Robotics_protocol_manager(CMD_TEST_RANGING);
		}
		else{
			PX4_ERR("Wrong use of the command");
		}
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
        return 0; // return of main function
}
