#include <inttypes.h>
#include <fcntl.h>
#include <termios.h>
#include <lib/crc/crc.h>

#include <float.h>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/log.h>
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
#include <uORB/topics/vehicle_land_detected.h>
#include "mk_uwb.hpp"

static bool thread_should_exit = false;	/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;			/**< Handle of daemon task / thread */

/* Run main loop at this rate in Hz. */
static constexpr uint32_t mk_uwb_UPDATE_RATE_HZ = 50;

/**
 * Landing target position estimator app start / stop handling function
 * This makes the module accessible from the nuttx shell
 * @ingroup apps
 */
extern "C" __EXPORT int mk_uwb_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int mk_uwb_thread_main(int argc, char *argv[]);

/**
* Main entry point for this module
**/
int mk_uwb_main(int argc, char *argv[])
{

	if (argc < 2) {
		goto exiterr;
	}

	if (argc >= 2 && !strcmp(argv[1], "start")) {
		if (thread_running) {
			PX4_INFO("already running");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("mk_uwb",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2100,
						 mk_uwb_thread_main,
						 (argv) ? (char *const *)&argv[2] : nullptr);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;

		if (!thread_running) {
			PX4_WARN("mk_uwb not running");
		}

		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			PX4_INFO("running");

		} else {
			PX4_INFO("not started");
		}

		return 0;
	}

exiterr:
	PX4_WARN("usage: mk_uwb {start|stop|status}");
	return 1;
}

int mk_uwb_thread_main(int argc, char *argv[])
{
	vehicle_land_detected_s vehicle_land_detected;
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	bool landed= true;
	PX4_DEBUG("starting");

	thread_running = true;
	MK_UWB est;
	est.init();

	bool start_success= false;

	while (!thread_should_exit) {
		est.parameters_update();
		//Determine the state
		if (_vehicle_land_detected_sub.update(&vehicle_land_detected)) {
			landed = vehicle_land_detected.landed;
		}
		//On ground state
		if (landed == true){
			px4_usleep(100000);
			//If ranging started will be stopped
			if (start_success){
				est.~MK_UWB();
				start_success = false;
			}
		}
		//Flying state
		else{
			//First time start
			if (start_success == false){
				start_success = est.start();
				if (start_success){
					PX4_INFO("UWBS RANGING START SUCCESS");
				}
				else{
					PX4_ERR("UWBS RANGING START FAILED");
				}
			}
			//Already ranging
			else{
				est.Run();
			}
		}
		px4_usleep(1000000 / mk_uwb_UPDATE_RATE_HZ);
	}

	thread_running = false;
	return 0;
}


