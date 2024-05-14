#pragma once
#ifndef PX4_RDDRONE_H
#define PX4_RDDRONE_H
#include <termios.h>
#include <poll.h>
#include <sys/select.h>
#include <sys/time.h>
#include <perf/perf_counter.h>
#include <lib/conversion/rotation.h>

#include <px4_platform_common/module_params.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/sensor_uwb.h>
#include <uORB/topics/parameter_update.h>

#include <matrix/math.hpp>

#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/SubscriptionInterval.hpp>


using namespace time_literals;

//extern "C" __EXPORT int template_module_main(int argc, char *argv[]);


class MK_UWB : public ModuleBase<MK_UWB>, public ModuleParams
{
public:
	MK_UWB();
	~MK_UWB();

	/**
	 * @see ModuleBase::task_spawn
	 */
	static int task_spawn(int argc, char *argv[]);

	/**
	 * @see ModuleBase::custom_command
	 */
	static int custom_command(int argc, char *argv[]);

	/**
	 * @see ModuleBase::print_usage
	 */
	static int print_usage(const char *reason = nullptr);

	bool init();

	bool start();

	void stop();

	int collectData();

	void Run() ;

private:
	void parameters_update();

	struct sensor_uwb_s sensor_uwb;
	// Create a uORB topic advertisement
	orb_advert_t sensor_uwb_pub = orb_advertise(ORB_ID(sensor_uwb), &sensor_uwb);


	// Publications
	// uORB::Publication<sensor_uwb_s> sensor_uwb_pub{ORB_ID(sensor_uwb)};

	// Subscriptions
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};


};
#endif //PX4_RDDRONE_H













namespace mk_uwb
{
	void openAndConfigureSerialPort(const char *serial_port, int *uart_fd);

	void send(const unsigned char *hex_data, size_t data_length);

	int request_handle(uint8_t *input_buf, const int data_length);

	int read_bytes(const int data_length);

	int measure();

	void collect(int *uart_fd);

	void Configure(const char* Group_ID, const char* Config_ID, const char* Config_Value);

	void GetConfig(const char* Group_ID, const char* Config_ID);

	void Robotics_protocol_manager(const int option);

	void TargetAnchor(const char *Target_Anchor);
};



