#pragma once
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


namespace mk_uwb
{
	void openAndConfigureSerialPort(const char *serial_port, int *uart_fd);

	void send(int *uart_fd, const unsigned char *hex_data);

	int request_handle(uint8_t *input_buf, const int data_length);

	int read_bytes(const int data_length);

	int measure();

	void collect(int *uart_fd);

	void Configure(const char* Group_ID, const char* Config_ID, const char* Config_Value);

	void GetConfig(const char* Group_ID, const char* Config_ID);

	void Robotics_protocol_manager(const int option);

	void TargetAnchor(const char *Target_Anchor);
};




