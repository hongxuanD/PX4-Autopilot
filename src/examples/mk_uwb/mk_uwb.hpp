#pragma once
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <drivers/drv_hrt.h>
#include <drivers/device/device.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
namespace mk_uwb
{
	void openAndConfigureSerialPort(const char *serial_port, int *uart_fd);

	void send(int *uart_fd, const unsigned char *hex_data);

	int request_handle(uint8_t *input_buf, const int data_length);

	void read_bytes(const int data_length);

	int measure();

	void collect(int *uart_fd);

	void Configure(const char* Group_ID, const char* Config_ID, const char* Config_Value);

	void GetConfig(const char* Group_ID, const char* Config_ID);

	void Robotics_protocol_manager(const int option);
};



