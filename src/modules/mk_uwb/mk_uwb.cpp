#include <inttypes.h>
#include <fcntl.h>
#include <termios.h>
#include <lib/crc/crc.h>
#include <string.h>


#include <float.h>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <errno.h>
#include <fcntl.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <ctype.h>
#include "mk_uwb.hpp"
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include "uwb_commands.hpp"

#include <uORB/uORB.h>
#include <uORB/topics/sensor_uwb.h>



/* Configuration Constants */
#define MK_UWB_MAX_PAYLOAD 256
uint8_t                         _sensor_state{0};
int 				uart_fd{-1};


MK_UWB::MK_UWB()
	: ModuleParams(nullptr)
{
}

MK_UWB::~MK_UWB()
{
	printf("UWB: Ranging Stopped\t\n");
	_sensor_state=STOP_RANGING;
	collectData();
	//close(uart_fd);
}

bool MK_UWB::start()
{
	/* schedule a cycle to start things */
	//ScheduleNow();
	parameters_update();
	int current_target_anchor = target_anchor.get();
    	PX4_INFO("Running with target anchor: %d", current_target_anchor);

	if (_sensor_state!=DEVICE_DISCOVERED){
		_sensor_state = START_DISCOVERY;
		collectData();
		if (_sensor_state!=DEVICE_DISCOVERED){
			_sensor_state = STOP_DISCOVERY;
			collectData();
			return false;
		}
	}

	if (_sensor_state==DEVICE_DISCOVERED){
		_sensor_state = STOP_DISCOVERY;
		collectData();

		_sensor_state = START_RANGING;
		collectData();

	}

	if (rx_field.Type == NTF_UWB_CHANGE_STATE && rx_field.Length == 0x26 && rx_field.Value[0] == 0x01) {
		return true;
	}
	else{
		usleep(10000);
		_sensor_state = START_RANGING;
		collectData();

		if (rx_field.Type == NTF_UWB_CHANGE_STATE && rx_field.Length == 0x26 && rx_field.Value[0] == 0x01) {
			return true;
		}
		else{
			return false;
		}
	}
}

bool MK_UWB::init()
{
	// alternatively, Run on fixed interval
	// ScheduleOnInterval(5000_us); // 2000 us interval, 200 Hz rate
	const char *serial_port = "/dev/ttyS6"; // Serial port device file
	// Open the serial port

	uart_fd = open(serial_port, O_RDWR | O_NOCTTY | O_NDELAY);

	if (uart_fd == -1) {
		perror("Unable to open serial port");
		return false;
	}

	// Configure the serial port
	struct termios uart_config;
	int termios_state;

	// Get the current configuration of the serial port
	if ((termios_state = tcgetattr(uart_fd, &uart_config)) < 0) {
		perror("Failed to get UART configuration");
		close(uart_fd);
		return false;
	}

	// Set the baud rate
	cfsetispeed(&uart_config, B115200);
	cfsetospeed(&uart_config, B115200);

	// Apply the serial port configuration
	if ((termios_state = tcsetattr(uart_fd, TCSANOW, &uart_config)) < 0) {
		perror("Failed to apply UART configuration");
		close(uart_fd);
		return false;
	}

	_sensor_state = UWB_NOT_INITIALIZED;
	collectData();
	usleep(10000); // Wait for 10 milliseconds

	_sensor_state = SCANNER_CONFIG;
	collectData();
	usleep(10000); // Wait for 10 milliseconds

	//DEVICE_UUID SET TO 80086ab8c6880990264c107ddb38555a
	_sensor_state = UUID_CONFIG;
	collectData();
	return true;
}

void MK_UWB::stop()
{
	//ScheduleClear();
}

void MK_UWB::Run()
{
	if (should_exit()) {
		//ScheduleClear();
		exit_and_cleanup();
		return;
	}

	//PX4_INFO("sensor state %u \n", _sensor_state);


	/* perform collection */
	_sensor_state = RECEIVE_RANGING_DATA;
	collectData();
}

int MK_UWB::collectData()
{
	switch (_sensor_state) {

	// sensor state 0 not initialized
	case UWB_NOT_INITIALIZED: {
			mk_uwb::read_bytes(7);
			unsigned char hex_data[] = {CMD_INIT_UWBS, 0x00, 0x00};
			size_t data_length = sizeof(hex_data);
			mk_uwb::send(hex_data, data_length);
			mk_uwb::read_bytes(5);

			if (rx_field.Type == CMD_INIT_UWBS && rx_field.Length == 2 && rx_field.Value[0] == 0x90 && rx_field.Value[1] == 0x00) {
				mk_uwb::read_bytes(7);
				_sensor_state = UWBS_READY;

			} else {
				PX4_WARN("Response not matched");
				// ("%02X ", rx_field.Type);
				// PX4_INFO("%02X ", rx_field.Length);
				// PX4_INFO("%02X ", rx_field.Value[0]);
				// PX4_INFO("%02X \n", rx_field.Value[1]);
			}

			break;
		}

	//sensor state 30 START_DISCOVERY
	case START_DISCOVERY: {
			unsigned char hex_data[] = {CMD_START_DISCOVERY, 0x00, 0x00};
			size_t data_length = sizeof(hex_data);
			mk_uwb::send(hex_data, data_length);
			mk_uwb::read_bytes(5);

			if (rx_field.Type == CMD_START_DISCOVERY && rx_field.Length == 2 && rx_field.Value[0] == 0x90
			    && rx_field.Value[1] == 0x00) {
				_sensor_state = DISCOVERING;
				PX4_INFO("Discovering \n");
				mk_uwb::read_bytes(26);

				if (rx_field.Type == NTF_DISCOVERED_DEVICE && rx_field.Length == 23) {
					_sensor_state = DEVICE_DISCOVERED;
					PX4_INFO("Discovered device ID\n");

					for (int i = 0; i < 16; ++i) {
						printf("%02X ", rx_field.Value[i + 3]);
						config_field.device1_uuid[i] = rx_field.Value[i + 3];
					}

					printf("\n");
					int device_exist = mk_uwb::read_bytes(26);

					if (device_exist == 1 && rx_field.Type == NTF_DISCOVERED_DEVICE && rx_field.Length == 23) {
						PX4_INFO("Discovered device ID\n");

						for (int i = 0; i < 16; ++i) {
							printf("%02X ", rx_field.Value[i + 3]);
							config_field.device2_uuid[i] = rx_field.Value[i + 3];
						}

						printf("\n");

					} else {
						PX4_WARN("Second device not found");
					}
				} else {
					PX4_WARN("Device not found");
				}

			} else {
				PX4_WARN("Response not matched");
				PX4_INFO("%02X ", rx_field.Type);
				PX4_INFO("%02X ", rx_field.Length);
				PX4_INFO("%02X ", rx_field.Value[0]);
				PX4_INFO("%02X \n", rx_field.Value[1]);
			}

			break;
		}

	//sensor state 2 STOP_DISCOVERY
	case STOP_DISCOVERY: {
			unsigned char hex_data[] = {CMD_STOP_DISCOVERY, 0x00, 0x00};
			size_t data_length = sizeof(hex_data);
			mk_uwb::send(hex_data, data_length);
			mk_uwb::read_bytes(5);
			break;
		}

	//sensor state 2 START_RANGING
	case START_RANGING: {
			if (target_anchor.get() == 1) {
				if (config_field.device1_uuid[15]==0x01 || config_field.device2_uuid[15]==0x01){
					unsigned char hex_data1[] = {CMD_START_UWB_RANGING, 0x00, 0x13, 0x03, 0x00, 0x10, config_field.device1_uuid[0], config_field.device1_uuid[1], config_field.device1_uuid[2], config_field.device1_uuid[3], config_field.device1_uuid[4], config_field.device1_uuid[5], config_field.device1_uuid[6], config_field.device1_uuid[7], config_field.device1_uuid[8], config_field.device1_uuid[9], config_field.device1_uuid[10], config_field.device1_uuid[11], config_field.device1_uuid[12], config_field.device1_uuid[13], config_field.device1_uuid[14], 0x01};
					size_t data_length = sizeof(hex_data1);
					mk_uwb::send(hex_data1, data_length);
				}
				else{
					break;
				}

			} else if (target_anchor.get() == 2) {
				if (config_field.device1_uuid[15]==0x02 || config_field.device2_uuid[15]==0x02){
					unsigned char hex_data2[] = {CMD_START_UWB_RANGING, 0x00, 0x13, 0x03, 0x00, 0x10, config_field.device1_uuid[0], config_field.device1_uuid[1], config_field.device1_uuid[2], config_field.device1_uuid[3], config_field.device1_uuid[4], config_field.device1_uuid[5], config_field.device1_uuid[6], config_field.device1_uuid[7], config_field.device1_uuid[8], config_field.device1_uuid[9], config_field.device1_uuid[10], config_field.device1_uuid[11], config_field.device1_uuid[12], config_field.device1_uuid[13], config_field.device1_uuid[14], 0x02};
					size_t data_length = sizeof(hex_data2);
					mk_uwb::send(hex_data2, data_length);
				}
				else{
					break;
				}

			} else {
				PX4_ERR("Target Anchor doesnt exist");
				break;
			}

			//CMD_START_RANGING

			mk_uwb::read_bytes(5);
			mk_uwb::read_bytes(41);

			// Clear the structure by filling it with 0s in memory
			memset(&sensor_uwb, 0, sizeof(sensor_uwb));

			break;
		}

	case RECEIVE_RANGING_DATA: {

		//Need to change, now they are hardcoded
		sensor_uwb.orientation		= ROTATION_NONE;
		sensor_uwb.offset_x		= 0;
		sensor_uwb.offset_y		= 0;
		sensor_uwb.offset_z		= 0;


		//usleep(4000);
		if (mk_uwb::read_bytes(0x5A) == 0) {
			break;
		};

		size_t i = 0;

		while (i < 256) {
			uint8_t tag = rx_field.Value[i];

			if (tag == 0x00) {
				break;
			}

			i += 1;
			uint16_t length = (rx_field.Value[i] * 256) + rx_field.Value[i + 1];
			i += 2;

			if (tag == 0x05) {
				for (uint16_t j = 0; j < length; ++j) {
					distance_result.MAC[j] = rx_field.Value[i + j];
				}

			} else if (tag == 0x09) {
				distance_result.distance = (rx_field.Value[i] << 8) + rx_field.Value[i + 1];
				printf("The distance is %hu cm \n", distance_result.distance);

			} else if (tag == 0x0B) {
				uint8_t Azimuth_bytes[2] = {rx_field.Value[i], rx_field.Value[i + 1]};
				distance_result.aoa_azimuth = (int16_t)((Azimuth_bytes[0] << 8) | Azimuth_bytes[1]);
				printf("The azimuth is %d degree\n", -distance_result.aoa_azimuth);

			} else if (tag == 0x0C) {
				uint8_t Elevation_bytes[2] = {rx_field.Value[i], rx_field.Value[i + 1]};
				distance_result.aoa_elevation = (int16_t)((Elevation_bytes[0] << 8) | Elevation_bytes[1]);
				printf("The elevation is %d degree\n", -distance_result.aoa_elevation);
			}

			i += length;
		}
		sensor_uwb.timestamp 		= hrt_absolute_time();
		sensor_uwb.distance 		= double(distance_result.distance) /100;
		sensor_uwb.aoa_azimuth_dev 	= - double(distance_result.aoa_azimuth)  ;
		sensor_uwb.aoa_elevation_dev 	= - double(distance_result.aoa_elevation) ;
		orb_publish(ORB_ID(sensor_uwb), sensor_uwb_pub, &sensor_uwb) ;

		break;
	}

	//sensor state 3 UWB_RANGING_STARTED
	case STOP_RANGING: {
			unsigned char hex_data[] = {0x04, 0x00, 0x00};
			//CMD_STOP_RANGING
			size_t data_length = sizeof(hex_data);
			mk_uwb::send(hex_data, data_length);
			mk_uwb::read_bytes(5);
			mk_uwb::read_bytes(41);
			_sensor_state = UWB_RANGING_STOPPED;
			break;
		}

	//sensor state 4 SET_CONFIG
	case SET_CONFIG: {
			unsigned char hex_data[] = {0x05, 0x00, 0x0C,
						    0x01, 0x00, 0x01, static_cast<unsigned char>(config_field.Group_ID),
						    0x02, 0x00, 0x01, static_cast<unsigned char>(config_field.Config_ID),
						    0x03, 0x00, 0x01,  static_cast<unsigned char>(config_field.Config_Value)
						   };
			//CMD_SET_CONFIG
			size_t data_length = sizeof(hex_data);
			mk_uwb::send(hex_data, data_length);
			break;
		}

	case GET_CONFIG: {
			unsigned char hex_data[] = {CMD_GET_CONFIGS, 0x00, 0x08,
						    0x01, 0x00, 0x01, static_cast<unsigned char>(config_field.Group_ID),
						    0x02, 0x00, 0x01, static_cast<unsigned char>(config_field.Config_ID),
						   };
			//CMD_SET_CONFIG
			size_t data_length = sizeof(hex_data);
			mk_uwb::send(hex_data, data_length);
			mk_uwb::read_bytes(0x18);
			break;
		}
	case SCANNER_CONFIG: {
			unsigned char hex_data[] = {0x05, 0x00, 0x0C,
						    0x01, 0x00, 0x01, 0x00,
						    0x02, 0x00, 0x01, 0x05,
						    0x03, 0x00, 0x01, 0x02
						   };
			//CMD_SET_CONFIG
			size_t data_length = sizeof(hex_data);
			mk_uwb::send(hex_data, data_length);
			if (mk_uwb::read_bytes(5) == 1){
				PX4_INFO("SCANNER CONFIGURED");
			}
			break;
		}

		// default:
		// 	break;
		// }


	case UUID_CONFIG: {
			unsigned char hex_data[] = {0x05, 0x00, 0x1B,
						    0x01, 0x00, 0x01, 0x00,
						    0x02, 0x00, 0x01, 0x04,
						    0x03, 0x00, 0x10, 0x80, 0x08, 0x6A, 0xB8, 0xC6, 0x88, 0x09, 0x90, 0x26, 0x4C, 0x10, 0x7D, 0xDB, 0x38, 0x55, 0x5A
						   };
			//CMD_SET_CONFIG
			size_t data_length = sizeof(hex_data);
			mk_uwb::send(hex_data, data_length);
			if (mk_uwb::read_bytes(5) == 1){
				PX4_INFO("DEVICE UUID CONFIGURED");
			}
			unsigned char hex_data2[] = {0x05, 0x00, 0x1B,
						    0x01, 0x00, 0x01, 0x00,
						    0x02, 0x00, 0x01, 0x03,
						    0x03, 0x00, 0x10, 0x00, 0x00, 0xFE, 0xE9, 0x00, 0x00, 0x10, 0x00, 0x91, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB
						   };
			//CMD_SET_CONFIG
			size_t data_length2 = sizeof(hex_data2);
			mk_uwb::send(hex_data2, data_length2);
			if (mk_uwb::read_bytes(5) == 1){
				PX4_INFO("OOB UUID CONFIGURED");
			}
			break;
	}

	}
	return 1;
}















int MK_UWB::custom_command(int argc, char *argv[])
{

	return print_usage("unknown command");
}

void MK_UWB::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();
	}
	target_anchor.update();
}

int MK_UWB::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}























namespace mk_uwb
{

int request_handle(uint8_t *input_buf, const int data_length)
{
	rx_field.Type = input_buf[0];
	rx_field.Length = input_buf[1] * 256 + input_buf[2];
	memset(rx_field.Value, 0, sizeof(rx_field.Value));

	for (int i = 0; i < rx_field.Length; ++i) {
		rx_field.Value[i] = input_buf[i + 3];
	}

	return 0;
};

void send(const unsigned char *hex_data, size_t data_length)
{
	ssize_t ret = ::write(uart_fd, hex_data, data_length);

	if (ret != sizeof(hex_data)) {
		PX4_DEBUG("write fail %d", ret);
	}

	// Log what has been sent to the console
	PX4_INFO("Data sent: ");

	for (size_t i = 0; i < data_length; i++) {
		printf("%02X ", hex_data[i]); // Print in hexadecimal format
	}

	printf("\n");

};

int read_bytes(const int data_length)
{
	static uint8_t recv_buffer[MK_UWB_MAX_PAYLOAD];// A maximum payload assumed

	ssize_t bytes_read = 0;
	ssize_t total_bytes_read = 0;
	// Define a timeout in microseconds
	const hrt_abstime timeout_usec = 10000000; // 10 seconds timeout
	hrt_abstime start_time = hrt_absolute_time();
	bool received = false;

	while (!received) {
		// Check for timeout
		if (hrt_absolute_time() - start_time >= timeout_usec) {
			PX4_ERR("Timeout waiting for data");
			return 0;
		}

		// Try to read data
		bytes_read = read(uart_fd, recv_buffer + total_bytes_read, data_length - total_bytes_read);

		if (bytes_read > 0) {
			total_bytes_read += bytes_read;

			if (total_bytes_read == data_length) {
				received = true;
			}

		}

		else {
			// No data read, add a delay before trying again
			usleep(10000); // Wait for 10 milliseconds
		}
	}

	if (received) {
		PX4_INFO("Response recieved \n");

		for (int i = 0; i < data_length; i++) {
			printf("%02X ", recv_buffer[i]); // Use %02X for hexadecimal format
		}

		printf("\n");

		request_handle(recv_buffer, data_length);
		return 1;

	} else {
		return 0;
	}
};

}

