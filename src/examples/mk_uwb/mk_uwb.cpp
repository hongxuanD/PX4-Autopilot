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
#include <errno.h>
#include <fcntl.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <ctype.h>
#include "mk_uwb.hpp"
#include "uwb_commands.hpp"

#include <uORB/uORB.h>
#include <uORB/topics/sensor_uwb.h>

/* Configuration Constants */
#define MK_UWB_MAX_PAYLOAD 256

uint8_t                         _sensor_state{0};
int 				uart_fd{-1};


namespace mk_uwb
{

void openAndConfigureSerialPort(const char *serial_port)
{
    // Open the serial port
    uart_fd = open(serial_port, O_RDWR | O_NOCTTY | O_NDELAY);

    if (uart_fd == -1) {
        perror("Unable to open serial port");
        return;
    }

    // Configure the serial port
    struct termios uart_config;
    int termios_state;

    // Get the current configuration of the serial port
    if ((termios_state = tcgetattr(uart_fd, &uart_config)) < 0) {
        perror("Failed to get UART configuration");
        close(uart_fd);
        return;
    }

    // Set the baud rate
    cfsetispeed(&uart_config, B115200);
    cfsetospeed(&uart_config, B115200);

    // Apply the serial port configuration
    if ((termios_state = tcsetattr(uart_fd, TCSANOW, &uart_config)) < 0) {
        perror("Failed to apply UART configuration");
        close(uart_fd);
        return;
    }
};

int request_handle(uint8_t *input_buf, const int data_length)
{
	rx_field.Type= input_buf[0];
	rx_field.Length= input_buf[1]*256 + input_buf[2];
	memset(rx_field.Value, 0 , sizeof(rx_field.Value));
	for (int i= 0; i< rx_field.Length; ++i)
	{
		rx_field.Value[i]=input_buf[i+3];
	}
	// PX4_INFO("The type is %d \n", rx_field.Type);
	// PX4_INFO("The length is %d \n", rx_field.Length);
	return 0;
};

void send(const unsigned char* hex_data, size_t data_length)
{
	ssize_t ret = ::write(uart_fd, hex_data, data_length);
	if (ret != sizeof(hex_data)) {
		PX4_DEBUG("write fail %d", ret);
	}

        // Log what has been sent to the console
        PX4_INFO("Data sent: ");
        for (size_t i = 0; i < data_length; i++) {
            PX4_INFO("%02X ", hex_data[i]); // Print in hexadecimal format
        }
        PX4_INFO("\n");

};

int read_bytes(const int data_length)
{
	static uint8_t recv_buffer[MK_UWB_MAX_PAYLOAD];// A maximum payload assumed

	ssize_t bytes_read = 0;
	ssize_t total_bytes_read = 0;
	// Define a timeout in microseconds
	const hrt_abstime timeout_usec = 5000000; // 5 seconds timeout
	hrt_abstime start_time = hrt_absolute_time();
	bool received = false;

	while (!received)
	{
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

	if (received)
	{
		PX4_INFO("Response recieved \n");
		for (int i = 0; i < data_length; i++) {
		printf("%02X ", recv_buffer[i]); // Use %02X for hexadecimal format
		}
		printf("\n");

		request_handle(recv_buffer, data_length);
		return 1;
	}
	else{
		return 0;
	}
};

void collect()
{
	//Response of CMD_STOP_DISCOVERY
	if (_sensor_state == STOP_DISCOVERY) {
		mk_uwb::read_bytes(5);
	}

	//Response of CMD_START_RANGING
	else if (_sensor_state == START_RANGING) {
		mk_uwb::read_bytes(5);

	}

	//Response of CMD_STOP_RANGING
	else if (_sensor_state == STOP_RANGING){
		int i=0;
		while(i<5){
			usleep(10000);
			mk_uwb::read_bytes(0x47);
			i=i+1;
		}
	}

	//Response of CMD_SET_CONFIG
	else if (_sensor_state == SET_CONFIG){
		mk_uwb::read_bytes(5);
	}

	else if (_sensor_state == GET_CONFIG){
		mk_uwb::read_bytes(0x16);
	}
};

int measure()
{

	switch (_sensor_state) {

	// sensor state 0 not initialized
	case UWB_NOT_INITIALIZED:{
		unsigned char hex_data[] = {CMD_INIT_UWBS, 0x00, 0x00};
		size_t data_length = sizeof(hex_data);
    		send(hex_data, data_length);
		mk_uwb::read_bytes(5);
		if(rx_field.Type == CMD_INIT_UWBS && rx_field.Length == 2 && rx_field.Value[0] == 0x90 && rx_field.Value[1] == 0x00){
			mk_uwb::read_bytes(7);
			_sensor_state=UWBS_READY;
		}
		else{
			PX4_WARN("Response not matched");
			// ("%02X ", rx_field.Type);
			// PX4_INFO("%02X ", rx_field.Length);
			// PX4_INFO("%02X ", rx_field.Value[0]);
			// PX4_INFO("%02X \n", rx_field.Value[1]);
		}
		break;
	}

	//sensor state 30 START_DISCOVERY
	case START_DISCOVERY:{
		unsigned char hex_data[] = {CMD_START_DISCOVERY, 0x00, 0x00};
		size_t data_length = sizeof(hex_data);
    		send(hex_data, data_length);
		mk_uwb::read_bytes(5);
		if(rx_field.Type == CMD_START_DISCOVERY && rx_field.Length == 2 && rx_field.Value[0] == 0x90 && rx_field.Value[1] == 0x00){
			_sensor_state=DISCOVERING;
			PX4_INFO("Discovering \n");
			mk_uwb::read_bytes(26);
			if(rx_field.Type == NTF_DISCOVERED_DEVICE && rx_field.Length == 23){
				_sensor_state=DEVICE_DISCOVERED;
				PX4_INFO("Discovered device ID\n");
				for (int i= 0; i< 16; ++i){
					PX4_INFO("%02X ", rx_field.Value[i+3]);
					config_field.device1_uuid[i]=rx_field.Value[i+3];
				}
				PX4_INFO("\n");
				int device_exist = mk_uwb::read_bytes(26);
				if(device_exist ==1 && rx_field.Type == NTF_DISCOVERED_DEVICE && rx_field.Length == 23){
					PX4_INFO("Discovered device ID\n");
					for (int i= 0; i< 16; ++i){
						PX4_INFO("%02X ", rx_field.Value[i+3]);
						config_field.device2_uuid[i]=rx_field.Value[i+3];
					}
					PX4_INFO("\n");
				}
				else{
					PX4_WARN("Second device not found");
				}
			}
			else{
			PX4_WARN("Device not found");
			}
		}
		else{
			PX4_WARN("Response not matched");
			PX4_INFO("%02X ", rx_field.Type);
			PX4_INFO("%02X ", rx_field.Length);
			PX4_INFO("%02X ", rx_field.Value[0]);
			PX4_INFO("%02X \n", rx_field.Value[1]);
		}
		break;
	}

	//sensor state 2 STOP_DISCOVERY
	case STOP_DISCOVERY:{
		unsigned char hex_data[] = {CMD_STOP_DISCOVERY, 0x00, 0x00};
		size_t data_length = sizeof(hex_data);
    		send(hex_data, data_length);
		mk_uwb::read_bytes(5);
		break;
	}

	//sensor state 2 START_RANGING
	case START_RANGING:{
		unsigned char hex_data[] = {CMD_START_UWB_RANGING, 0x00, 0x13, 0x03, 0x00, 0x10, config_field.device1_uuid[0], config_field.device1_uuid[1], config_field.device1_uuid[2], config_field.device1_uuid[3], config_field.device1_uuid[4], config_field.device1_uuid[5], config_field.device1_uuid[6], config_field.device1_uuid[7], config_field.device1_uuid[8], config_field.device1_uuid[9], config_field.device1_uuid[10], config_field.device1_uuid[11], config_field.device1_uuid[12], config_field.device1_uuid[13], config_field.device1_uuid[14], config_field.device1_uuid[15]};
		//CMD_START_RANGING
		size_t data_length = sizeof(hex_data);
    		send(hex_data, data_length);
		mk_uwb::read_bytes(5);
		mk_uwb::read_bytes(41);
		struct sensor_uwb_s sensor_uwb;

		// Clear the structure by filling it with 0s in memory
		memset(&sensor_uwb, 0, sizeof(sensor_uwb));

		// Create a uORB topic advertisement
		orb_advert_t sensor_uwb_pub = orb_advertise(ORB_ID(sensor_uwb), &sensor_uwb);

		sensor_uwb.timestamp 		= hrt_absolute_time();
		//Need to change, now they are hardcoded
		sensor_uwb.orientation		= ROTATION_NONE;
		sensor_uwb.offset_x		= 0;
		sensor_uwb.offset_y		= 0;
		sensor_uwb.offset_z		= 0;

		if (rx_field.Type == NTF_UWB_CHANGE_STATE && rx_field.Length == 0x26)
		{
			int index=0;
			while(index<500){
				//usleep(4000);
				if(mk_uwb::read_bytes(0x47)==0){
					break;
				};
				size_t i=0;
				while (i < 256){
					uint8_t tag =rx_field.Value[i];
					if(tag == 0x00){
						break;
					}
					i+=1;
					uint16_t length= (rx_field.Value[i] *256)+ rx_field.Value[i + 1];
					i +=2;
					if(tag == 0x05){
						for (uint16_t j = 0; j<length; ++j){
							distance_result.MAC[j] =rx_field.Value[i + j];
						}
					}
					else if(tag == 0x09){
						distance_result.distance=(rx_field.Value[i] <<8)+ rx_field.Value[i + 1];
						printf("The distance is %hu cm \n", distance_result.distance);
					}
					else if(tag == 0x0B){
						uint8_t Azimuth_bytes[2] = {rx_field.Value[i] , rx_field.Value[i+1]};
						distance_result.aoa_azimuth = (int16_t)((Azimuth_bytes[0] << 8) | Azimuth_bytes[1]);
						printf("The azimuth is %d degree\n", distance_result.aoa_azimuth);
					}
					else if(tag == 0x0C){
						uint8_t Elevation_bytes[2] = {rx_field.Value[i] , rx_field.Value[i+1]};
						distance_result.aoa_elevation = (int16_t)((Elevation_bytes[0] << 8) | Elevation_bytes[1]);
						printf("The elevation is %d degree\n", distance_result.aoa_elevation);
					}

					i+= length;

					sensor_uwb.distance 		= double(distance_result.distance) / 100;
					sensor_uwb.aoa_azimuth_dev 	= - double(distance_result.aoa_azimuth) / 128;
					sensor_uwb.aoa_elevation_dev 	= - double(distance_result.aoa_elevation) / 128;
       					 orb_publish(ORB_ID(sensor_uwb), sensor_uwb_pub, &sensor_uwb);
				}

				index=index+1;
				printf("The index is %hu \n", index);

			}
		}

		break;
	}
	//sensor state 3 UWB_RANGING_STARTED
	case STOP_RANGING:{
		unsigned char hex_data[] = {0x04, 0x00, 0x00};
		//CMD_STOP_RANGING
		size_t data_length = sizeof(hex_data);
    		send(hex_data, data_length);
		mk_uwb::read_bytes(5);
		mk_uwb::read_bytes(41);
		_sensor_state = UWB_RANGING_STOPPED;
		break;
	}
	//sensor state 4 SET_CONFIG
	case SET_CONFIG:{
		unsigned char hex_data[] = {0x05, 0x00, 0x0C,
				0x01, 0x00, 0x01, static_cast<unsigned char>(config_field.Group_ID),
                         	0x02, 0x00, 0x01,static_cast<unsigned char>(config_field.Config_ID),
                          	0x03, 0x00, 0x01,  static_cast<unsigned char>(config_field.Config_Value)};
		//CMD_SET_CONFIG
		size_t data_length = sizeof(hex_data);
    		send(hex_data, data_length);
		mk_uwb::collect();
		break;
	}
	case GET_CONFIG:{
		unsigned char hex_data[] = {CMD_GET_CONFIGS, 0x00, 0x08,
				0x01, 0x00, 0x01, static_cast<unsigned char>(config_field.Group_ID),
                         	0x02, 0x00, 0x01,static_cast<unsigned char>(config_field.Config_ID),
                          	};
		//CMD_SET_CONFIG
		size_t data_length = sizeof(hex_data);
    		send(hex_data, data_length);
		mk_uwb::collect();
		break;
	}
	// default:
	// 	break;
	// }
	}
	return PX4_OK;
};

void Configure(const char* Group_ID, const char* Config_ID, const char* Config_Value)
{
	config_field.Group_ID=atoi(Group_ID);
	config_field.Config_ID=atoi(Config_ID);
	config_field.Config_Value=atoi(Config_Value);
};

void GetConfig(const char* Group_ID, const char* Config_ID)
{
	config_field.Group_ID=atoi(Group_ID);
	config_field.Config_ID=atoi(Config_ID);
};

void Robotics_protocol_manager(const int option)
{
	PX4_INFO("sensor state %u \n", _sensor_state);
	const char *serial_port = "/dev/ttyS6"; // Serial port device file
	mk_uwb::openAndConfigureSerialPort(serial_port);
	usleep(100000);

	if (option==CMD_START_RANGING)
	{
		_sensor_state = UWB_NOT_INITIALIZED;
		mk_uwb::measure();
		if(_sensor_state==UWBS_READY){
			_sensor_state=START_DISCOVERY;
			mk_uwb::measure();
			// if(_sensor_state==DEVICE_DISCOVERED){
			// 	_sensor_state=STOP_DISCOVERY;
			// 	mk_uwb::measure();
			// 	_sensor_state=START_RANGING;
			// 	mk_uwb::measure();
			// 	_sensor_state=STOP_RANGING;
			// 	mk_uwb::measure();
			// }
		}
	}

	else if(option==CMD_TEST_RANGING)
	{
		_sensor_state = UWB_NOT_INITIALIZED;
		mk_uwb::measure();
		if(_sensor_state==UWBS_READY){
			_sensor_state=START_RANGING;
			mk_uwb::measure();
		}
	}

	else if(option==CMD_SET_CONFIGS)
	{
		_sensor_state = SET_CONFIG;
		mk_uwb::measure();
	}

	else if(option==CMD_GET_CONFIGS)
	{
		_sensor_state = GET_CONFIG;
		mk_uwb::measure();
	}

	// Close the serial port
	close(uart_fd);
};

};


