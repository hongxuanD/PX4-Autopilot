#include <inttypes.h>
#include <fcntl.h>
#include <termios.h>
#include <lib/crc/crc.h>
#include <string.h>

#include <float.h>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/log.h>
#include "mk_uwb.hpp"
#include "uwb_commands.hpp"

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
	printf("The type is %d \n", rx_field.Type);
	printf("The length is %d \n", rx_field.Length);
	return 0;
};

void send(const unsigned char* hex_data, size_t data_length)
{
	ssize_t ret = ::write(uart_fd, hex_data, data_length);
	if (ret != sizeof(hex_data)) {
		PX4_DEBUG("write fail %d", ret);
	}

        // Log what has been sent to the console
        printf("Data sent: ");
        for (size_t i = 0; i < data_length; i++) {
            printf("%02X ", hex_data[i]); // Print in hexadecimal format
        }
        printf("\n");

};

void read_bytes(const int data_length)
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
			break;
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
		printf("Response recieved \n");
		for (int i = 0; i < data_length; i++) {
		printf("%02X ", recv_buffer[i]); // Use %02X for hexadecimal format
		}
		printf("\n");

		request_handle(recv_buffer, data_length);
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
	else if (_sensor_state == UWB_RANGING_STARTED){
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
			printf("%02X ", rx_field.Type);
			printf("%02X ", rx_field.Length);
			printf("%02X ", rx_field.Value[0]);
			printf("%02X \n", rx_field.Value[1]);
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
			printf("Discovering \n");
			mk_uwb::read_bytes(26);
			if(rx_field.Type == NTF_DISCOVERED_DEVICE && rx_field.Length == 23){
				_sensor_state=DEVICE_DISCOVERED;
				printf("Discovered device ID\n");
				for (int i= 0; i< 16; ++i){
					printf("%02X ", rx_field.Value[i+3]);
					config_field.device1_uuid[i]=rx_field.Value[i+3];
				}
				printf("\n");
			}
			else{
			PX4_WARN("Device not found");
			}
		}
		else{
			PX4_WARN("Response not matched");
			printf("%02X ", rx_field.Type);
			printf("%02X ", rx_field.Length);
			printf("%02X ", rx_field.Value[0]);
			printf("%02X \n", rx_field.Value[1]);
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
		if (rx_field.Type == NTF_UWB_CHANGE_STATE && rx_field.Length == 0x26)
		{
			int i=0;
			while(i<5){
				usleep(10000);
				mk_uwb::read_bytes(0x47);
				i=i+1;
			}
		}
		break;
	}
	//sensor state 3 UWB_RANGING_STARTED
	case 3:{
		unsigned char hex_data[] = {0x04, 0x00, 0x00};
		//CMD_STOP_RANGING
		size_t data_length = sizeof(hex_data);
    		send(hex_data, data_length);
		mk_uwb::collect();
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
	printf("sensor state %u \n", _sensor_state);
	const char *serial_port = "/dev/ttyS3"; // Serial port device file
	mk_uwb::openAndConfigureSerialPort(serial_port);

	if (option==CMD_INIT_GET_PLATFORM_INFO)
	{
		_sensor_state = UWB_NOT_INITIALIZED;
		mk_uwb::measure();
		if(_sensor_state==UWBS_READY){
			_sensor_state=START_DISCOVERY;
			mk_uwb::measure();
			if(_sensor_state==DEVICE_DISCOVERED){
				_sensor_state=STOP_DISCOVERY;
				mk_uwb::measure();
				_sensor_state=START_RANGING;
				mk_uwb::measure();
			}
		}
	}

	else if(option==CMD_START_RANGING)
	{
		if(_sensor_state == DEVICE_DISCOVERED)
		{
			_sensor_state = STOP_DISCOVERY;
			mk_uwb::measure();
		}
		else{
			PX4_ERR("NOT READY FOR RANGING");
		}

	}
	else if(option==CMD_STOP_RANGING)
	{
		if(_sensor_state == UWB_RANGING_STARTED)
		{
			mk_uwb::measure();
		}
		else{
			PX4_ERR("NO RANGING STARTED");
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


