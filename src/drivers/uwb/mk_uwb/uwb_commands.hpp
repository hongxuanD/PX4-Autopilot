#pragma once
#define MK_UWB_MAX_PAYLOAD 256


enum UWB_CMD_TYPE {
	CMD_INIT_UWBS = 0x01,
	CMD_GET_PLATFORM_INFO=0x02,
	CMD_START_UWB_RANGING=0x03,
	CMD_STOP_UWB_RANGING=0x04,
	CMD_SET_CONFIG=0x05,
	CMD_GET_CONFIGS=0x06,
	CMD_START_DISCOVERY=0x07,
	CMD_STOP_DISCOVERY=0x08,
};

enum UWB_NTF_TYPE {
	NTF_UWB_RANGING=0xA0,
	NTF_UWB_CHANGE_STATE=0xA1,
	NTF_SYSTEM_CHANGE_STATE=0xA2,
	NTF_DISCOVERED_DEVICE=0xA3,
};


enum CONFIG_GROUP_ID{
	GENERAL=0x00,
	iOS=0x01,
	Android=0x02,
	Device_SR150=0x03,
};

enum CONFIG_ID{
	// General
	ANTENNA_CONFIG=0x02,

};

enum SENSOR_STATE{
	//Initialize anchor
	UWB_NOT_INITIALIZED=0,
	UWBS_READY=1,
	//Configure UWBS
	SET_CONFIG=20,
	GET_CONFIG=21,
	SCANNER_CONFIG=22,
	UUID_CONFIG=23,
	//Discover bases
	START_DISCOVERY=30,
	DISCOVERING=31,
	DEVICE_DISCOVERED=32,
	STOP_DISCOVERY=33,
	//ranging
	START_RANGING=50,//Same as UWB_RANGING_STOPPED
	STOP_RANGING=51,
	RECEIVE_RANGING_DATA=53,
	UWB_RANGING_STOPPED=50,//Same as START_RANGING
	RANGING_FAILED=52,

};

enum PROTOCOL_CMD_OPTION{
	CMD_INIT_GET_PLATFORM_INFO=0,
	CMD_START_RANGING=1,
	CMD_STOP_RANGING=2,
	CMD_SET_CONFIGS=3,
	CMD_TEST_RANGING=4,
};

// Store contents of rx'd frame
struct {
	int Type{0xFF};   // Message type
	int Length{0xFF};    // Message length
	uint8_t Value[MK_UWB_MAX_PAYLOAD] ;
} rx_field;

struct{
	int Group_ID{0xFF};
	int Config_ID{0xFF};
	int Config_Value{0xFF};

	unsigned char device1_uuid[16];
	unsigned char device2_uuid[16];

} config_field;

struct {
	uint16_t MAC[2];					// MAC address of UWB device
	uint8_t status;					// Status of Measurement
	uint16_t distance{0}; 				// Distance in cm
	uint8_t nLos; 					// line of sight y/n
	int16_t aoa_azimuth;				// AOA of incoming msg for Azimuth antenna pairing
	int16_t aoa_elevation;				// AOA of incoming msg for Altitude antenna pairing
	int16_t aoa_dest_azimuth;			// AOA destination Azimuth
	int16_t aoa_dest_elevation; 			// AOA destination elevation
	uint8_t aoa_azimuth_FOM;			// AOA Azimuth FOM
	uint8_t aoa_elevation_FOM;			// AOA Elevation FOM
	uint8_t aoa_dest_azimuth_FOM;			// AOA Azimuth FOM
	uint8_t aoa_dest_elevation_FOM;			// AOA Elevation FOM
} distance_result;




