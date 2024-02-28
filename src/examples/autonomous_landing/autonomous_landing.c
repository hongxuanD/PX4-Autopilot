#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_uwb.h> // uORB for sensor_uwb

__EXPORT int autonomous_landing_main(int argc, char *argv[]); // Export main for starting in another thread

int autonomous_landing_main(int argc, char *argv[])
{
        PX4_INFO("Hello autonomous landing");
            // Create structure to store data in
        struct sensor_uwb_s sensor_uwb;

        // Clear the structure by filling it with 0s in memory
        memset(&sensor_uwb, 0, sizeof(sensor_uwb));

        // Create a uORB topic advertisement
        orb_advert_t sensor_uwb_pub = orb_advertise(ORB_ID(sensor_uwb), &sensor_uwb);

        if (argc>1){
            if (strcmp(argv[1], "Start") == 0)
            {
                sensor_uwb.uwb_anchor_cmd = 0;
            }
            else if(strcmp(argv[1], "StartRanging") == 0)
            {
                sensor_uwb.uwb_anchor_cmd = 1;
            }
            else if(strcmp(argv[1], "StopRanging") == 0)
            {
                sensor_uwb.uwb_anchor_cmd = 2;
            }
            else if(strcmp(argv[1], "SetConfig") == 0)
            {
                sensor_uwb.uwb_anchor_cmd = 3;
                sensor_uwb.group_id = atoi(argv[2]);
                sensor_uwb.config_id = atoi(argv[3]); 
                sensor_uwb.config_value = atoi(argv[4]);
            };

        }
        else{
            sensor_uwb.uwb_anchor_cmd = 0;
        }
        orb_publish(ORB_ID(sensor_uwb), sensor_uwb_pub, &sensor_uwb);

        PX4_INFO("UWB publisher exit");
        return 0; // return of main function

}
