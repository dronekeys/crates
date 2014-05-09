// Library headers
#include <hal/sensor/GNSS.h>

#define DEFAULT_SENSOR_RATE   1.0

using namespace hal::sensor;

GNSS::GNSS() : hal::HAL()
{
	// Do nothign
}        

// Called when HAL loads
void GNSS::OnInit()
{
    // Advertise this message on the ROS backbone (note the use of template here to fix GCC error)
    publisher = GetRosNodePtr()->template advertise<hal_sensor_gnss::Data>("sensor/gnss/Data", DEFAULT_QUEUE_LENGTH);

    // Create a timer to broadcast the data
    timer = GetRosNodePtr()->createTimer(
        ros::Duration(1.0/DEFAULT_SENSOR_RATE),     // Callback rate
        &GNSS::Broadcast,                         // Callback
        this,                                       // Callee
        false,                                      // Oneshot
        true                                        // Autostart
    );
}

void GNSS::Broadcast(const ros::TimerEvent& event)
{                  
    publisher.publish(message);
}