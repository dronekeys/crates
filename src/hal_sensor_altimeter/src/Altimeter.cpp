// Library headers
#include <hal/sensor/Altimeter.h>

#define DEFAULT_SENSOR_RATE   1.0

using namespace hal::sensor;

Altimeter::Altimeter() : hal::HAL()
{
	// Do nothign
}        

// Called when HAL loads
void Altimeter::OnInit()
{
    // Advertise this message on the ROS backbone (note the use of template here to fix GCC error)
    publisher = GetRosNodePtr()->template advertise<hal_sensor_altimeter::Data>("sensor/altimeter/Data", DEFAULT_QUEUE_LENGTH);

    // Create a timer to broadcast the data
    timer = GetRosNodePtr()->createTimer(
        ros::Duration(1.0/DEFAULT_SENSOR_RATE),     // Callback rate
        &Altimeter::Broadcast,                         // Callback
        this,                                       // Callee
        false,                                      // Oneshot
        true                                        // Autostart
    );
}

void Altimeter::Broadcast(const ros::TimerEvent& event)
{                  
    publisher.publish(message);
}