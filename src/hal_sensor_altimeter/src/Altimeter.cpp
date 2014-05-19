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

    // Advertice the ability to configure the sensor rate
    service = GetRosNodePtr()->advertiseService("sensor/altimeter/Configure", &Altimeter::Configure, this);

    // Create a timer to broadcast the data
    timer = GetRosNodePtr()->createTimer(
        ros::Duration(1.0/DEFAULT_SENSOR_RATE),     // Callback rate
        &Altimeter::Broadcast,                         // Callback
        this,                                       // Callee
        false,                                      // Oneshot
        true                                        // Autostart
    );
}

bool Altimeter::Configure(hal_sensor_altimeter::Configure::Request &req, hal_sensor_altimeter::Configure::Response &res)
{
    timer.stop();
    if (req.rate > 0)
    {
        timer.setPeriod(ros::Duration(1.0/req.rate));
        timer.start();
        return true;
    }
    res.success = false;
    res.status  = "Could not change sensor rate, because of invalid rate request value";
    return false;
}

void Altimeter::Broadcast(const ros::TimerEvent& event)
{   
    // Obtain the measure               
    GetMeasurement(message);
    
    // Publish the measurement
    publisher.publish(message);
}