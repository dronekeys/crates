// Library headers
#include <hal/sensor/Compass.h>

#define DEFAULT_SENSOR_RATE   1.0

using namespace hal::sensor;

Compass::Compass() : hal::HAL()
{
	// Do nothign
}        

// Called when HAL loads
void Compass::OnInit()
{
    // Advertise this message on the ROS backbone (note the use of template here to fix GCC error)
    publisher = GetRosNodePtr()->template advertise<hal_sensor_compass::Data>("sensor/compass/Data", DEFAULT_QUEUE_LENGTH);

    // Advertice the ability to configure the sensor rate
    service = GetRosNodePtr()->advertiseService("sensor/compass/Configure", &Compass::Configure, this);

    // Create a timer to broadcast the data
    timer = GetRosNodePtr()->createTimer(
        ros::Duration(1.0/DEFAULT_SENSOR_RATE),     // Callback rate
        &Compass::Broadcast,                         // Callback
        this,                                       // Callee
        false,                                      // Oneshot
        true                                        // Autostart
    );
}

bool GNSS::Configure(hal_sensor_compass::Configure::Request &req, hal_sensor_compass::Configure::Response &res)
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

void Compass::Broadcast(const ros::TimerEvent& event)
{                  
    // Obtain the measure               
    GetMeasurement(message);

    // Publish the data    
    publisher.publish(message);
}