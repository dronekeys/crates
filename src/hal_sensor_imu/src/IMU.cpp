// Library headers
#include <hal/sensor/IMU.h>

#define DEFAULT_SENSOR_RATE   1.0

using namespace hal::sensor;

IMU::IMU() : hal::HAL()
{
	// Do nothign
}        

// Called when HAL loads
void IMU::OnInit()
{
    // Advertise this message on the ROS backbone (note the use of template here to fix GCC error)
    publisher = GetRosNodePtr()->template advertise<hal_sensor_imu::Data>("sensor/imu/Data", DEFAULT_QUEUE_LENGTH);

    // Advertice the ability to configure the sensor rate
    service = GetRosNodePtr()->advertiseService("sensor/imu/SetRate", &IMU::SetRate, this);

    // Create a timer to broadcast the data
    timer = GetRosNodePtr()->createTimer(
        ros::Duration(1.0/DEFAULT_SENSOR_RATE),     // Callback rate
        &IMU::Broadcast,                         // Callback
        this,                                       // Callee
        false,                                      // Oneshot
        true                                        // Autostart
    );
}

bool IMU::SetRate(hal_sensor_imu::SetRate::Request &req, hal_sensor_imu::SetRate::Response &res)
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

void IMU::Broadcast(const ros::TimerEvent& event)
{                  
    publisher.publish(message);
}