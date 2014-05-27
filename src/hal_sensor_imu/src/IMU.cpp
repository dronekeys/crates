// Library headers
#include <hal/sensor/IMU.h>

#define DEFAULT_SAMP_RATE 50
#define DEFAULT_SEND_RATE  1

using namespace hal::sensor;

IMU::IMU() : hal::HAL()
{
	// Do nothign
}        

// Called when HAL loads
void IMU::OnInit()
{
    // Advertise this message on the ROS backbone (note the use of template here to fix GCC error)
    publisher = GetRosNodePtr().template advertise<hal_sensor_imu::Data>("sensor/imu/Data", DEFAULT_QUEUE_LENGTH);

    // Advertice the ability to configure the sensor rate
    service = GetRosNodePtr().advertiseService("sensor/imu/Configure", &IMU::Configure, this);

    // Create a timer to broadcast the data
    timerSamp = GetRosNodePtr().createTimer(
        ros::Duration(1.0/DEFAULT_SAMP_RATE),       // Callback rate
        &IMU::Sample,                               // Callback
        this,                                       // Callee
        false,                                      // Oneshot
        true                                        // Autostart
    );

    // Create a timer to broadcast the data
    timerSend = GetRosNodePtr().createTimer(
        ros::Duration(1.0/DEFAULT_SEND_RATE),       // Callback rate
        &IMU::Broadcast,                            // Callback
        this,                                       // Callee
        false,                                      // Oneshot
        true                                        // Autostart
    );
}

bool IMU::Configure(hal_sensor_imu::Configure::Request &req, hal_sensor_imu::Configure::Response &res)
{
    timerSamp.stop();
    if (req.samprate > 0)
    {
        timerSamp.setPeriod(ros::Duration(1.0/req.samprate));
        timerSamp.start();
    }
    timerSend.stop();
    if (req.sendrate > 0)
    {
        timerSend.setPeriod(ros::Duration(1.0/req.sendrate));
        timerSend.start();
    }
    res.success = true;
    res.status  = "Sensor update rate changed";
    return true;
}

void IMU::Broadcast(const ros::TimerEvent& event)
{                  
    publisher.publish(message);
}

void IMU::Sample(const ros::TimerEvent& event)
{                  
    GetMeasurement(message);
}