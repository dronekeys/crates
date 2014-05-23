// Library headers
#include <hal/sensor/Orientation.h>

#define DEFAULT_SAMP_RATE 50
#define DEFAULT_SEND_RATE  1

using namespace hal::sensor;

Orientation::Orientation() : hal::HAL()
{
	// Do nothign
}        

// Called when HAL loads
void Orientation::OnInit()
{
    // Advertise this message on the ROS backbone (note the use of template here to fix GCC error)
    publisher = GetRosNodePtr()->template advertise<hal_sensor_orientation::Data>("sensor/orientation/Data", DEFAULT_QUEUE_LENGTH);

    // Advertice the ability to configure the sensor rate
    service = GetRosNodePtr()->advertiseService("sensor/orientation/Configure", &Orientation::Configure, this);

    // Create a timer to broadcast the data
    timerSamp = GetRosNodePtr()->createTimer(
        ros::Duration(1.0/DEFAULT_SAMP_RATE),       // Callback rate
        &Orientation::Sample,                       // Callback
        this,                                       // Callee
        false,                                      // Oneshot
        true                                        // Autostart
    );

    // Create a timer to broadcast the data
    timerSend = GetRosNodePtr()->createTimer(
        ros::Duration(1.0/DEFAULT_SEND_RATE),       // Callback rate
        &Orientation::Broadcast,                    // Callback
        this,                                       // Callee
        false,                                      // Oneshot
        true                                        // Autostart
    );
}

bool Orientation::Configure(hal_sensor_orientation::Configure::Request &req, hal_sensor_orientation::Configure::Response &res)
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

void Orientation::Broadcast(const ros::TimerEvent& event)
{                  
    publisher.publish(message);
}

void Orientation::Sample(const ros::TimerEvent& event)
{                  
    GetMeasurement(message);
}