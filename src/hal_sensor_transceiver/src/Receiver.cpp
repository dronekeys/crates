#include <hal_sensor_transceiver/Receiver.h>

#define DEFAULT_SAMP_RATE  1
#define DEFAULT_SEND_RATE  1

using namespace hal::sensor;

Receiver::Receiver() : hal::HAL()
{
	//DO NOTHING
    ROS_INFO("RECEIVER INIT!!");
}

void Receiver::OnInit()
{
    ROS_INFO("RECEIVER INIT!!");
	//DO NOTHING
	//// Advertise this message on the ROS backbone (note the use of template here to fix GCC error)
    publisher = GetRosNodePtr().template advertise<hal_sensor_transceiver::Data>("sensor/receiver/Data", DEFAULT_QUEUE_LENGTH);

    // Advertice the ability to configure the sensor rate
    service = GetRosNodePtr().advertiseService("sensor/receiver/Configure", &Receiver::Configure, this);

    // Create a timer to broadcast the data
    timerSamp = GetRosNodePtr().createTimer(
        ros::Duration(1.0/DEFAULT_SAMP_RATE),       // Callback rate
        &Receiver::Sample,                               // Callback
        this,                                       // Callee
        false,                                      // Oneshot
        true                                        // Autostart
    );

    // Create a timer to broadcast the data
    timerSend = GetRosNodePtr().createTimer(
        ros::Duration(1.0/DEFAULT_SEND_RATE),       // Callback rate
        &Receiver::Broadcast,                            // Callback
        this,                                       // Callee
        false,                                      // Oneshot
        true                                        // Autostart
    );
}

bool Receiver::Configure(hal_sensor_transceiver::Configure::Request &req, hal_sensor_transceiver::Configure::Response &res)
{
    ROS_INFO("RECEIVER CONFIGURE!!");
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

void Receiver::Broadcast(const ros::TimerEvent& event)
{
	publisher.publish(message);
}

void Receiver::Sample(const ros::TimerEvent& event)
{
	GetMeasurement(message);
}