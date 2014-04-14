#include <hal/Sensor.h>

using namespace hal::sensor;

template <class DataMsgClass>
Sensor<DataMsgClass>::Sensor(const char *name) : hal::HAL(name)
{
    // Advertise this message on the ROS backbone
    publisher = rosNode.advertise<DataMsgClass>(name, DEFAULT_QUEUE_LENGTH);

    // Advertice the ability to configure the sensor rate
    service = rosNode.advertiseService(name, &Sensor<DataMsgClass>::SetRate, this);

    // Create a timer to broadcast the data
    timer = rosNode.createTimer(
        ros::Duration(1.0/DEFAULT_SENSOR_RATE),     // Callback rate
        &Sensor::Broadcast,                         // Callback
        this,                                       // Callee
        false,                                      // Oneshot
        true                                        // Autostart
    );
}

template <class DataMsgClass>
bool Sensor<DataMsgClass>::SetRate(hal::SetRate::Request &req, hal::SetRate::Response &res)
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

template <class DataMsgClass>
void Sensor<DataMsgClass>::Broadcast(const ros::TimerEvent& event)
{                  
    publisher.publish(message);
}

