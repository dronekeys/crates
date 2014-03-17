// ROS libraries
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// Project libraries
#include <uas_hal/peripheral/Radio.h>

namespace platform_complacs
{
    // Class derives from Nodelet and all HALs it will provide to ROS
    class WiFi : public uas_hal::Radio, public nodelet::Nodelet
    {

    private:


    public:

        // Constructor
        WiFi() : uas_hal::Radio() {}

        // Called when nodelet is initialised
        void onInit()
        {
            //////////////////////////////////////////
            // Perform HAL binds to expose ROS node //
            //////////////////////////////////////////

            // The moment the nodelet is initialised, set up the HAL
            ROS_INFO("Binding HAL");
            this->bind(this->getPrivateNodeHandle());

            /////////////////////////////////////////
            // Get parameters from the launch file //
            /////////////////////////////////////////

            /*
            ROS_INFO("Grabbing serial parameters from launch file");
            if (!this->getPrivateNodeHandle().getParam("serial_port",port)) 
                port = "/dev/ttyUSB0";
            if (!this->getPrivateNodeHandle().getParam("serial_baud",baud))
                baud = 57600;
            */

        }
    };

    // Required in order to be used as a nodelet
    PLUGINLIB_EXPORT_CLASS(platform_complacs::WiFi, nodelet::Nodelet);

}