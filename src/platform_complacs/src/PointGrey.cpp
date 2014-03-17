// ROS libraries
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// Project libraries
#include <uas_hal/peripheral/Camera.h>

namespace platform_complacs
{
    // Class derives from Nodelet and all HALs it will provide to ROS
    class PointGrey : public uas_hal::Camera, public nodelet::Nodelet
    {

    private:


    public:

        // Constructor
        PointGrey() : uas_hal::Camera() {}

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
    PLUGINLIB_EXPORT_CLASS(platform_complacs::PointGrey, nodelet::Nodelet);

}