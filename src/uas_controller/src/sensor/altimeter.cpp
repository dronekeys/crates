/* 

  The barometric altimeter is modelled using information from:

    http://www.hills-database.co.uk/altim.html

  Barometric pressure sensors operate on the following principle:

    change in height = (RT/gM).ln(s/p)

  Where

    s = reference height            [m]
    p = height                      [m]
    R = gas constant                [] 
    T = measured air temperature    [Kelvin]
    g = acceleration due to gravity []
    M = molar mass of air           []
  
*/


// Required to bind to non-static methods
#include <boost/bind.hpp>

// Required simulation API  
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/gazebo.hh>

namespace uas_controller
{
  class Altimeter : public gazebo::SensorPlugin
  {
    public: CameraPlugin();

    /// \brief Destructor
    public: virtual ~CameraPlugin();

    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    public: virtual void OnNewFrame(const unsigned char *_image,
                              unsigned int _width, unsigned int _height,
                              unsigned int _depth, const std::string &_format);

    protected: unsigned int width, height, depth;
    protected: std::string format;

    protected: sensors::CameraSensorPtr parentSensor;
    protected: rendering::CameraPtr camera;

    private: event::ConnectionPtr newFrameConnection;
  };
}
#endif




// Required to bind to non-static methods
#include <boost/bind.hpp>

// Gazebo API
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>

namespace uas_controller
{
  class Altimeter : public gazebo::ModelPlugin
  {

  private:

    // Pointer to the current model
    gazebo::physics::ModelPtr         modelPtr;

  public: 

    // On initial plugin load
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) 
    {
      // Save pointer to the model
      this->modelPtr = _model;
    }

    // When an update is called 
    void Update(ConstWorldStatisticsPtr &_msg)
    {
     


    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Altimeter)
}