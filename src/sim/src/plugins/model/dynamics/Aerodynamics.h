#ifndef SIM_DYNAMICS_AERODYNAMICS_H
#define SIM_DYNAMICS_AERODYNAMICS_H

// Basic dynamics functionality
#include "Dynamics.h"

// Wind messages
#include "wind.pb.h"

namespace gazebo
{
  // Convenience declaraion
  typedef const boost::shared_ptr<const msgs::Wind> WindPtr;

  // A class for modelling aerodynamics
  class Aerodynamics : public Dynamics
  {
  private:

    // Requirements for listening for Gazbeo messages
    event::ConnectionPtr          conPtr;
    transport::NodePtr            nodePtr;
    transport::SubscriberPtr      subPtr;

    // Current wind cpnditions
    msgs::Wind                    global;

    // Internal parameters
    math::Vector3                 wind;

    // Current time
    double _kuv, _kw;

    // Periodically the simulation produces a wind message
    void ReceiveWind(WindPtr& msg);

  public:

    // All sensors must be configured using the current model information and the SDF
    bool Configure(sdf::ElementPtr root);

    // All sensors must be resettable
    void Reset();

    // Get the current altitude
    void Update(physics::LinkPtr linkPtr, double dt);

  };

}

#endif