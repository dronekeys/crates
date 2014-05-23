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

    // Pointer to the rigid body
    physics::LinkPtr              linkPtr;

    // Requirements for listening for Gazbeo messages
    event::ConnectionPtr          conPtr;
    transport::NodePtr            nodePtr;
    transport::SubscriberPtr      subPtr;

    // When global wind has been received
    bool                          ready;

    // Current wind cpnditions
    msgs::Wind                    global;

    // Turbulence noise
    Noise                         *nTurbulence;

    // Drag parameters
    double                        _kuv, _kw, _ma, _z0;

    // Periodically the simulation produces a wind message
    void ReceiveWind(WindPtr& msg);

  public:

    // All sensors must be configured using the current model information and the SDF
    bool Configure(physics::LinkPtr link, sdf::ElementPtr root);

    // All sensors must be resettable
    void Reset();

    // Get the current altitude
    void Update(double dt);

  };

}

#endif