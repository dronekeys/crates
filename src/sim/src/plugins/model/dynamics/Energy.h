#ifndef SIM_DYNAMICS_ENERGY_H
#define SIM_DYNAMICS_ENERGY_H

// Basic sensor functionality
#include "Dynamics.h"

namespace gazebo
{
  class Energy : public Dynamics
  {

  private:
  
      // Pointer to the rigid body
    physics::LinkPtr linkPtr;

    // Parameters fro the energy model
    double energyRemaining;
    double energyConsumptionBase;
    double energyRateFactor;

    // Used online to store tallys
    double consumption, remaining;

  public:

    // All sensors must be configured using the current model information and the SDF
    bool Configure(physics::LinkPtr linkPtr, sdf::ElementPtr root);

    // All sensors must be resettable
    void Reset();

    // Get the current altitude
    void Update(double dt);

    // CUSTOM FUNCTIONALITY //////////////////////////////////////////////////////////

    // Get the remaining energy (mAh)
    double GetRemaining();

    // Set the remaining energy (mAh)
    void SetRemaining(double val); 
  };

}

#endif