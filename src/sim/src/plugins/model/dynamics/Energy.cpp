#include "Energy.h"

using namespace gazebo;

// All sensors must be configured using the current model information and the SDF
bool Energy::Configure(physics::LinkPtr link, sdf::ElementPtr root)
{
  // Save the link pointer
  linkPtr = link;

  // Configuration
  root->GetElement("remaining")->GetValue()->Get(energyRemaining);
  root->GetElement("consumption")->GetElement("base")->GetValue()->Get(energyConsumptionBase);
  root->GetElement("consumption")->GetElement("flight")->GetValue()->Get(energyRateFactor);
  root->GetElement("limits")->GetElement("warn")->GetValue()->Get(energyWarn);
  root->GetElement("limits")->GetElement("land")->GetValue()->Get(energyLand);

  // Setup the noise distribution
  nConsumption = NoiseFactory::Create(root->GetElement("errors")->GetElement("consumption"));

  // Success!
  return true;
}

// All sensors must be resettable
void Energy::Reset()
{
  // Current consumption rate
  consumption = 0;

  // How much energy remaining
  remaining = energyRemaining;
}

// Get the current altitude
void Energy::Update(double dt)
{
  // Store current consumption rate
  consumption = energyConsumptionBase 
              + energyRateFactor * linkPtr->GetWorldLinearAccel().z
              + (double) nConsumption->DrawScalar(dt);

  // Calculate th
  remaining -= dt * consumption;
}

// Get the remaining energy (mAh)
double Energy::GetRemaining()
{
  return remaining;
}

// Set the remaining energy (mAh)
void Energy::SetRemaining(double val)
{
  remaining = val;
}