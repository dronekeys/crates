#include "Energy.h"

using namespace gazebo;

// All sensors must be configured using the current model information and the SDF
bool Energy::Configure(sdf::ElementPtr root)
{
  root->GetElement("remaining")->GetValue()->Get(energyRemaining);
  root->GetElement("consumption")->GetElement("base")->GetValue()->Get(energyConsumptionBase);
  root->GetElement("consumption")->GetElement("flight")->GetValue()->Get(energyRateFactor);
  root->GetElement("limits")->GetElement("warn")->GetValue()->Get(energyKill);
  root->GetElement("limits")->GetElement("land")->GetValue()->Get(energyLand);
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
void Energy::Update(physics::LinkPtr linkPtr, double dt)
{
  // Store current consumption rate
  consumption = energyConsumptionBase 
              + energyRateFactor * linkPtr->GetWorldLinearAccel().z;

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