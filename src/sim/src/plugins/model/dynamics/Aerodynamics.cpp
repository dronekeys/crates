#include "Aerodynamics.h"

using namespace gazebo;

// Basic constants 
#define METERS_TO_FEET      3.2808399
#define FEET_TO_METERS      0.3048000
#define DEGREES_TO_RADIANS  0.01745329258399
#define RADIANS_TO_DEGREES  57.2957795000000

// All sensors must be configured using the current model information and the SDF
bool Aerodynamics::Configure(physics::LinkPtr link, sdf::ElementPtr root)
{
    // Save the link pointer
    linkPtr = link;

    // Get drag parameters
    root->GetElement("shear")->GetElement("ma")->GetValue()->Get(_ma);
    root->GetElement("shear")->GetElement("z0")->GetValue()->Get(_z0);

    // Get drag parameters
    root->GetElement("drag")->GetElement("kuv")->GetValue()->Get(_kuv);
    root->GetElement("drag")->GetElement("kw")->GetValue()->Get(_kw);

    // Setup the noise distribution
    nTurbulence = NoiseFactory::Create(root->GetElement("errors")->GetElement("turbulence"));

    // Initialise a node pointer
    nodePtr = transport::NodePtr(new transport::Node());
    nodePtr->Init(linkPtr->GetModel()->GetWorld()->GetName());

    // Subscribe to messages about wind conditions
    subPtr = nodePtr->Subscribe("~/wind", &Aerodynamics::ReceiveWind, this);

    // Reset
    Reset();

    // Success
    return true;
}

// All sensors must be resettable
void Aerodynamics::Reset()
{
    ready = false;
}

// Get the current altitude
void Aerodynamics::Update(double dt)
{
    // Extract the altitude and orientation from the state
    double altitude = linkPtr->GetWorldPose().pos.z;

    // Calculate wind shear (in the navigation frame)
    math::Vector3 wind(0.0, 0.0, 0.0);
    if (ready && altitude > _ma)
    {
        // Calculate wind shear
        double k = global.speed() * log(METERS_TO_FEET*altitude/_z0) / log(20.0/_z0);
        wind.x  = -k * cos(DEGREES_TO_RADIANS * global.direction());
        wind.y  = -k * sin(DEGREES_TO_RADIANS * global.direction());
        wind.z  = 0.0;

        // Parameterise the turbulence model
        nTurbulence->Configure(DRYDEN_PARS_WNDSPEED, global.speed());
        nTurbulence->Configure(DRYDEN_PARS_AIRSPEED, linkPtr->GetWorldLinearVel().GetLength());
        nTurbulence->Configure(DRYDEN_PARS_ALTITUDE, altitude);

        // Add wind turbulence
        wind += (math::Vector3) nTurbulence->DrawVector(dt);
    }

    // Body-frame airspeed
    math::Vector3 airspeed = linkPtr->GetRelativeLinearVel() 
                           - linkPtr->GetWorldPose().rot.RotateVector(wind);

    // Rotate to bodyframe,a nd 
    math::Vector3 force(
        _kuv * linkPtr->GetInertial()->GetMass() * airspeed.x,
        _kuv * linkPtr->GetInertial()->GetMass() * airspeed.y,
         _kw * linkPtr->GetInertial()->GetMass() * airspeed.z
    );

    // Apply force and torque
    linkPtr->AddRelativeForce(force);
    linkPtr->AddRelativeTorque(linkPtr->GetInertial()->GetCoG().Cross(force));
}

// Periodically the simulation produces a wind message
void Aerodynamics::ReceiveWind(WindPtr& msg)
{
    // Save the wind data
    global = *msg;

    // We now have wind data
    ready = true;
}