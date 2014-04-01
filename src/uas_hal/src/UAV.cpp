#include <uas_hal/UAV.h>

#define CONTROL_UPDATE_RATE 50.0

using namespace uas_hal;

UAV::UAV(const char *name, ros::NodeHandle& node) :

    // Time = 0
    tick(0.0),

    // Initialise the peripherals
    Peripheral<MsgAltitude>(node,"Altitude"),
    Peripheral<MsgInertial>(node,"Inertial"),
    Peripheral<MsgMagnetic>(node,"Magnetic"),
    Peripheral<MsgPosition>(node,"Position"),
    Peripheral<MsgAttitude>(node,"Attitude")

    // Initialise the controllers

{
    // Maske sure that ROS actually started, or there will be some issues...
    if (!ros::isInitialized())
        ROS_FATAL("A ROS node for Gazebo has not been initialized");

    // Advertise this message
    pubI = node.advertise<MsgInformation>("Information",1);
    pubC = node.advertise<MsgControl>("Control",1);
    pubS = node.advertise<MsgState>("State",1);

    // Immediately start control loop
    timU = node.createTimer(ros::Duration(1.0/CONTROL_UPDATE_RATE), &UAV::Update, this);
}

// TIMER CALLBACKS ////////////////////////////////////////////////////////////

void UAV::BroadcastInformation(const ros::TimerEvent& event)
{
    pubI.publish(msgI);
}

void UAV::BroadcastControl(const ros::TimerEvent& event)
{
    pubC.publish(msgC);
}

void UAV::BroadcastState(const ros::TimerEvent& event)
{
    pubS.publish(msgS);
}

// Propagate the system forward in time
void UAV::Update(const ros::TimerEvent& event)
{
    // Get the current UAV state
    msgS = nav.GetState();

    // Determine the control to apply, given the current state and time tick
    //msgI = Controller::Update(msgS, event.current_real.toSec() - tick);

    // Forward the control to the FCS or Simulation
    AcceptControl(msgC);

    // Save the current time tick
    tick = event.current_real.toSec();
}

// PERIPHERAL CALLBACKS ///////////////////////////////////////////////////////

// Receive altitude measurement immediately
void UAV::Receive(const MsgAltitude &msg)
{
    nav.Measurement(msg);
}

// Receive inertial measurement immediately
void UAV::Receive(const MsgInertial &msg)
{
    nav.Measurement(msg);
}

// Receive magnetic measurement immediately
void UAV::Receive(const MsgMagnetic &msg)
{
    nav.Measurement(msg);
}

// Receive position measurement immediately
void UAV::Receive(const MsgPosition &msg)
{
    nav.Measurement(msg);
}

// Receive position measurement immediately
void UAV::Receive(const MsgAttitude &msg)
{
    nav.Measurement(msg);
}

// CONFIGURE MESSAGING RATES //////////////////////////////////////////////////////

void UAV::Reset(const double &rateI, const double &rateC, const double &rateS)
{
    timI = node.createTimer(ros::Duration(1.0/rateI), &UAV::BroadcastInformation, this);
    timC = node.createTimer(ros::Duration(1.0/rateC), &UAV::BroadcastControl, this);
    timS = node.createTimer(ros::Duration(1.0/rateS), &UAV::BroadcastState, this);
}