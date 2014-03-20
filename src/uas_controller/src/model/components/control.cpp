/* This gazebo model plugin implements a second order model for quadrotor dynamics */ 
#include "control.h"

using namespace uas_controller;

// Default constructor
Control::Control() 
	: sp(-2291.83118052329), 
		sr(-2291.83118052329), 
		sy(-460.597254433196),
    st( 4097.0),
		sv(1.0) {}

// Default constructor takes configuration + pointer to link
void Control::Configure(sdf::ElementPtr root)
{
 	/***********************************

      <control>
        <pitch>-2291.83118052329</pitch>
        <roll>-2291.83118052329</roll>
        <yaw>-460.597254433196</yaw>
        <throttle>4097.0</throttle>
        <voltage>1.0</voltage>
      </control>                         

    ***********************************/

  // Direct parameters
  sp = GetSDFDouble(root,"control.pitch",sp);
  sr = GetSDFDouble(root,"control.roll",sr);
  sy = GetSDFDouble(root,"control.yaw",sy);
  st = GetSDFDouble(root,"control.throttle",st);
  sv = GetSDFDouble(root,"control.voltage",sv);
}

// Returns control scaled to RC values
double Control::GetScaledPitch()
{
	return  sp * pitch;
}

double Control::GetScaledRoll()
{
	return  sr * roll;
}

double Control::GetScaledYaw()
{
	return  sy * yaw;
}

double Control::GetScaledThrottle()
{
  return  st * throttle;
}

double Control::GetScaledVoltage()
{
	return  sv * voltage;
}
