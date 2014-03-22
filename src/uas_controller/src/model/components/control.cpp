/* This gazebo model plugin implements a second order model for quadrotor dynamics */ 
#include "control.h"

using namespace uas_controller;

// Default constructor
Control::Control() :
		srs(-2291.83118052329), srl(-0.9), sru(0.9), 
    sps(-2291.83118052329), spl(-0.9), spu(0.9), 
		sys(-460.597254433196), syl(-4.5), syu(4.5), 
    sts( 4097.0), stl(0.0), stu(1.0),
		svs( 1.0), svl(9.0), svu(12.0)    {}

// Default constructor takes configuration + pointer to link
void Control::Configure(sdf::ElementPtr root)
{
 	/***********************************

      <control>
        <pitch>
           <scale>-2291.83118052329</scale>
           <min>-0.9</min>
           <max>-0.9</max>
        </pitch>
        <roll>-2291.83118052329</roll>
        <yaw>-460.597254433196</yaw>
        <throttle>4097.0</throttle>
        <voltage>1.0</voltage>
      </control>                         

    ***********************************/

  // CONTROL_LIMITS = [-0.9,0.9; -0.9,0.9; 0,1; -4.5,4.5; 9,12]; %limits of the control inputs
  // SI_2_UAVCTRL = [-1/degsToRads(0.025);-1/degsToRads(0.025);4097;-1/degsToRads(254.760/2047);1]; % conversion factors
  // BATTERY_RANGE = [9,12]; % range of valid battery values volts
  
  // Direct parameters
  srs = GetSDFDouble(root,"control.roll.scale",srs);
  srl = GetSDFDouble(root,"control.roll.min",srl);
  sru = GetSDFDouble(root,"control.roll.max",sru);
  sps = GetSDFDouble(root,"control.pitch.scale",sps);
  spl = GetSDFDouble(root,"control.pitch.min",spl);
  spu = GetSDFDouble(root,"control.pitch.max",spu);
  sys = GetSDFDouble(root,"control.yaw.scale",sys);
  syl = GetSDFDouble(root,"control.yaw.min",syl);
  syu = GetSDFDouble(root,"control.yaw.max",syu);
  sts = GetSDFDouble(root,"control.throttle.scale",sts);
  stl = GetSDFDouble(root,"control.throttle.min",stl);
  stu = GetSDFDouble(root,"control.throttle.max",stu);
  svs = GetSDFDouble(root,"control.voltage.scale",svs);
  svl = GetSDFDouble(root,"control.voltage.min",svl);
  svu = GetSDFDouble(root,"control.voltage.max",svu);
}

double Control::GetScaledRoll()
{
  return  srs * limit(roll,srl,sru);
}

double Control::GetScaledPitch()
{
	return  sps * limit(pitch,spl,spu);
}

double Control::GetScaledYaw()
{
	return  sys * limit(yaw,syl,syu);
}

double Control::GetScaledThrottle()
{
  return  sts * limit(throttle,stl,stu);
}

double Control::GetScaledVoltage()
{
	return  svs * limit(voltage,svl,svu);
}
