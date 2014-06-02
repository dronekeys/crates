#include <hal_quadrotor/Navigation.h>

using namespace hal::quadrotor;

Navigation::Navigation() : ready(false),
  wgs84_ecef(
    GeographicLib::Constants::WGS84_a(), 
    GeographicLib::Constants::WGS84_f()
  )
{
  // Do nothing
}


void Navigation::Reset()
{
  // Do nothing
}

void Navigation::GetState(hal_quadrotor::State &msg)
{
  msg   = state;
  msg.t = ros::Time::now().toSec();
}

void Navigation::SetState(const hal_quadrotor::State &msg)
{
  state   = msg;
  state.t = ros::Time::now().toSec();  
}

void Navigation::Process(const hal_sensor_altimeter::Data &msg)
{
  state.z = msg.z;
  state.w = msg.w;
}

void Navigation::Process(const hal_sensor_compass::Data &msg)
{
  // Ignore
}

void Navigation::Process(const hal_sensor_imu::Data &msg)
{
  // Ignore
}

void Navigation::Process(const hal_sensor_gnss::Data &msg)
{
  // Used to discard height
  double height;
  
  // If the LTP origin was set
  if (ready)
  {
    // 
    //ROS_INFO("Current GPS position: %f %f %f", msg.latitude, msg.longitude, msg.altitude);

    // Recover LTP coordinates from the WGS84 message
    wgs84_enu.Forward(
      msg.latitude, 
      msg.longitude, 
      msg.altitude,
      state.x,
      state.y,
      height
    );

    // 
    //ROS_INFO("Current LTP position: %f %f %f", state.x, state.y, height);

  }

  // n-frame velocity estimate
  state.x = msg.vel_ew;
  state.y = msg.vel_ns;

}

void Navigation::Process(const hal_sensor_orientation::Data &msg)
{
  state.roll  = msg.roll;
  state.pitch = msg.pitch;
  state.yaw   = msg.yaw;
  state.p     = msg.p;
  state.q     = msg.q;
  state.r     = msg.r;
}

void Navigation::SetOrigin(double latitude, double longitude, double altitude)
{
  // Convert from WGS84 to LTP coordinates
  wgs84_enu = GeographicLib::LocalCartesian(
    latitude, 
    longitude, 
    altitude, 
    wgs84_ecef
  ); 

  // We are notw ready to capture GPS data
  ready = true;
}
