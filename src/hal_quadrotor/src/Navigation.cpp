#include <hal_quadrotor/Navigation.h>

#define DEBUG false

#define MASK_GPS  0b00000001
#define MASK_ALT  0b00000010
#define MASK_MAG  0b00000100
#define MASK_IMU  0b00001000
#define MASK_ROT  0b00010000
#define MASK_ALL  0b00011111

using namespace hal::quadrotor;

Navigation::Navigation() : data(0x0), ready(false),
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

bool Navigation::GetState(hal_quadrotor::State &msg)
{
  msg   = state;
  msg.t = ros::Time::now().toSec();
  
  // Only successful if all data has been received
  return (data==MASK_ALL);
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

  // Update mask
  data |= MASK_ALT;

  // Information
  if (DEBUG) ROS_INFO("A %f-- POS %f VEL: %f", msg.t, 
    state.z, state.w);
}

void Navigation::Process(const hal_sensor_compass::Data &msg)
{
  // Update mask
  data |= MASK_MAG;

  // Information
  if (DEBUG) ROS_INFO("C %f -- MAG %f %f %f", msg.t, 
    msg.x, msg.y, msg.z);
}

void Navigation::Process(const hal_sensor_imu::Data &msg)
{
  // Update mask
  data |= MASK_IMU;

  // Information
  if (DEBUG) ROS_INFO("I %f -- ANG %f %f %f ACC %f %f %f", msg.t,
    msg.p, msg.q, msg.r, msg.du, msg.dv, msg.dw);
}

void Navigation::Process(const hal_sensor_gnss::Data &msg)
{
  // If the LTP origin was set
  if (ready)
  {
    // Used to discard height
    double x, y, h;
    
    // Recover LTP coordinates from the WGS84 message
    wgs84_enu.Forward(
      msg.latitude, 
      msg.longitude, 
      msg.altitude,
      x,
      y,
      h
    );

    // Set the values
    state.x = x;
    state.y = y;
    state.u = msg.vel_ew;
    state.v = msg.vel_ns;

    // Update mask
    data |= MASK_GPS;

    // Information
    if (DEBUG) ROS_INFO("G %f -- POS %f %f %f LTP: %f %f %f", msg.t,
      msg.latitude, msg.longitude, msg.altitude, x, y, h);
  }

}

void Navigation::Process(const hal_sensor_orientation::Data &msg)
{
  state.roll  = msg.roll;
  state.pitch = msg.pitch;
  state.yaw   = msg.yaw;
  state.p     = msg.p;
  state.q     = msg.q;
  state.r     = msg.r;

  // Update mask
  data |= MASK_ROT;

  // Information
  if (DEBUG) ROS_INFO("O %f -- POS %f %f %f VEL: %f %f %f", msg.t, 
    msg.roll, msg.pitch, msg.yaw, msg.p, msg.q, msg.r);

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
