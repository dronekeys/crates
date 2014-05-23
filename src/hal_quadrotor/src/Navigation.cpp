#include <hal/quadrotor/Navigation.h>

using namespace hal::quadrotor;

void Navigation::Reset()
{
  // What to do here?
}

void Navigation::GetState(hal_quadrotor::State &msg)
{
  msg = state;
}

void Navigation::SetState(const hal_quadrotor::State &msg)
{
  state = msg;
}

void Navigation::Process(const hal_sensor_altimeter::Data &msg)
{
  //ROS_WARN("Incoming ALT: %f %f", msg.z, msg.w);
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
  //ROS_WARN("Incoming GPS: %f %f %f %f", msg.x, msg.y, msg.u, msg.v);
  state.x = msg.x;
  state.y = msg.y;
  state.u = msg.u;
  state.v = msg.v;
}

void Navigation::Process(const hal_sensor_orientation::Data &msg)
{
  //ROS_WARN("Incoming ROT: %f %f %f %f %f %f", msg.roll, msg.pitch, msg.yaw, msg.p, msg.q, msg.r);
  state.roll  = msg.roll;
  state.pitch = msg.pitch;
  state.yaw   = msg.yaw;
  state.p     = msg.p;
  state.q     = msg.q;
  state.r     = msg.r;
}