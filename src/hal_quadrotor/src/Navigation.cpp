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
  state.z = msg.z;
  state.w = msg.w;
}

void Navigation::Process(const hal_sensor_compass::Data &msg)
{
  // Do nothing
}

void Navigation::Process(const hal_sensor_imu::Data &msg)
{
  state.p = msg.p;
  state.q = msg.q;
  state.r = msg.r;
}

void Navigation::Process(const hal_sensor_gnss::Data &msg)
{
  state.x = msg.x;
  state.y = msg.y;
  state.u = msg.u;
  state.v = msg.v;
}

void Navigation::Process(const hal_sensor_orientation::Data &msg)
{
  state.roll  = msg.roll;
  state.pitch = msg.pitch;
  state.yaw   = msg.yaw;
}