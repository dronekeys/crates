#include <uas_hal/Navigation.h>

using namespace uas_hal;

// The four primary sensors that are fused
void Navigation::Measurement(const MsgAltitude &msg)
{
	// Set the data
	state.epoch = msg.epoch;			// Time
	state.z 	= msg.z;				// Altitude
	state.w 	= msg.w;				// Vertical altitude
}

// Add an inertial measurement to the navigation filter
void Navigation::Measurement(const MsgInertial &msg)
{
	state.epoch = msg.epoch;			// Time
	state.p 	= msg.p;				// Body-frame X ang vel
	state.q 	= msg.q;				// Body-frame Y ang vel
	state.r 	= msg.r;				// Body-frame Z ang vel
}

// Add a position measurement to the navigation filter
void Navigation::Measurement(const MsgPosition &msg)
{
	state.epoch = msg.epoch;			// Time
	state.x 	= msg.x;				// Nav-frame X pos
	state.y 	= msg.y;				// Nav-frame Y pos
	state.u 	= msg.u;				// Nav-frame X vel
	state.v 	= msg.v;				// Nav-frame Y vel
}

// Add a magnetic measurement to the navigation fiter
void Navigation::Measurement(const MsgMagnetic &msg)
{
	// Do nothing yet
}

// Add a magnetic measurement to the navigation fiter
void Navigation::Measurement(const MsgAttitude &msg)
{
	// Set the data
	state.epoch = msg.epoch;			// Time
	state.roll 	= msg.roll;				// Roll
	state.pitch = msg.pitch;			// Pitch
	state.yaw   = msg.yaw;				// Yaw
}

// Query the state
const MsgState& Navigation::GetState()
{
	return state;
}