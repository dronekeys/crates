#ifndef HAL_QUADROTOR_H
#define HAL_QUADROTOR_H

// System libraries
#include <string>

// Basic ROS stuff
#include <ros/ros.h>

//  Buffered topic broadcasting
#include <hal_quadrotor/Topic.h>

// Calculate a navigation solution
#include <hal_quadrotor/Navigation.h>

// Messages
#include <hal_quadrotor/Altitude.h>
#include <hal_quadrotor/Inertial.h>
#include <hal_quadrotor/Magnetic.h>
#include <hal_quadrotor/Position.h>
#include <hal_quadrotor/Energy.h>
#include <hal_quadrotor/Orientation.h>
#include <hal_quadrotor/Information.h>
#include <hal_quadrotor/State.h>
#include <hal_quadrotor/Control.h>

// Services (controllers)
#include <hal_quadrotor/controller/Emergency.h>
#include <hal_quadrotor/controller/Hover.h>
#include <hal_quadrotor/controller/Idle.h>
#include <hal_quadrotor/controller/Land.h>
#include <hal_quadrotor/controller/Takeoff.h>
#include <hal_quadrotor/controller/AnglesHeight.h>
#include <hal_quadrotor/controller/Velocity.h>
#include <hal_quadrotor/controller/VelocityHeight.h>
#include <hal_quadrotor/controller/Waypoint.h>

// Services (configuration)
#include <hal_quadrotor/configuration/Rate.h>

namespace hal_quadrotor
{
    /**
    * The 
    * - hal
    *  - uav
    *   - identifier
    *    - information............ General information about this platform
    *    - state.................. The current platform state
    *    - control................ The last control sent to the device
    *    - config
    *      - rate................. Set the rate of a given message
    *    - sensors
    *     - inertial.............. Body frame angular velocity, linear acceleration
    *     - magnetic.............. Body frame magnetic field strength
    *     - altitude.............. Altitude relative to ground plane
    *     - position.............. GNSS solution in WGS84, velocity, DOP
    *     - orientation........... Euler, quaternion, DCM orientation from fusion
    *    - actions
    *     - emergency............. Turn the rotors off
    *     - idle.................. On the ground, rotors off
    *     - hover................. Hover in place 
    *     - takeoff............... Move from the ground to given altitude
    *     - land.................. Move from the current position to the ground
    *     - anglesheight.......... Change the angles and height of the UAV
    *     - velocity.............. Maintain 3D velocity and heading
    *     - velocityheight........ Maintain 2D velocity, altitude and heading
    *     - waypoint.............. Move to a waypoint and hover
    */
    class HAL : 
        public Topic<Information>,
        public Topic<Control>,
        public Topic<State>,
        public Topic<Altitude>,       
        public Topic<Inertial>,
        public Topic<Magnetic>,
        public Topic<Energy>,
        public Topic<Position>,
        public Topic<Orientation>,
        public Idle,
        public Takeoff,
        public Hover,
        public Land,
        public Emergency,
        public AnglesHeight,
        public Velocity,
        public Waypoint,
        public VelocityHeight,
        public Rate
    {       

    private:

        // Last time at which the internal state was updated
        double tick;

        // Navigation system for fusing measurements into a state estimate
        Navigation nav;

        // Timer for state update callbacks
        ros::Timer timer;

        // This is called on every platform-level controller clock tick
        void Update(const ros::TimerEvent& event);
        
    protected:

        // Handle to the ROS node
        ros::NodeHandle rosNode;

        // Pass altiude measurement to navigation subsystem
        void Receive(const Altitude &msg);

        // Pass inertial measurement to navigation subsystem
        void Receive(const Inertial &msg);

        // Pass position measurement to navigation subsystem
        void Receive(const Position &msg);

        // Pass magnetic measurement to navigation subsystem
        void Receive(const Magnetic &msg);

        // Pass magnetic measurement to navigation subsystem
        void Receive(const Energy &msg);

        // Pass orientation measurement to navigation subsystem
        void Receive(const Orientation &msg);

        // Pass state measurement to navigation subsystem
        void Receive(const State &msg);

        // Receive a rate configuration updates
        bool ConfigRate(const std::string& name, const double& rate);

        // This must be overriden by the child class in order to accept control 
        virtual void Receive(const Control &ctl) = 0;

    public:

        // CONSTRUCTOR ////////////////////////////////////////////////////////////////////
    
        HAL(const char *name);

    };
}

#endif