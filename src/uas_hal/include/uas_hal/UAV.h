#ifndef UAS_HAL_UAV_H
#define UAS_HAL_UAV_H

// Basic ROS stuff
#include <ros/ros.h>

// This package's messages
#include <uas_hal/Peripheral.h>
#include <uas_hal/Navigation.h>
#include <uas_hal/Control.h>

// Messages broadcast by this class
#include <uas_hal/MsgInformation.h>
#include <uas_hal/MsgControl.h>
#include <uas_hal/MsgState.h>

namespace uas_hal
{
    class UAV : 
        
        public Peripheral<MsgAltitude>,       
        public Peripheral<MsgInertial>,
        public Peripheral<MsgMagnetic>,
        public Peripheral<MsgPosition>,
        public Peripheral<MsgAttitude>,

        public Control<Idle>,
        public Control<Takeoff>,
        public Control<Hover>,
        public Control<Land>,
        public Control<Emergency>,
        public Control<AnglesHeight>,
        public Control<Velocity>,
        public Control<Waypoint>,
        public Control<VelocityHeight>
    {       

    private:

        // MAINTAINS THE CURRENT TIME TICK ////////////////////////////////////////////////

        double          tick;

        // NODE HANDLE ////////////////////////////////////////////////////////////////////

        ros::NodeHandle node;

        // DATA FUSION ////////////////////////////////////////////////////////////////////

        Navigation      nav;

        // MESSAGES ///////////////////////////////////////////////////////////////////////

        MsgInformation  msgI;
        MsgControl      msgC;
        MsgState        msgS;

        // PUBLISHERS /////////////////////////////////////////////////////////////////////

        ros::Publisher  pubI;      // Information
        ros::Publisher  pubS;      // State
        ros::Publisher  pubC;      // Control

        // TIMERS /////////////////////////////////////////////////////////////////////////

        ros::Timer      timI;      // Timer
        ros::Timer      timS;      // Timer
        ros::Timer      timC;      // Timer
        ros::Timer      timU;      // Timer

        // TIMER CALLBACKS ////////////////////////////////////////////////////////////////

        void BroadcastInformation(const ros::TimerEvent& event);
        void BroadcastControl(const ros::TimerEvent& event);
        void BroadcastState(const ros::TimerEvent& event);

        // SYATEM PROPAGATION /////////////////////////////////////////////////////////////

        void Update(const ros::TimerEvent& event);
        
    protected:

        // PARENT CALLBACKS //////////////////////////////////////////////////////////////

        void Receive(const MsgAltitude &msg);
        void Receive(const MsgInertial &msg);
        void Receive(const MsgPosition &msg);
        void Receive(const MsgMagnetic &msg);
        void Receive(const MsgAttitude &msg);

        // CHILD CALLBACKS ////////////////////////////////////////////////////////////////
    
        virtual void AcceptControl(const MsgControl &ctl) = 0;

    public:

        // CONSTRUCTOR ////////////////////////////////////////////////////////////////////
    
        UAV(const char* name, ros::NodeHandle& node);

        // CONFIGURE MESSAGING RATES //////////////////////////////////////////////////////

        void Reset(const double &rateI, const double &rateC, const double &rateS);

    };
}

#endif