#ifndef UAS_HAL_CONTROLLER_H
#define UAS_HAL_CONTROLLER_H

// Binding capabilities
#include <boost/bind.hpp>

// Actual controller implementations
#include <uas_hal/controller/AnglesHeight.h>
/*
#include <uas_hal/controller/Emergency.h>
#include <uas_hal/controller/Hover.h>
#include <uas_hal/controller/Idle.h>
#include <uas_hal/controller/Land.h>
#include <uas_hal/controller/Takeoff.h>
#include <uas_hal/controller/Velocity.h>
#include <uas_hal/controller/VelocityHeight.h>
#include <uas_hal/controller/Waypoint.h>
*/

// Exploit resuable action interface
#include <actionlib/server/simple_action_server.h>

// Default peripheral data queue length before discard
#define DEFAULT_QUEUE_LENGTH      10
#define DEFAULT_PROGRESS_RATE    1.0
#define DEFAULT_UPDATE_RATE     50.0
#define DEFAULT_TIMEOUT_RATE     1.0

namespace uas_hal
{
    // T is the message type sent by the peripheral
    template <class C>  // Controller class
    template <class A>  // Action message
    template <class G>  // Goal message
    template <class F>  // Feedback message
    template <class R>  // Result message
    class Controller
    {

    private:

        // STATIC MEMBERS //////////////////////////////////////////////////////////

        // The current controller
        static Controller* controller;

        // State and control
        static MsgControl control;
        static MsgState   state;

        // Timers for control and progress
        static ros::Timer timerControl;
        static ros::Timer timerProgress;

        // INSTANCE MEMBERS ////////////////////////////////////////////////////////

        // Reusable action code
        actionlib::SimpleActionServer<A> action;

        // ROS node handle
        ros::NodeHandle node;

        // Publishers for progress and updare
        ros::Publisher  publisherProgress;    // Publisher
        ros::Publisher  publisherUpdate;      // Publisher

        // Messages
        C controller; 
        F feedback; 
        R result;
        
        // Callback when a new action is received
        void Goal(const G &goal)
        {
            // Update the state, given the control
            controller.SetGoal(goal);

            // Since the controllers below are static, the previous control action will
            // be stopped immediately and replaced by this one.

            // Create a timer callback
            timerControl = node.createTimer(
                ros::Duration(1.0/DEFAULT_UPDATE_RATE),   &Controller::Control, this
            );

            // Create a timer callback
            timerProgress = node.createTimer(
                ros::Duration(1.0/DEFAULT_PROGRESS_RATE), &Controller::Progress, this
            );
        }

        // Send progress to the user (called at about 1Hz)
        void Progress(const ros::TimerEvent& event)
        {
            // Get the feedback from the controller
            controller.GetFeedback(feedback);

            // Publish the feedback
            action.publishFeedback(feedback);
        }

        // Issue control to the HAL (called at about 50Hz)
        void Control(const ros::TimerEvent& event)
        {
            // Update the state, given the control
            if (controller.GetControl(state,control))
            {
                // Get the result
                controller.GetResult(result);

                // Mark as successful
                action.setSucceeded(result);
            }
        }

    public:

        // Constructor - creates the action.
        Controller(ros::NodeHandle &h, const char *name) : node(h)
            : action(h,name,boost::bind(&Controller::Goal,this,_1),false)
        {
            // Start listening for this control action
            action.start();
        }

        // Update all controllers with a new state
        static void SetState(const MsgState &newstate)
        {
            state = newstate;
        }
    };
}

#endif