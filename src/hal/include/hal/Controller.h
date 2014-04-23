#ifndef HAL_CONTROLLER_H
#define HAL_CONTROLLER_H

#include <algorithm>
#include <map>
#include <string>
#include <cstdarg>
#include <hal/HAL.h>

namespace hal
{
    namespace controller
    {
        //! Controller comparator
        /*!
          This is a simple comparator function that searches a 2D string lookup table to 
          check whether a transition between controllers is permitted.
        */
        class Comparator : public std::unary_function < std::pair < std::string, std::string >, bool >
        {
            explicit Comparator(const std::string& a, std::string& b) : from(a), to(b) {}
            bool operator() (const std::pair<std::string,std::string> &arg)
            { 
                return ((arg.first.compare(from)==0) && (arg.second.compare(to)==0));
            }
            std::string from, to;
        };

        //! Controller updateable
        /*!
          The objective of any controller is to determine the control that takes the current
          state to some goal state, given a discrete time tick. As such, each derived controller
          must support some Update(.) function call. We introduce a pure abstract base class so
          that each controller may be downcasted and treated equally.
        */
        template <class STATE, class CONTROL>
        class ControllerBase
        {
        public:

            //! Perform a control update, given a state and discrete time update
            /*!
              \param state the current state value
              \param dt the discrete time step
              \return the resultant control
            */
            virtual CONTROL Update(const STATE &state, const double &dt) = 0;       
        
            //! Goal reach check
            /*!
              \return whether the goal has been reached
            */
            virtual bool HasGoalBeenReached() = 0;
        };

        //! Controller core
        /*!
          Each controller receives goals over the ROS network, and therefore it offers a service.
          This class deals with the creation of such a service, as well as the ability to switch
          between controllers. The deriv
        */
        template <class STATE, class CONTROL, class REQUEST, class RESPONSE>
        class Controller : public hal::HAL, public ControllerBase<STATE,CONTROL>
        {

        private:

            /// A list of all controllers
            static std::map<std::string,ControllerBase<STATE,CONTROL>*> controllers;

            /// A list of permissable transitions
            static std::map<std::pair<std::string,std::string>,bool> allowed;

            /// The current controller
            static std::string current;

        protected: 

            // INSTANCE METHODS /////////////////////////////////////////////////////////

            /// Name of this controller
            const char* name;

            /// Used to receive a new goal
            ros::ServiceServer service;
          
            //! Receive a control message from the ROS backbone
            /*!
              \param req the goal request
              \param res the goal response
              \return whether the control was accepted
            */
            bool Receive(REQUEST &req, RESPONSE &res);

            // STATIC METHODS /////////////////////////////////////////////////////////

            //! Project a body-frame point to a navigation-frame point
            /*!
              \param rot the 3x3 rotation matrix
              \param vec the sample point
            */
            static void n2b(double rot[3], double vec[3]);

            //! Clamp a value to within a range
            /*!
              \param val the sample value
              \param minval the lower bound
              \param maxval the upper bound
              \return the resultant value
            */
            static double limit(const double& val, const double& minval, const double& maxval);

            //! Try and switch to the current controller
            /*!
              \return whether the controller switch was permitted
            */
            bool Switch(); 

        public:

            // INSTANCE METHODS /////////////////////////////////////////////////////////

            //! Create a controller object
            /*!
              \param name name of this controller
              \return new object
            */
            Controller(const char* n);

            /// Reset the current state
            virtual void Reset() = 0;

            // STATIC METHODS /////////////////////////////////////////////////////////

            //! Add a permissable transition that occurs immediately
            /*!
              \param controller list of controllers
              \return whether the transition could be added
            */
            static bool PermitInstant(const char* from, const char* to);

            //! Add a permissable transition that must wait for current to complete 
            /*!
              \param from the active controller
              \param to the proposed controller
              \return whether the transition could be added
            */
            static bool PermitQueued(const char* from, const char* to);

            //! Switch to a new controller 
            /*!
              \param controller the new controller
              \return whether the transition is allowed
            */
            static bool Switch(const char* controller);

            //! Get the control for the current 
            /*!
              \param state the current state
              \param dt the time tick
              \return the control vector
            */
            static CONTROL GetControl(const STATE &state, const double &dt);    

            //! Set the current controller
            /*!
              \param controller the new controller
              \return whether the transition is allowed
            */
            static bool SetController(const char* controller);                      
        };
    }
}

#endif