#include <hal/Controller.h>

using namespace hal::controller;

// CONTROLLERBASE IMPLEMENTATIONS ////////////////////////////////////////////////////////////////////////

// Initialise current state
template<class STATE, class CONTROL> 
std::string ControllerBase<STATE,CONTROL>::current = "";

// Add an immediate transition
template<class STATE, class CONTROL>
bool ControllerBase<STATE,CONTROL>::PermitInstant(const char* a, const char* b)
{
    if (b==NULL)
    {
        typename std::map<std::string,ControllerBase<STATE,CONTROL>*>::iterator iter;
        for (iter = controllers.begin(); iter != controllers.end(); ++iter)
            allowed[std::make_pair(iter->first,static_cast<std::string>(a))] = true;
    }
    else
        allowed[std::make_pair(static_cast<std::string>(a),static_cast<std::string>(b))] = true;
}

// Allow a wait-based controller transition
template<class STATE, class CONTROL>
bool ControllerBase<STATE,CONTROL>::PermitQueued(const char* a, const char* b)
{
    {
        typename std::map<std::string,ControllerBase<STATE,CONTROL>*>::iterator iter;
        for (iter = controllers.begin(); iter != controllers.end(); ++iter)
            allowed[std::make_pair(iter->first,static_cast<std::string>(a))] = false;
    }
        allowed[std::make_pair(static_cast<std::string>(a),static_cast<std::string>(b))] = false;
}

// Get the control
template<class STATE, class CONTROL>
CONTROL ControllerBase<STATE,CONTROL>::GetControl(const STATE &state, const double &dt)
{
    // Call Update() on the current controller
    return controllers[current]->Update(state,dt);
}

// Set the start controller state
template<class STATE, class CONTROL>
bool ControllerBase<STATE,CONTROL>::SetController(const char* controller)
{
    // Check that this transition is allowed
    std::string next = static_cast<std::string>(controller);

    // Special case : no initial state set
    if (current.compare("")==0)
    {
        current = next;
        return true;
    }

    // Check if transition is permissable
    if (std::find_if(allowed.begin(),allowed.end(),Comparator(current,next)) != allowed.end())
    {
        if (allowed[std::make_pair(current,next)] || controllers[current]->HasGoalBeenReached())
        {
            current = next;
            return true;
        }
    }

    // Default is denial
    return false;
}

// CONTROLLER IMPLEMENTATIONS ////////////////////////////////////////////////////////////////////////

template<class STATE, class CONTROL, class REQUEST, class RESPONSE>
Controller<STATE,CONTROL,REQUEST,RESPONSE>::Controller(const char *n) : name(n), ControllerBase<STATE,CONTROL>()
{
    // Advertise this service
    service = rosNode.advertiseService(n, &Controller<STATE,CONTROL,REQUEST,RESPONSE>::Receive, this);

    // Add to the controller map
    ControllerBase<STATE,CONTROL>::controllers[static_cast<std::string>(n)] = (ControllerBase<STATE,CONTROL>*) this;
}

template<class STATE, class CONTROL, class REQUEST, class RESPONSE>
void Controller<STATE,CONTROL,REQUEST,RESPONSE>::n2b(double rot[3], double vec[3])
{
    double t[3], c[3], s[3];
    for (int i = 0; i < 3; i++)
    {
        t[i] = vec[i];
        c[i] = cos(rot[i]);
        s[i] = sin(rot[i]);
    }
    vec[0] =                (c[1]*c[2])*t[0] +                (c[1]*s[2])*t[1] -      s[1]*t[2];
    vec[1] = (s[1]*s[0]*c[2]-s[2]*c[0])*t[0] + (s[1]*s[0]*s[2]+c[2]*c[0])*t[1] + c[1]*s[0]*t[2];
    vec[2] = (s[1]*c[0]*c[2]-s[2]*s[0])*t[0] + (s[1]*c[0]*s[2]+c[2]*s[0])*t[1] + c[1]*c[0]*t[2];
}

template<class STATE, class CONTROL, class REQUEST, class RESPONSE>
double Controller<STATE,CONTROL,REQUEST,RESPONSE>::limit(const double& val, const double& minval, const double& maxval)
{
    if (val < minval) return minval;
    if (val > maxval) return maxval;
    return val;
}

// Try and switch the controllers
template<class STATE, class CONTROL, class REQUEST, class RESPONSE>
bool Controller<STATE,CONTROL,REQUEST,RESPONSE>::Switch()
{
    return ControllerBase<STATE,CONTROL>::SetController(name);
}

