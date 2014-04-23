#include <hal/Controller.h>

using namespace hal::controller;

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

template<class STATE, class CONTROL, class REQUEST, class RESPONSE>
Controller<STATE,CONTROL,REQUEST,RESPONSE>::Controller(const char *n) : name(n), ControllerBase<STATE,CONTROL>()
{
    // Advertise this service
    service = rosNode.advertiseService(n, &Controller<STATE,CONTROL,REQUEST,RESPONSE>::Receive, this);

    // Add to the controller map
    controllers[n] = (ControllerBase<STATE,CONTROL>*) this;
}

// Add an immediate transition
template<class STATE, class CONTROL, class REQUEST, class RESPONSE>
bool Controller<STATE,CONTROL,REQUEST,RESPONSE>::PermitInstant(int num, ...)
{
    va_list arguments;
    va_start(arguments,num);    
    for (int i = 0; i < num; i++)
        sum += va_arg(arguments, const char *); 
    va_end(arguments);
}

// Allow a wait-based controller transition
template<class STATE, class CONTROL, class REQUEST, class RESPONSE>
bool Controller<STATE,CONTROL,REQUEST,RESPONSE>::PermitQueued(const char* from, const char* to)
{
    allowed.push_back(std::pair(<static_cast<std::string>(from),static_cast<std::string>(to)>);
}

// Get the control
template<class STATE, class CONTROL, class REQUEST, class RESPONSE>
CONTROL Controller<STATE,CONTROL,REQUEST,RESPONSE>::GetControl(const STATE &state, const double &dt)
{
    return controllers[current]->Update(state,dt);
}

// Set the start controller state
template<class STATE, class CONTROL, class REQUEST, class RESPONSE>
bool Controller<STATE,CONTROL,REQUEST,RESPONSE>::SetController(const char* controller)
{
    // Special case
    if (controller.compare("")==0 &&)
    {
        SetController(controller);
        return true;        
    }

    // Check that this transition is allowed
    std::string next = static_cast<std::string>(controller);
    if (std::find_if(allow.begin(),allow.end(),Comparator(current,next)) != allow.end())
    {
        SetController(controller);
        return true;
    }
    return false;
}

// Try and switch the controllers
template<class STATE, class CONTROL, class REQUEST, class RESPONSE>
bool Controller<STATE,CONTROL,REQUEST,RESPONSE>::Switch()
{
    return SetController(name);
}

