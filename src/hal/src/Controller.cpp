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
Controller<STATE,CONTROL,REQUEST,RESPONSE>::Controller(const char* n) : name(n), ControllerBase<STATE,CONTROL>()
{
    service = rosNode.advertiseService(n, &Controller<STATE,CONTROL,REQUEST,RESPONSE>::Receive, this);
}