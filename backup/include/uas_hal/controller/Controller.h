#ifndef UAS_HAL_CONTROLLER_H
#define UAS_HAL_CONTROLLER_H

#include <cmath>

namespace uas_hal
{
    template<class G>
    template<class A>
    template<class R>
    class Controller
    {

    protected: 

        // Convert a navigation frame vector
        static void n2b(double rot[3], double vec[3])
        {
            // Precompute for speed
            double t[3], c[3], s[3];
            for (int i = 0; i < 3; i++)
            {
                t[i] = vec[i];
                c[i] = cos(rot[i]);
                s[i] = sin(rot[i]);
            }

            // Brute-force calculation of the following matrix time vec
            //     [          cy*cz,          cy*sz,            -sy]
            //     [ sy*sx*cz-sz*cx, sy*sx*sz+cz*cx,          cy*sx] 
            //     [ sy*cx*cz+sz*sx, sy*cx*sz-cz*sx,          cy*cx]
            vec[0] =                (c[1]*c[2])*t[0] +                (c[1]*s[2])*t[1] -      s[1]*t[2];
            vec[1] = (s[1]*s[0]*c[2]-s[2]*c[0])*t[0] + (s[1]*s[0]*s[2]+c[2]*c[0])*t[1] + c[1]*s[0]*t[2];
            vec[2] = (s[1]*c[0]*c[2]-s[2]*s[0])*t[0] + (s[1]*c[0]*s[2]+c[2]*s[0])*t[1] + c[1]*c[0]*t[2];
        }
    
        // Clamp a value to exist between two intervals
        static double limit(double val, double minval, double maxval)
        {
            if (val < minval) return minval;
            if (val > maxval) return maxval;
            return val;
        }

    public:

        // Pass the state and discrete time step and receive a control back
        virtual void Update(State* state, double dt, Control* ctl) = 0;

        // Reset the controller
        virtual void Reset() = 0;

        virtual void Preempt() = 0;

        virtual void SetGoal(const &G) = 0;

    };
}

#endif