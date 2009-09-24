/** PID Controller:

 @author Jason Kulk

NUbots (c) 2008 All Rights Reserved
*/

#include "pidcontroller.h"


/*! Create a generic PID Controller
 
 @param mname                   the name of the PID controller (this is used to idenitify it)
 @param mKp                     the proportional gain
 @param mKi                     the integra gain
 @param mKd                     the derivative gain
 @param mperiod                 the control loop period
 @param moutputlowerlimit       the actuator lower limit
 @param moutputupperlimit       the actuator upper limit
 */
PIDController::PIDController(std::string mname, float mKp, float mKi, float mKd, float mperiod, float moutputlowerlimit, float moutputupperlimit)
{
    name = mname;
    Kp = mKp; 
    Ki = mKi;
    Kd = mKd;
    period = mperiod;
    outputlowerlimit = moutputlowerlimit;
    outputupperlimit = moutputupperlimit;
    
    target = 0;
    
    error = 0;
    previouserror = 0;
    proportional = 0;
    previousproportional = 0;
    integral = 0;
    previousintegral = 0;
    derivative = 0;
    previousderivative = 0;
    
    d_constant = 10*Kd;
    if (Kd != 0)
        a_constant = sqrt(Ki/Kd);
    else
        a_constant = 0.5*(Ki/Kp);
    
    if (a_constant > 1)
        a_constant = 1.0;
#if PIDCONTROLLER_DEBUG > 0
    name.append(".csv");
    name.insert(0,"/var/log/");
    cout << "PID: Log filename " << name.c_str() << endl;
    pidLog.open(name.c_str());
    pidLog << "Time (ms), Error, Proportional, Integral, Derivative, Unlimited, Output" << endl;
#endif
}

/*! Destroys the PID controller
 */
PIDController::~PIDController()
{
#if PIDCONTROLLER_DEBUG > 0
    pidLog.close();
#endif
}
 
/*! The controller function; call this inside the control loop.
 
 @param input                   the measured value
 
 @return                        the calculated control output
 */
float PIDController::doControl(float input)
{
    float unlimitedoutput, output;
    error = target - input;
    
    proportional = Kp*error;
    derivative = (Kd*(error - previouserror) + d_constant*previousderivative)/(period + d_constant);
    
    unlimitedoutput = proportional + previousintegral + derivative;
    output = limitOutput(unlimitedoutput);
    
    // calculate the integral term with anti-windup
    integral = integral + Ki*period*error + a_constant*(output - unlimitedoutput); 

    // update controller state
    previouserror = error;
    previousproportional = proportional;
    previousintegral = integral;
    previousderivative = derivative;
#if PIDCONTROLLER_DEBUG > 0
    pidLog << dcmTimeSinceStart << ", " << error << ", " << proportional << ", " << integral << ", " << derivative << ", " << unlimitedoutput << ", " << output << endl;
#endif
    return output;
}

/*! Set the controller's set point
 
 @param mtarget                 the target current (A)
 */
void PIDController::setTarget(float mtarget)
{
    target = mtarget;
    return;
}

/*! Clear the controller's state.
 
 This will reset the integral and derivative terms, as well as reset any historical values.
 Use this just prior to turning the motor back on, or when turning the motor off
 */
void PIDController::clearState()
{
    integral = 0;
    previousproportional = 0;
    previousderivative = 0;
    previousintegral = 0;
    previouserror = 0;
    return;
}
        

float PIDController::limitOutput(float unlimitedoutput)
{
    if (unlimitedoutput > outputupperlimit)
        return outputupperlimit;
    if (unlimitedoutput < outputlowerlimit)
        return outputlowerlimit;
    return unlimitedoutput;
}
