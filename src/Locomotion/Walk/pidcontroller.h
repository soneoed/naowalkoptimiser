/** Generic PID Controller
   

   @author Jason Kulk

   NUbots (c) 2008 All Rights Reserved

 */

#include "jwalkincludes.h"
#include "sensors.h"
#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#define PIDCONTROLLER_DEBUG     1

class PIDController
{
public:
    PIDController(std::string name, float Kp, float Ki, float Kd, float period, float outputlowerlimit, float outputupperlimit);
    ~PIDController();
    
    float doControl(float input);
    void setTarget(float target);
    
    void clearState();
    
private:
    std::string name;                           // The name of the controller (used to idenitify it amoung ther controllers)
    
    // PID Controller constants
    float Kp, Ki, Kd;                 // The proportional, integral and derivative gains
    float period;                               // The period of the control loop
    float outputupperlimit, outputlowerlimit;   // The upper and lower limits of the controller output    
    float d_constant, a_constant;               // The derivative filter constant, and the anti-windup filter constant
    
    // PID Controller variables
    float target;                               // The current set point
    float error, previouserror;                 // The error between the set point and the actual value
    float proportional, previousproportional;   // The proportional term
    double integral, previousintegral;          // The integral term
    float derivative, previousderivative;       // The derivative term
    
    float limitOutput(float unlimitedoutput);   // Clip the controller output to the actuator limits
    
#if PIDCONTROLLER_DEBUG > 0
    ofstream pidLog;
#endif
};


#endif
