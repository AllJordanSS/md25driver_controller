#include "motor_controller/PID_control.hpp"

PIDcontrol::PIDcontrol()
{
    kp = 0; ki = 0; kd = 0;
    lowB = 0; highB = 0;
    windupGuard = 0;
    error = 0; control = 0;
    pTerm = 0; iTerm = 0; dTerm = 0;

    state = 0; iState = 0;

    dLast = 0; dState = 0; 
}

void PIDcontrol::setGains(float KP, float KI, float KD)
{
    kp = KP;
    ki = KI;
    kd = KD;
}

void PIDcontrol::setBounds(float low, float high)
{
    lowB = low;
    highB = high;
}

void PIDcontrol::setWindupGuard(float gain)
{
    windupGuard = gain;
}

float PIDcontrol::updatePID(float reference, float value, float Ts)
{   
    error = reference - value;
    pTerm = error * kp;
    iTerm = integral(error, Ts);
    dTerm = derivative(error, Ts);

    control = pTerm + iTerm - dTerm;
    
    return saturation(control);
}

float PIDcontrol::integral(float error, float Ts)
{
    iState += Ts * state; 
    state = error;
  
    if(iState > windupGuard)
    {   
        iState = windupGuard;

    }
    else if (iState < -windupGuard)
    {   
        iState = -windupGuard;
    }
 
    return iState * ki;
}

float PIDcontrol::derivative(float error, float Ts)
{
    dState = (error - dLast) / Ts;
    dLast = error;
    return dState * kd;
}

float PIDcontrol::saturation(float control)
{
    if(control > highB)
    {
        return highB;
    }
    else if (control < lowB)
    {
        return lowB;
    }
    else
    {
        return control;
    }
}