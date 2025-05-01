
#include <iostream>

class PIDcontrol
{
    private:
    //! @note Controller parameters
    float kp, ki, kd;
    float lowB, highB;
    float windupGuard;
    float error, control; 
    float pTerm, iTerm, dTerm;

    //! @note Integral parameters
    float state, iState;

    //! @note Derivative parameters
    float dLast, dState;

    public:
    //! @note Constructor
    PIDcontrol();

    //! @note Set controller gains
    void setGains(float KP, float KI, float KD);

    //! @note Set controller bounds
    void setBounds(float low, float high);

    //! @note Set controller windup guard parameter
    void setWindupGuard(float gain);

    //! @note Update control signal
    float updatePID(float reference, float value, float Ts);

    //! @note Integral function
    float integral(float error, float Ts);

    //! @note Derivative function
    float derivative(float error, float Ts);

    //! @note Saturation function
    float saturation(float control);
};