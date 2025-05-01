#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>
#include <cmath>
#include <tuple>

class MD25Driver
{
private:
    //! @note I2C variables
    int fd;	                                                           
    unsigned char buffer[10];

    //! @note MD25 parameters
    int mode, acceleration;
    bool regulator, timeout;
    float leftVelRPM, rightVelRPM;

    //! @note Encoder variables
    int lastLeftEncoder, lastRightEncoder;
    int leftEncoder, rightEncoder;
    float cpr, motorRatio, deltaTime;
    float leftDeltaCounts, rightDeltaCounts;
    float lastLeftDeltaCounts, lastRightDeltaCounts;

public:
    //! @note Class constructor
    MD25Driver(int address, const char *i2c_port);

    //! @note Class destructor
    ~MD25Driver();

    //! @note Function to initiate the variables
    void initVariables();

    //! @note Function to check Md25 module
    void checkDevice(int address, const char *i2c_port);

    //! @note Function to configure driver paramameters
    void configureDevice(float cpr_, float motorRatio_, float rate);

    //! @note Function to get the software version
    void getSoftwareVersion();

    //! @note Function to get the driver mode
    void getMode();

    //! @note Function to read the driver mode status
    int readMode();

    //! @note Function to set the driver mode
    void setMode(int new_mode);

    //! @note Function to get the motor velocities
    std::tuple<int,int,float,float,float,float> getVelocities();

    //! @note Function to get the acceleration rate  
    void getAcceleration();

    //! @note Function to read the acceleration rate status
    int readAcceleration();

    //! @note Function to set the acceleration rate  
    void setAcceleration(int new_acceleration);

    //! @note Function to read the encoder data
    void readEncoders();

    //! @note Function to reset the encoders 
    void resetEncoders();

    //! @note Function to stop the motors
    void stopMotors();

    //! @note Function to get the battery voltage
    int getBatteryVoltage();

    //! @note Function to get the current of the motors
    std::tuple<int,int> getCurrents();

    //! @note Function to enable the speed regulation
    void enableRegulation();

    //! @note Function to disable the speed regulation
    void disableRegulation();

    //! @note Function to read the speed regulation status
    int readRegulation();

    //! @note Function to enable the timeout of motors
    void enableTimeout();

    //! @note Function to disable the timeout of motors
    void disableTimeout();

    //! @note Function to read the timeout of motors status
    int readTimeout();

    //! @note Functions to set the velocity of motors
    void setLeftMotorVelocity(float leftVelocity);
    void setRightMotorVelocity(float rightVelocity);

    //! @note Function to remap voltage in the range 0-255
    uint8_t voltageToInt(float valueVolts);
    
    //! @note Function to convert encoder counts to RPM
    void countsToRPM();

    //! @note Function to filter encoder data
    void filterDelta();
};
