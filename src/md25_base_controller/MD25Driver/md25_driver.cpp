#include "md25_base_controller/MD25Driver/md25_driver.hpp"

MD25Driver::MD25Driver(int address, const char *i2c_port)
{
    this->initVariables();
    this->checkDevice(address, i2c_port);
    this->getSoftwareVersion();
    this->resetEncoders();
}

MD25Driver::~MD25Driver()
{
	stopMotors();
}

void MD25Driver::initVariables()
{   
    //! @note MD25 parameters
    mode = 0;            //default
    acceleration = 5;    //default
    regulator = true;    //default
    timeout = true;      //default

    //! @note Encoder variables
    lastLeftEncoder = 0; lastRightEncoder = 0;
    leftEncoder = 0; rightEncoder = 0;
    cpr = 0.0; motorRatio = 0.0; deltaTime = 0.0;
    leftDeltaCounts = 0.0; rightDeltaCounts = 0.0;
    lastLeftDeltaCounts = 0.0; lastRightDeltaCounts = 0.0;
}

void MD25Driver::checkDevice(int address, const char *i2c_port)
{	
	// Open port for reading and writing
	if ((fd = open(i2c_port, O_RDWR)) < 0) 
	{					
		throw std::runtime_error("Failed to open i2c port\n");
		exit(1);
	}
	
	// Set the port options and set the address of the device we wish to speak to
	if (ioctl(fd, I2C_SLAVE, address) < 0) 
	{					
		throw std::runtime_error("Unable to get bus access to talk to slave\n");
		exit(1);
	}
}

void MD25Driver::configureDevice(float cpr_, float motorRatio_, float rate)
{
    cpr = cpr_;
    motorRatio = motorRatio_;
    deltaTime=(1.0/rate);
}

void MD25Driver::getSoftwareVersion()
{
	std::cout << "Getting Software Version" << std::endl;

    // This is the register we wish to read software version from
	buffer[0] = 13;		
	
    // Send regiter to read software from from
	if ((write(fd, buffer, 1)) != 1) 
    {			
		throw std::runtime_error("Error writing to i2c slave\n");
		exit(1);
	}
	
    // Read back data into buf[]
	if (read(fd, buffer, 1) != 1) 
    {								
		throw std::runtime_error("Unable to read from slave\n");
		exit(1);
	}
	else 
	{
		std::cout << "Software version: " << static_cast<unsigned int>(buffer[0]) << std::endl;
	}
}

void MD25Driver::getMode()
{
	buffer[0]=15;

	if((write(fd, buffer, 1)) != 1)
	{	
        // Send regiter to read software from from
		throw std::runtime_error("Error writing to i2c slave\n");
		exit(1);
	}
	
	if (read(fd, buffer, 1) != 1)
	{	
        // Read back data into buf[]
		throw std::runtime_error("Unable to read from slave\n");
		exit(1);
	}
	else
	{
		mode = buffer[0];
	}
}

int MD25Driver::readMode()
{
	return mode;
}

void MD25Driver::setMode(int new_mode)
{
	buffer[0] = 15;													
	buffer[1] = new_mode;	
    
    // A speed of 128 stops the motor
	if ((write(fd, buffer, 2)) != 2)
	{
		throw std::runtime_error("Error writing to i2c slave\n");
		exit(1);
	}

	mode = new_mode;
}

std::tuple<int,int,float,float,float,float> MD25Driver::getVelocities()
{
	this->readEncoders();

	leftDeltaCounts=(leftEncoder - lastLeftEncoder);
	rightDeltaCounts=(rightEncoder - lastRightEncoder);

	filterDelta();
	countsToRPM();

	lastLeftEncoder = leftEncoder;
	lastRightEncoder= rightEncoder;

	return std::make_tuple(leftEncoder, rightEncoder, leftVelRPM, rightVelRPM,
		leftDeltaCounts, rightDeltaCounts);
}

void MD25Driver::getAcceleration()
{
	buffer[0] = 14;

	if((write(fd, buffer, 1)) != 1)
	{	
        // Send regiter to read software from from
		throw std::runtime_error("Error writing to i2c slave\n");
		exit(1);
	}
	
	if (read(fd, buffer, 1) != 1)
	{   
        // Read back data into buf[]
		throw std::runtime_error("Unable to read from slave\n");
		exit(1);
	}
	else
	{
		acceleration = buffer[0];
	}
}

int MD25Driver::readAcceleration()
{
	return acceleration;
}

void MD25Driver::setAcceleration(int new_acceleration)
{
	buffer[0] = 14;													
	buffer[1] = new_acceleration;	
    
    // A speed of 128 stops the motor
	if ((write(fd, buffer, 2)) != 2)
	{
		throw std::runtime_error("Error writing to i2c slave\n");
		exit(1);
	}

	acceleration = new_acceleration;
}

void MD25Driver::readEncoders()
{
	unsigned char tx_encoders[1]= {0x02};
	uint8_t payload[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

	if((write(fd, tx_encoders, 1)) != 1)
	{
		throw std::runtime_error("Error writing to i2c slave\n");
		exit(1);
	}

	usleep(2000);

	if(read(fd, payload, 8) != 8)
	{	
        // Read back 8 bytes for the encoder values into buffer[]
		throw std::runtime_error("Unable to read from slave\n");
		exit(1);
	}
	else
	{
		// printf("[%d] [%d] [%d] [%d] [%d] [%d] [%d] [%d]\n",payload[0],payload[1],payload[2],payload[3],payload[4],payload[5],payload[6],payload[7]);
		rightEncoder = (payload[0] <<24) + (payload[1] << 16) + (payload[2] << 8) + payload[3];
		leftEncoder = (payload[4] <<24) + (payload[5] << 16) + (payload[6] << 8) + payload[7];
	}

}

void MD25Driver::resetEncoders() 
{
	std::cout << "Reseting Encoders" << std::endl;

	buffer[0] = 16;		// Command register
	buffer[1] = 32;		// command to set decoders back to zero
	
	if((write(fd, buffer, 2)) != 2) 
	{
		throw std::runtime_error("Error writing to i2c slave\n");
		exit(1);
	}
	
	leftEncoder = 0;
	rightEncoder = 0;
    leftDeltaCounts = 0;
    rightDeltaCounts = 0;
    lastLeftEncoder = 0;
    lastRightEncoder = 0;
    lastLeftDeltaCounts = 0;
    lastRightDeltaCounts = 0;
	leftVelRPM = 0;
	rightVelRPM = 0;
}

void MD25Driver::stopMotors()
{
	buffer[0] = 0;													
	buffer[1] = 128;	
    
    // A speed of 128 stops the motor
	if ((write(fd, buffer, 2)) != 2)
	{
		throw std::runtime_error("Error writing to i2c slave\n");
		exit(1);
	}

	buffer[0] = 1;													
	buffer[1] = 128;

	if ((write(fd, buffer, 2)) != 2)
	{
		throw std::runtime_error("Error writing to i2c slave\n");
		exit(1);
	}

	resetEncoders();
}

int MD25Driver::getBatteryVoltage()
{
	int battery_voltage = 0;

	buffer[0]=10;

	if((write(fd, buffer, 1)) != 1)
	{	
        // Send regiter to read software from from
		throw std::runtime_error("Error writing to i2c slave\n");
		exit(1);
	}
	
	if (read(fd, buffer, 1) != 1)
	{	
        // Read back data into buf[]
		throw std::runtime_error("Unable to read from slave\n");
		exit(1);
	}
	else
	{
		battery_voltage = buffer[0];
	}

	return battery_voltage;
}

std::tuple<int,int> MD25Driver::getCurrents()
{
	int current_l = 0, current_r = 0; 

	buffer[0] = 11;

	if((write(fd, buffer, 1)) != 1)
	{	
        // Send regiter to read software from from
		throw std::runtime_error("Error writing to i2c slave\n");
		exit(1);
	}
	
	if(read(fd, buffer, 1) != 1)
	{	
        // Read back data into buf[]
		throw std::runtime_error("Unable to read from slave\n");
		exit(1);
	}
	else
	{
		current_l = buffer[0];
	}

	buffer[0] = 12;

	if((write(fd, buffer, 1)) != 1)
	{	
        // Send regiter to read software from from
		throw std::runtime_error("Error writing to i2c slave\n");
		exit(1);
	}
	
	if (read(fd, buffer, 1) != 1)
	{	
        // Read back data into buf[]
		throw std::runtime_error("Unable to read from slave\n");
		exit(1);
	}
	else
	{
		current_r = buffer[0];
	}

	return std::make_tuple(current_l, current_r);
}

void MD25Driver::enableRegulation() 
{
	buffer[0] = 16;		// Command register
	buffer[1] = 49;	    // command to set decoders back to zero
	
	if((write(fd, buffer, 2)) != 2) 
	{
		throw std::runtime_error("Error writing to i2c slave\n");
		exit(1);
	}
	
	regulator=true;
}

void MD25Driver::disableRegulation() 
{
	buffer[0] = 16;		// Command register
	buffer[1] = 48;		// command to set decoders back to zero
	
	if((write(fd, buffer, 2)) != 2) 
	{
		throw std::runtime_error("Error writing to i2c slave\n");
		exit(1);
	}
	
	regulator=false;
}

int MD25Driver::readRegulation()
{
	return regulator;
}

void MD25Driver::enableTimeout() 
{
	buffer[0] = 16;		// Command register
	buffer[1] = 51;		// command to set decoders back to zero
	
	if((write(fd, buffer, 2)) != 2) 
	{
		throw std::runtime_error("Error writing to i2c slave\n");
		exit(1);
	}
	
	timeout=true;
}

void MD25Driver::disableTimeout() 
{
	buffer[0] = 16;	    // Command register
	buffer[1] = 50;	    // command to set decoders back to zero
	
	if((write(fd, buffer, 2)) != 2) 
	{
		throw std::runtime_error("Error writing to i2c slave\n");
		exit(1);
	}
	
	timeout = false;
}

int MD25Driver::readTimeout()
{
	return timeout;
}

void MD25Driver::setLeftMotorVelocity(float leftVelocity)
{
	buffer[0] = 1;													
	buffer[1] = voltageToInt(leftVelocity);	

	if ((write(fd, buffer, 2)) != 2) {
		throw std::runtime_error("Error writing to i2c slave\n");
		exit(1);
	}
}
void MD25Driver::setRightMotorVelocity(float rightVelocity)
{
	buffer[0] = 0;													
	buffer[1] = voltageToInt(rightVelocity);												
	
	if ((write(fd, buffer, 2)) != 2) {
		throw std::runtime_error("Error writing to i2c slave\n");
		exit(1);
	}

}

uint8_t MD25Driver::voltageToInt(float valueVolts)
{
	/*
	Given a value in volts that we want to output,
	we must map from our driver (MD25) voltage range (-12, +12)
	to our i2c command range (0, 255)
	*/
	if(valueVolts >12.0)
	{
		valueVolts=12.0;
	}
	else if(valueVolts<-12.0)
	{
		valueVolts=-12.0;
	}
	return int(std::round(valueVolts*(10.625)+ 127.5));
}

void MD25Driver::countsToRPM()
{
    leftVelRPM=((leftDeltaCounts)/(cpr))*(1.0/motorRatio)*(60.0/deltaTime);
    rightVelRPM=((rightDeltaCounts)/(cpr))*(1.0/motorRatio)*(60.0/deltaTime);
}

void MD25Driver::filterDelta()
{
  	if (abs(leftDeltaCounts) > 75 ) 
  	{
    	leftDeltaCounts = lastLeftDeltaCounts;
  	}
  	if (abs(rightDeltaCounts) > 75 ) 
  	{
    	rightDeltaCounts = lastRightDeltaCounts;
  	}

    lastLeftDeltaCounts = leftDeltaCounts;
    lastRightDeltaCounts  = rightDeltaCounts;
}
