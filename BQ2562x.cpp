
#include <Wire.h>

#include "BQ2562x.hpp"

#define BQ72562x_I2C_TIMEOUT 2000
#define BQ2562x_I2C_ADDRESS 0x6a

BQ2562x::BQ2562x()
{
	abort();
	_deviceAddress = BQ2562x_I2C_ADDRESS;
	Wire.begin(48, 47);
}

// Read a specified number of bytes over I2C at a given subAddress
int16_t BQ2562x::i2cReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	int16_t timeout = BQ72562x_I2C_TIMEOUT;	
	Wire.beginTransmission(_deviceAddress);
	Wire.write(subAddress);
	int error = Wire.endTransmission(true);
	
	Wire.requestFrom(_deviceAddress, count);
	
	for (int i=0; i<count; i++)
	{
		dest[i] = Wire.read();
	}
	
	return timeout;
}

// Write a specified number of bytes over I2C to a given subAddress
uint16_t BQ2562x::i2cWriteBytes(uint8_t subAddress, uint8_t * src, uint8_t count)
{
	Wire.beginTransmission(_deviceAddress);
	Wire.write(subAddress);
	for (int i=0; i<count; i++)
	{
		Wire.write(src[i]);
	}	
	Wire.endTransmission(true);
	
	return true;	
}