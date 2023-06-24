
#include <Wire.h>

#include "BQ2562x.hpp"

#define BQ72562x_I2C_TIMEOUT 2000

void BQ2562x::Initialize()
{

}

// Read a specified number of bytes over I2C at a given subAddress
int16_t BQ2562x::i2cReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	int16_t timeout = BQ72562x_I2C_TIMEOUT;	
	Wire.beginTransmission(_deviceAddress);
	Wire.write(subAddress);
	Wire.endTransmission(true);
	
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