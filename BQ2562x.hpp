#include <cstdint>


#define PART_INFORMATION_REGISTER 0x38

class BQ2562x
{
public:
    BQ2562x();

    int16_t i2cReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);
    uint16_t i2cWriteBytes(uint8_t subAddress, uint8_t * src, uint8_t count);

private:
	uint8_t _deviceAddress;
};