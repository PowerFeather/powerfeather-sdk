#include <cstdint>


class BQ2562x
{
public:
    void Initialize();

private:
	uint8_t _deviceAddress;

    int16_t i2cReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);
    uint16_t i2cWriteBytes(uint8_t subAddress, uint8_t * src, uint8_t count);
};