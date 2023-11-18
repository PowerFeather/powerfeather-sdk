
#include <Wire.h>

#include "MasterI2C.h"

namespace PowerFeather
{
    class ArduinoMasterI2C : public MasterI2C
    {
    public:
        bool init(uint8_t port, uint8_t sdaPin, uint8_t sclPin, uint32_t freq) override;
        bool write(uint8_t address, uint8_t reg, const uint8_t *buf, size_t len) override;
        bool read(uint8_t address, uint8_t reg, uint8_t *buf, size_t len) override;
    private:
        bool read(uint8_t address, uint8_t *buffer, size_t len);
        TwoWire *_wire;
    };
}
