#include <LC709204F.h>

namespace PowerFeather
{
    bool LC709204F::readReg(uint8_t command, uint16_t& data)
    {
        uint8_t reply[6];
        reply[0] = _i2cAddress * 2; // write byte
        reply[1] = command;                       // command / register
        reply[2] = reply[0] | 0x1;                // read byte

        if (!_i2c.read(_i2cAddress, command, reply + 3, 3)) {
            return false;
        }

        uint8_t crc = computeCRC8(reply, 5);
        // CRC failure?
        if (crc != reply[5])
            return false;

        data = reply[4];
        data <<= 8;
        data |= reply[3];

        return true;
    }

    bool LC709204F::writeReg(uint8_t command, uint16_t data)
    {
        uint8_t send[5];
        send[0] = _i2cAddress * 2;
        send[1] = command;
        send[2] = data & 0xFF;
        send[3] = data >> 8;
        send[4] = computeCRC8(send, 4);

        return _i2c.write(_i2cAddress, command, send + 2, 4);
    }

    uint8_t LC709204F::computeCRC8(uint8_t *data, int len)
    {
        const uint8_t POLYNOMIAL(0x07);
        uint8_t crc(0x00);

        for (int j = len; j; --j)
        {
            crc ^= *data++;

            for (int i = 8; i; --i)
            {
                crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
            }
        }
        return crc;
    }

    uint16_t LC709204F::getCellVoltage(void)
    {
        uint16_t val = 0;
        readReg(Registers::Cell_Voltage, val);
        return val;
    }

    uint16_t LC709204F::getRSOC(void)
    {
        uint16_t val = 0;
        readReg(Registers::RSOC, val);
        return val;
    }

    uint16_t LC709204F::getSOH(void)
    {
        uint16_t val = 0;
        readReg(Registers::State_Of_Health, val);
        return val;
    }

    uint16_t LC709204F::getTimeToEmpty(void)
    {
        uint16_t val = 0;
        readReg(Registers::TimeToEmpty, val);
        return val;
    }

    uint16_t LC709204F::getTimeToFull(void)
    {
        uint16_t val = 0;
        readReg(Registers::TimeToFull, val);
        return val;
    }
}