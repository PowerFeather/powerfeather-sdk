#include <LC709204F.h>

namespace PowerFeather
{
    bool LC709204F::readReg(Registers reg, uint16_t& data)
    {
        uint8_t reply[6];
        reply[0] = _i2cAddress << 1; // write byte
        reply[1] = static_cast<uint8_t>(reg);     // register
        reply[2] = reply[0] | 0x1;                // read byte

        if (!_i2c.read(_i2cAddress, reply[0], reply + 3, 3)) {
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

    bool LC709204F::writeReg(Registers reg, uint16_t data)
    {
        uint8_t send[5];
        send[0] = _i2cAddress << 1;
        send[1] = static_cast<uint8_t>(reg);
        send[2] = data & 0xFF;
        send[3] = data >> 8;
        send[4] = computeCRC8(send, 4);

        return _i2c.write(_i2cAddress, send[1], send + 2, 4);
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

    bool LC709204F::getCellVoltage(uint16_t& mV)
    {
        return readReg(Registers::Cell_Voltage, mV);
    }

    bool LC709204F::getRSOC(uint8_t& rsoc)
    {
        uint16_t val = 0;
        if (readReg(Registers::RSOC, val))
        {
            rsoc = val;
            return true;
        }
        return false;
    }

    bool LC709204F::getSOH(uint8_t& soh)
    {
        uint16_t val = 0;
        if (readReg(Registers::State_Of_Health, val))
        {
            soh = val;
            return true;
        }
        return false;
    }

    bool LC709204F::getTimeToEmpty(uint16_t& minutes)
    {
        return readReg(Registers::TimeToEmpty, minutes);
    }

    bool LC709204F::getTimeToFull(uint16_t& minutes)
    {
        return readReg(Registers::TimeToFull, minutes);
    }

    bool LC709204F::setAPA(uint16_t mAh)
    {
        if (mAh == 500)
        {
            return writeReg(Registers::APA, 0x2121);
        }
        return false;
    }

    bool LC709204F::setChangeOfParameter(ChangeOfParameter param)
    {
        return writeReg(Registers::Change_Of_The_Parameter, static_cast<uint16_t>(param));
    }

    bool LC709204F::enableTSENSE(bool tsense1, bool tsense2)
    {
        uint16_t status = tsense1 << 0 | tsense2 << 1;
        return writeReg(Registers::Status_Bit, status);
    }

    bool LC709204F::enableOperation(bool enable)
    {
        return writeReg(Registers::IC_Power_Mode, enable ? 0x0001 : 0x0002);
    }
}