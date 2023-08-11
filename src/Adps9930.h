/*-------------------------------------------------------------------------
ADPS library

Written by Michael C. Miller.

I invest time and resources providing this open source code,
please support me by dontating (see https://github.com/Makuna/Rtc)

-------------------------------------------------------------------------
This file is part of the Makuna/ADPS library.

Rtc is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of
the License, or (at your option) any later version.

Rtc is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with Rtc.  If not, see
<http://www.gnu.org/licenses/>.
-------------------------------------------------------------------------*/

#pragma once

#include <Arduino.h>
#include "AdpsUtil.h"
#include "WireUtil.h"
#include "Adps9930_types.h"

namespace ADPS9930
{

template<class T_WIRE_METHOD> class Adps9930
{
public:
    Adps9930(T_WIRE_METHOD& wire) :
        _wire(wire),
        _lastError(WIRE_UTIL::Error_None)
    {
    }

    void Begin()
    {
        _wire.begin();
        initToRecommendedConfig();
    }

    void Begin(int sda, int scl)
    {
        _wire.begin(sda, scl);
        initToRecommendedConfig();
    }

    uint8_t LastError()
    {
        return _lastError;
    }

    void Start(Feature feature = Feature_Proximity_Als,
            bool intEnable = false, 
            bool sleepAfterInt = false)
    {
        uint8_t value = _BV(ENABLE_WEN) | _BV(ENABLE_PO);

        if (feature & Feature_Proximity)
        {
            value |= _BV(ENABLE_PEN);
            if (intEnable)
            {
                value |= _BV(ENABLE_PIEN);
            }
        }
        if (feature & Feature_AmbiantLightSensor)
        {
            value |= _BV(ENABLE_AEN);
            if (intEnable)
            {
                value |= _BV(ENABLE_AIEN);
            }
        }

        if (sleepAfterInt)
        {
            value |= _BV(ENABLE_SAI);
        }
               
        setReg(REG_ENABLE, value);
    }

    void Stop()
    {
        // disable and power down
        setReg(REG_ENABLE, 0);
    }

    void LatchInterrupt(Feature feature)
    {
        uint8_t command = CMD_TRANSACTION_SPECIAL;
        if (feature & Feature_Proximity)
        {
            command |= CMD_SPECIAL_PROXIMITY_INT_CLEAR;
        }
        if (feature & Feature_AmbiantLightSensor)
        {
            command |= CMD_SPECIAL_ALS_INT_CLEAR;
        }
        setReg(command, 0x00);
    }

    void SetAlsAdcTime(float msAlsAdcTime)
    {
        uint8_t value = msToTimeReg(msAlsAdcTime);
            
        setReg(REG_ATIME, value);
    }

    void SetProximityAdcTime(float msProxityAdcTime)
    {
        uint8_t value = msToTimeReg(msProxityAdcTime);
        setReg(REG_PTIME, value);
    }

    void SetWaitTime(float msWaitTime)
    {
        constexpr float LONG_WAIT_MULTIPLIER = 12.0f;
        constexpr float MIN_LONGWAIT_MS = MIN_TIME_ADC_MS * LONG_WAIT_MULTIPLIER;

        float value;
        uint8_t config = getReg(REG_CONFIG);
        if (_lastError == WIRE_UTIL::Error_None)
        {
            if (msWaitTime >= MIN_LONGWAIT_MS)
            {
                // using the long wait ranges
                config |= CONFIG_WLONG;

                float msNormalized = msWaitTime / LONG_WAIT_MULTIPLIER;
                value = msToTimeReg(msNormalized);
            }
            else
            {
                // using the normal wait ranges
                config &= ~CONFIG_WLONG;
                value = msToTimeReg(msWaitTime);
            }

            setReg(REG_CONFIG, config);
            if (_lastError == WIRE_UTIL::Error_None)
            {
                setReg(REG_WTIME, value);
            }
        }
    }

    void SetAlsIntThresholds(uint16_t lowCh0Value, uint16_t highCh0Value)
    {
        _wire.beginTransmission(I2C_ADDRESS);
        _wire.write(CMD_TRANSACTION_AUTO_INC | REG_ALS_INT_THRESHOLDS);
        _wire.write(lowCh0Value & 0xff);
        _wire.write(lowCh0Value >> 8);
        _wire.write(highCh0Value & 0xff);
        _wire.write(highCh0Value >> 8);
        _lastError = _wire.endTransmission();
    }

    void SetProximityIntThresholds(uint16_t lowValue, uint16_t highValue)
    {
        _wire.beginTransmission(I2C_ADDRESS);
        _wire.write(CMD_TRANSACTION_AUTO_INC | REG_PROXIMITY_INT_THRESHOLDS);
        _wire.write(lowValue & 0xff);
        _wire.write(lowValue >> 8);
        _wire.write(highValue & 0xff);
        _wire.write(highValue >> 8);
        _lastError = _wire.endTransmission();
    }

    void SetThresholdPersistenceFilterCounts(
            uint8_t alsCh0FilterCount,
            uint8_t proximityFilterCount )
    {
        const uint8_t MAX_ALSCH0 = 60;
        const uint8_t MAX_PROXIMITY = 15;

            
        if (alsCh0FilterCount > MAX_ALSCH0)
        {
            alsCh0FilterCount = MAX_ALSCH0;
        }
        if (proximityFilterCount > MAX_PROXIMITY)
        {
            proximityFilterCount = MAX_PROXIMITY;
        }

        // encode proximityFilterCount
        uint16_t alsValue = alsCh0FilterCount;

        // two levels of encoding
        if (alsValue >= 5)
        {
            // map(alsValue, 5, 60, 4, 15)
            // return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
            alsValue = ((alsValue - 5) * 11) / 55 + 4;
        }

        setReg(REG_PERSISTENCE, alsValue | (proximityFilterCount << 4));
    }

    void SetProximityPulseCount(uint8_t count)
    {
        setReg(REG_PPULSE, count);
    }

    void SetAnalogControl(
        LedDriveCurrent ledDriveCurrent,
        ProximityGain proximityGain,
        AlsGain alsGain)
    {
        uint8_t value;
        uint8_t config = getReg(REG_CONFIG);
        uint8_t ldc = ledDriveCurrent;
        uint8_t ag = alsGain;

        if (ldc >= LedDriveCurrent_11mA)
        {
            config |= _BV(CONFIG_PDL);
            ldc &= 0x03;
        }

        if (ag >= AlsGain_1_6x)
        {
            config |= _BV(CONFIG_AGL);
            ag &= 0x03;
        }

        value = ldc << 6 |
            CONTROL_PDIODE_CH1 |
            proximityGain << 2 |
            ag;

        setReg(REG_CONFIG, config);
        if (_lastError == WIRE_UTIL::Error_None)
        {
            setReg(REG_CONTROL, value);
        }
    }
          
    uint8_t GetId()
    {
        return getReg(REG_ID);
    }

    bool IsIdValid(uint8_t id)
    {
        // ID Register values
        const uint8_t IDS_APDS_9930[] = { 0x30, 0x39 }; // spec'ed 0x39, observed 0x30
        const uint8_t IDS_APDS_9930_COUNT = sizeof(IDS_APDS_9930) / sizeof(IDS_APDS_9930[0]);

        for (uint8_t index = 0; index < IDS_APDS_9930_COUNT; index++)
        {
            if (IDS_APDS_9930[index] == id)
            {
                return true;
            }
        }
        return false;
    }

    Status GetStatus()
    {
        return Status(getReg(REG_STATUS));
    }

    AlsData GetAlsData()
    {
        _wire.beginTransmission(I2C_ADDRESS);
        _wire.write(CMD_TRANSACTION_AUTO_INC | REG_ALS_DATA);
        _lastError = _wire.endTransmission();
        if (_lastError != WIRE_UTIL::Error_None)
        {
            return AlsData();
        }

        uint8_t countRead = this->_wire.requestFrom(I2C_ADDRESS, REG_ALS_DATA_SIZE);
        if (_lastError != WIRE_UTIL::Error_None ||
            countRead != REG_ALS_DATA_SIZE)
        {
            return AlsData();
        }

        uint16_t ch0;

        ch0 = _wire.read();
        ch0 += _wire.read() << 8;

        uint16_t ch1;

        ch1 = _wire.read();
        ch1 += _wire.read() << 8;

        return AlsData(ch0, ch1);
    }

    uint16_t GetProximityData()
    {
        return getWord(REG_PROXIMITY_DATA);
    }

    void SetProximityOffset(int8_t offset)
    {
        setReg(REG_PROXIMITY_OFFSET, offset);
    }

    int8_t GetProximityOffset()
    {
        return getReg(REG_PROXIMITY_OFFSET);
    }


protected:
    T_WIRE_METHOD& _wire;
    uint8_t _lastError;

    // I2C Slave Address  
    const uint8_t I2C_ADDRESS = 0x39;

    // Register Addresses
    static constexpr uint8_t REG_ENABLE = 0x00;
    static constexpr uint8_t REG_ATIME = 0x01;
    static constexpr uint8_t REG_PTIME = 0x02;
    static constexpr uint8_t REG_WTIME = 0x03;
    static constexpr uint8_t REG_ALS_INT_THRESHOLDS = 0x04;
    static constexpr uint8_t REG_PROXIMITY_INT_THRESHOLDS = 0x08;
    static constexpr uint8_t REG_PERSISTENCE = 0x0C;
    static constexpr uint8_t REG_CONFIG = 0x0D;
    static constexpr uint8_t REG_PPULSE = 0x0E;
    static constexpr uint8_t REG_CONTROL = 0x0F;
    static constexpr uint8_t REG_ID = 0x12;
    static constexpr uint8_t REG_STATUS = 0x13;
    static constexpr uint8_t REG_ALS_DATA = 0x14;
    static constexpr uint8_t REG_PROXIMITY_DATA = 0x18;
    static constexpr uint8_t REG_PROXIMITY_OFFSET = 0x1E;

    //Register Data Size if not just 1
    static constexpr size_t REG_ALS_DATA_SIZE = 4;
 
    // Command Register Flags
    static constexpr uint8_t CMD_TRANSACTION_REPEATED = 0x80;
    static constexpr uint8_t CMD_TRANSACTION_AUTO_INC = 0xA0;
    static constexpr uint8_t CMD_TRANSACTION_SPECIAL = 0xE0;

    // CMD_TRANSACTION_SPECIAL flags
    static constexpr uint8_t CMD_SPECIAL_NONE = 0x00;
    static constexpr uint8_t CMD_SPECIAL_ALS_INT_CLEAR = 0x06;
    static constexpr uint8_t CMD_SPECIAL_PROXIMITY_INT_CLEAR = 0x05;

    // ENABLE Register Bits
    static constexpr uint8_t ENABLE_SAI = 6;
    static constexpr uint8_t ENABLE_PIEN = 5;
    static constexpr uint8_t ENABLE_AIEN = 4;
    static constexpr uint8_t ENABLE_WEN = 3;
    static constexpr uint8_t ENABLE_PEN = 2;
    static constexpr uint8_t ENABLE_AEN = 1;
    static constexpr uint8_t ENABLE_PO = 0;

    // PERSISTENCE Register MASKS
    static constexpr uint8_t PERSISTENCE_PPERS_MASK = 0xF0;
    static constexpr uint8_t PERSISTENCE_APERS_MASK = 0x0F;

    // CONFIG Register Bits
    static constexpr uint8_t CONFIG_AGL = 2;
    static constexpr uint8_t CONFIG_WLONG = 1;
    static constexpr uint8_t CONFIG_PDL = 0;

    // CONTROL Register flags
    static constexpr uint8_t CONTROL_PDIODE_CH1 = 0x20;

    static constexpr float MAX_TIME_ADC_MS = 699.0f;
    static constexpr float MIN_TIME_ADC_MS = MS_ADC_TIME_QUOTUM;
    static constexpr float CONV_TIME_ADC_RATIO = 1.0f / MS_ADC_TIME_QUOTUM;

    uint8_t getReg(uint8_t regAddress)
    {
        _wire.beginTransmission(I2C_ADDRESS);
        _wire.write(CMD_TRANSACTION_REPEATED | regAddress);
        _lastError = _wire.endTransmission();
        if (_lastError != WIRE_UTIL::Error_None)
        {
            return 0;
        }

        // control register
        size_t bytesRead = _wire.requestFrom(I2C_ADDRESS, (uint8_t)1);
        if (1 != bytesRead)
        {
            _lastError = WIRE_UTIL::Error_Unspecific;
            return 0;
        }

        uint8_t regValue = _wire.read();
        return regValue;
    }

    void setReg(uint8_t regAddress, uint8_t regValue)
    {
        _wire.beginTransmission(I2C_ADDRESS);
        _wire.write(CMD_TRANSACTION_REPEATED | regAddress);
        _wire.write(regValue);
        _lastError = _wire.endTransmission();
    }

    uint16_t getWord(uint8_t regAddress)
    {
        _wire.beginTransmission(I2C_ADDRESS);
        _wire.write(CMD_TRANSACTION_AUTO_INC | regAddress);
        _lastError = _wire.endTransmission();
        if (_lastError != WIRE_UTIL::Error_None)
        {
            return 0;
        }

        // control register
        size_t bytesRead = _wire.requestFrom(I2C_ADDRESS, (uint8_t)2);
        if (2 != bytesRead)
        {
            _lastError = WIRE_UTIL::Error_Unspecific;
            return 0;
        }

        uint16_t result;

        result = _wire.read();
        result += _wire.read() * 256;

        return result;
    }

    uint8_t msToTimeReg(float msTime)
    {
        if (msTime > MAX_TIME_ADC_MS)
        {
            msTime = MAX_TIME_ADC_MS;
        }
        if (msTime < MIN_TIME_ADC_MS)
        {
            msTime = MIN_TIME_ADC_MS;
        }

        uint8_t value = static_cast<uint8_t>(256.0f - (msTime * CONV_TIME_ADC_RATIO));
        return value;
    }

    float timeRegToMs(uint8_t time)
    {
        float value = MS_ADC_TIME_QUOTUM * (256 - time);
        return value;
    }

    void initToRecommendedConfig()
    {
        // disable and power down first
        setReg(REG_ENABLE, 0);
        // ALS ADC TIME = 27.3ms
        setReg(REG_ATIME, ALS_ADC_TIME_DEFAULT);
        // PROXIMITY ADC TIME = 2.73ms (minimum) 
        setReg(REG_PTIME, 0xff);
        // WAIT TIME = 2.73ms (minimum)
        setReg(REG_WTIME, 0xff);
        // Proximity Pulse Count = 8
        setReg(REG_PPULSE, 8);

        SetAnalogControl(LedDriveCurrent_Default,
            ProximityGain_Default,
            AlsGain_Default);
    }
};

} // namespace Adps9930