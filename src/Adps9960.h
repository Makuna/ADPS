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
#include "Adps9960_types.h"
#include "Adps9960_GestureEngine.h"

namespace ADPS9960
{

template<class T_WIRE_METHOD> class Adps9960
{
public:
    Adps9960(T_WIRE_METHOD& wire) :
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

    void Start(Feature feature = Feature_Gesture_Proximity_Als,
            Feature intEnable = Feature_None,
            bool sleepAfterInt = false)
    {
        // disable and power down first
        setReg(REG_ENABLE, 0);

        if (_lastError == WIRE_UTIL::Error_None)
        {
            uint8_t value = _BV(ENABLE_PO) | _BV(ENABLE_WEN);

            if (feature & Feature_Proximity)
            {
                value |= _BV(ENABLE_PEN);
                if (intEnable & Feature_Proximity)
                {
                    value |= _BV(ENABLE_PIEN);
                }
            }
            if (feature & Feature_AmbiantLightSensor)
            {
                value |= _BV(ENABLE_AEN);
                if (intEnable & Feature_AmbiantLightSensor)
                {
                    value |= _BV(ENABLE_AIEN);
                }
            }
            if (feature & Feature_Gesture)
            {
                value |= _BV(ENABLE_GEN) | _BV(ENABLE_PEN); // proximity must also be enabled

                uint8_t gconfig4 = getReg(REG_GESTURE_CONFIG4);
                if (_lastError == WIRE_UTIL::Error_None)
                {
                    if (intEnable & Feature_Gesture)
                    {
                        gconfig4 |= _BV(GESTURE_CONFIG4_GIEN);
                    }
                    else
                    {
                        gconfig4 &= ~_BV(GESTURE_CONFIG4_GIEN);
                    }
                    setReg(REG_GESTURE_CONFIG4, gconfig4);
                }
            }

#ifdef ADPS_DEBUG
            if (feature & Feature_Gesture)
            {
                printGestureRegisters(value);
            }
#endif
            setReg(REG_ENABLE, value);

            if (_lastError == WIRE_UTIL::Error_None)
            {
                uint8_t value = getReg(REG_CONFIG3);
                if (_lastError == WIRE_UTIL::Error_None)
                {
                    if (sleepAfterInt)
                    {
                        value |= _BV(CONFIG3_SAI);
                    }
                    else
                    {
                        value &= ~_BV(CONFIG3_SAI);
                    }
                    setReg(REG_CONFIG3, value);
                }
            }
        }
    }

    void Stop()
    {
        // disable and power down
        setReg(REG_ENABLE, 0);
    }

    // Saturation Int are cleared with general Feature
    void LatchInterrupt(Feature feature)
    {
        if (feature & Feature_Gesture)
        {
            uint8_t gconfig4 = getReg(REG_GESTURE_CONFIG4);
            if (_lastError == WIRE_UTIL::Error_None)
            {
                gconfig4 |= _BV(GESTURE_CONFIG4_GFIFO_CLEAR);

                setReg(REG_GESTURE_CONFIG4, gconfig4);
            }
        }
        if (feature & Feature_Proximity_Als)
        {
            uint8_t command = REG_AICLEAR;
            if (feature & Feature_Proximity && !(feature & Feature_AmbiantLightSensor))
            {
                command |= REG_PICLEAR;
            }
            else if (feature & Feature_AmbiantLightSensor && !(feature & Feature_Proximity))
            {
                command |= REG_CICLEAR;
            }

            setReg(command, 0x00);
        }
    }

    void SetAlsAdcTime(float msAlsAdcTime)
    {
        uint8_t value = msToTimeReg(msAlsAdcTime);
            
        setReg(REG_ATIME, value);
    }

    void SetWaitTime(float msWaitTime)
    {
        constexpr float LONG_WAIT_MULTIPLIER = 12.0f;
        constexpr float MIN_LONGWAIT_MS = MIN_TIME_ADC_MS * LONG_WAIT_MULTIPLIER;

        float value;
        uint8_t config = getReg(REG_CONFIG1);
        if (_lastError == WIRE_UTIL::Error_None)
        {
            if (msWaitTime >= MIN_LONGWAIT_MS)
            {
                // using the long wait ranges
                config |= CONFIG1_WLONG;

                float msNormalized = msWaitTime / LONG_WAIT_MULTIPLIER;
                value = msToTimeReg(msNormalized);
            }
            else
            {
                // using the normal wait ranges
                config &= ~CONFIG1_WLONG;
                value = msToTimeReg(msWaitTime);
            }

            setReg(REG_CONFIG1, config);
            if (_lastError == WIRE_UTIL::Error_None)
            {
                setReg(REG_WTIME, value);
            }
        }
    }

    void SetAlsIntThresholds(uint16_t lowValue, uint16_t highValue)
    {
        _wire.beginTransmission(I2C_ADDRESS);
        _wire.write(REG_ALS_INT_THRESHOLDS);
        _wire.write(lowValue & 0xff);
        _wire.write(lowValue >> 8);
        _wire.write(highValue & 0xff);
        _wire.write(highValue >> 8);
        _lastError = _wire.endTransmission();
    }

    void SetProximityIntThresholds(uint8_t lowValue, uint8_t highValue)
    {
        _wire.beginTransmission(I2C_ADDRESS);
        _wire.write(REG_PROXIMITY_INT_THRESHOLDS);
        _wire.write(lowValue);
        _wire.write(highValue);
            _lastError = _wire.endTransmission();
    }

    void SetThresholdPersistenceFilterCounts(
            uint8_t alsFilterCount,
            uint8_t proximityFilterCount )
    {
        const uint8_t MAX_ALS = 60;
        const uint8_t MAX_PROXIMITY = 15;

            
        if (alsFilterCount > MAX_ALS)
        {
            alsFilterCount = MAX_ALS;
        }
        if (proximityFilterCount > MAX_PROXIMITY)
        {
            proximityFilterCount = MAX_PROXIMITY;
        }

        // encode alsFilterCount
        uint16_t alsValue = alsFilterCount;

        // two levels of encoding
        if (alsFilterCount >= 5)
        {
            // map(alsFilterCount, 5, 60, 4, 15)
            // return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
            alsValue = ((alsValue - 5) * 11) / 55 + 4;
        }

        setReg(REG_PERSISTENCE, alsValue | (proximityFilterCount << 4));
    }


    void SetProximityPulseConfig(uint8_t count, ProximityPulseLength length = ProximityPulseLength_Default)
    {
        if (count > 64)
        {
            count = 64;
        }
        if (count > 0)
        {
            count--;
        }

        uint8_t value = count | (length << 6);
        setReg(REG_PPULSE, value);
    }

    void SetAnalogControl(
        LedDriveCurrent ledDriveCurrent,
        ProximityGain proximityGain,
        AlsGain alsGain)
    {
        uint8_t value;

        value = ledDriveCurrent << 6 |
            proximityGain << 2 |
            alsGain;
        setReg(REG_CONTROL, value);
        if (_lastError == WIRE_UTIL::Error_None)
        {
            value = getReg(REG_CONFIG2);
            if (_lastError == WIRE_UTIL::Error_None)
            {
                value &= ~CONFIG2_LEDBOOST_MASK;
                value |= (ledDriveCurrent & 0xf0);
                setReg(REG_CONFIG2, value);
            }
        }
    }
          
    void EnableSaturationInt(bool proximitySat, bool clearPhotodiodeSat)
    {
        uint8_t value = getReg(REG_CONFIG2);
        if (_lastError == WIRE_UTIL::Error_None)
        {
            value &= ~CONFIG2_SIEN_MASK;
            if (proximitySat)
            {
                value |= _BV(CONFIG2_PSIEN);
            }
            if (clearPhotodiodeSat)
            {
                value |= _BV(CONFIG2_CPSIEN);
            }
            setReg(REG_CONFIG2, value);
        }
    }

    uint8_t GetId()
    {
        return getReg(REG_ID);
    }

    bool IsIdValid(uint8_t id)
    {
        // ID Register values
        const uint8_t IDS_APDS_9960[] = { 0xAB, 0xA8 }; // spec'ed 0xAB, observed A8
        const uint8_t IDS_APDS_9960_COUNT = sizeof(IDS_APDS_9960) / sizeof(IDS_APDS_9960[0]);

        for (uint8_t index = 0; index < IDS_APDS_9960_COUNT; index++)
        {
            if (IDS_APDS_9960[index] == id)
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
        _wire.write(REG_RGBC_DATA);
        _lastError = _wire.endTransmission();
        if (_lastError != WIRE_UTIL::Error_None)
        {
            return AlsData();
        }

        uint8_t countRead = this->_wire.requestFrom(I2C_ADDRESS, REG_RGBC_DATA_SIZE);
        if (_lastError != WIRE_UTIL::Error_None ||
            countRead != REG_RGBC_DATA_SIZE)
        {
            return AlsData();
        }

        uint16_t clear;

        clear = _wire.read();
        clear += _wire.read() << 8;

        uint16_t red;

        red = _wire.read();
        red += _wire.read() << 8;

        uint16_t green;

        green = _wire.read();
        green += _wire.read() << 8;

        uint16_t blue;

        blue = _wire.read();
        blue += _wire.read() << 8;

        return AlsData(clear, red, green, blue);
    }


    uint8_t GetProximityData()
    {
        return getReg(REG_PROXIMITY_DATA);
    }

    void SetProximityOffset(int8_t offsetUpRight, int8_t offsetDownLeft )
    {
        _wire.beginTransmission(I2C_ADDRESS);
        _wire.write(REG_PROXIMITY_OFFSET);
        _wire.write(offsetUpRight);
        _wire.write(offsetDownLeft);
        _lastError = _wire.endTransmission();
    }

    void DisableProximityPhotoDiodes(uint8_t photoDiodeDisableFlags)
    {
        uint8_t value = getReg(REG_CONFIG3);
        if (_lastError == WIRE_UTIL::Error_None)
        {
            value &= ~CONFIG3_PBITS_MASK;
            value |= (photoDiodeDisableFlags & 0x0f);

            // if at least one is disabled from either paired photodiodes
            if ((photoDiodeDisableFlags & PhotoDiode_RL_Pair) &&
                (photoDiodeDisableFlags & PhotoDiode_DU_Pair))
            {
                bool bothRlDisabled = ((photoDiodeDisableFlags & PhotoDiode_RL_Pair) == PhotoDiode_RL_Pair);
                bool bothDuDisabled = ((photoDiodeDisableFlags & PhotoDiode_DU_Pair) == PhotoDiode_DU_Pair);
                if (!(bothRlDisabled && bothDuDisabled))
                {
                    value |= _BV(CONFIG3_PCMP);
                }
            }
            setReg(REG_CONFIG3, value);
        }
    }

    void SetGestureProximityThreshold(uint8_t enter = 30, uint8_t exit = 30)
    {
        enter &= ~_BV(4); // bit four must be set to 0
        _wire.beginTransmission(I2C_ADDRESS);
        _wire.write(REG_GESTURE_THRESHOLD);
        _wire.write(enter);
        _wire.write(exit);
        _lastError = _wire.endTransmission();
    }

    void SetGestureConfig(GestureFifoThreshold fifoThresholdInt = GestureFifoThreshold_Default,
            PhotoDiode photoDiodeExcludeExitMask = PhotoDiode_None,
            GestureExitPresistence exitPresistence = GestureExitPresistence_Default,
            LedDriveCurrent ledDriveCurrent = LedDriveCurrent_GestureDefault,
            GestureGain gain = GestureGain_Default,
            GestureWaitTime waitTime = GestureWaitTime_Default)
    {
        uint8_t gconfig1 = (fifoThresholdInt << 6) |
            ((photoDiodeExcludeExitMask & 0x0f) << 2) |
            (exitPresistence & 0x03);
        uint8_t gconfig2 = (gain << 5) |
            ((ledDriveCurrent & 0x0f) << 3) |
            (waitTime & 0x07);

        _wire.beginTransmission(I2C_ADDRESS);
        _wire.write(REG_GESTURE_CONFIG);
        _wire.write(gconfig1);
        _wire.write(gconfig2);
        _lastError = _wire.endTransmission();

        if (_lastError != WIRE_UTIL::Error_None)
        {
            return;
        }

        uint8_t config2 = getReg(REG_CONFIG2);
        if (_lastError != WIRE_UTIL::Error_None)
        {
            return;
        }

        config2 &= ~CONFIG2_LEDBOOST_MASK;
        config2 |= (ledDriveCurrent & 0xf0);
        setReg(REG_CONFIG2, config2);
        if (_lastError != WIRE_UTIL::Error_None)
        {
            return;
        }
    }

    void SetGestureOffset(int8_t offsetUp, 
            int8_t offsetDown,
            int8_t offsetLeft, 
            int8_t offsetRight)
    {
        setReg(REG_GESTURE_OFFSET_UP, offsetUp);
        if (_lastError != WIRE_UTIL::Error_None)
        {
            return;
        }
        setReg(REG_GESTURE_OFFSET_DOWN, offsetDown);
        if (_lastError != WIRE_UTIL::Error_None)
        {
            return;
        }
        setReg(REG_GESTURE_OFFSET_LEFT, offsetLeft);
        if (_lastError != WIRE_UTIL::Error_None)
        {
            return;
        }
        setReg(REG_GESTURE_OFFSET_RIGHT, offsetRight);
        if (_lastError != WIRE_UTIL::Error_None)
        {
            return;
        }
    }

    void SetGesturePulseConfig(uint8_t count = 8, ProximityPulseLength length = ProximityPulseLength_8us)
    {
        if (count > 64)
        {
            count = 64;
        }
        if (count > 0)
        {
            count--;
        }

        uint8_t value = count | (length << 6);
        setReg(REG_GESTURE_PULSE, value);
    }

    uint8_t GetGestureFifoCount()
    {
        return getReg(REG_GESTURE_FIFO_COUNT);
    }

    GestureStatus GetGestureStatus()
    {
        GestureStatus result;

        uint8_t status = getReg(REG_GESTURE_STATUS);
        if (_lastError == WIRE_UTIL::Error_None)
        {
            result = status;
        }
        return result;
    }

    GestureData GetNextGestureData()
    {
        GestureData result;

        _wire.beginTransmission(I2C_ADDRESS);
        _wire.write(REG_GESTURE_DATA);
        _lastError = _wire.endTransmission();
        if (_lastError == WIRE_UTIL::Error_None)
        {
            size_t bytesRead = _wire.requestFrom(I2C_ADDRESS, REG_GESTURE_DATA_SIZE);
            if (REG_GESTURE_DATA_SIZE != bytesRead)
            {
                _lastError = WIRE_UTIL::Error_Unspecific;
            }
            else
            {
                uint8_t up = _wire.read();
                uint8_t down = _wire.read();
                uint8_t left = _wire.read();
                uint8_t right = _wire.read();

                result = GestureData(up, down, left, right);
            }
        }
        return result;
    }

protected:
    T_WIRE_METHOD& _wire;
    uint8_t _lastError;

    // I2C Slave Address  
    const uint8_t I2C_ADDRESS = 0x39;

    // Register Addresses
    static constexpr uint8_t REG_RAM_FIRST = 0x00;
    static constexpr uint8_t REG_RAM_LAST = 0x7F;
    static constexpr uint8_t REG_ENABLE = 0x80;
    static constexpr uint8_t REG_ATIME = 0x81;
    // static constexpr uint8_t REG_PTIME = 0x82;
    static constexpr uint8_t REG_WTIME = 0x83;
    static constexpr uint8_t REG_ALS_INT_THRESHOLDS = 0x84;
    static constexpr uint8_t REG_PROXIMITY_INT_THRESHOLDS = 0x89;
    static constexpr uint8_t REG_PERSISTENCE = 0x8C;
    static constexpr uint8_t REG_CONFIG1 = 0x8D;
    static constexpr uint8_t REG_PPULSE = 0x8E;
    static constexpr uint8_t REG_CONTROL = 0x8F;
    static constexpr uint8_t REG_CONFIG2 = 0x90;
    static constexpr uint8_t REG_ID = 0x92;
    static constexpr uint8_t REG_STATUS = 0x93;
    static constexpr uint8_t REG_RGBC_DATA = 0x94;
    static constexpr uint8_t REG_PROXIMITY_DATA = 0x9C;
    static constexpr uint8_t REG_PROXIMITY_OFFSET = 0x9D;
    static constexpr uint8_t REG_CONFIG3 = 0x9F;

    static constexpr uint8_t REG_GESTURE_THRESHOLD = 0xA0;
    static constexpr uint8_t REG_GESTURE_CONFIG = 0xA2;
    static constexpr uint8_t REG_GESTURE_OFFSET_UP = 0xA4;
    static constexpr uint8_t REG_GESTURE_OFFSET_DOWN = 0xA5;
    static constexpr uint8_t REG_GESTURE_PULSE = 0xA6;
    static constexpr uint8_t REG_GESTURE_OFFSET_LEFT = 0xA7;
    static constexpr uint8_t REG_GESTURE_OFFSET_RIGHT = 0xA9;
    static constexpr uint8_t REG_GESTURE_CONFIG3 = 0xAA;
    static constexpr uint8_t REG_GESTURE_CONFIG4 = 0xAB;
    static constexpr uint8_t REG_GESTURE_FIFO_COUNT = 0xAE;
    static constexpr uint8_t REG_GESTURE_STATUS = 0xAF;

    static constexpr uint8_t REG_IFRORCE = 0xE4;
    static constexpr uint8_t REG_PICLEAR = 0xE5;
    static constexpr uint8_t REG_CICLEAR = 0xE6;
    static constexpr uint8_t REG_AICLEAR = 0xE7;

    static constexpr uint8_t REG_GESTURE_DATA = 0xFC;

    //Register Data Size if not just 1
    static constexpr size_t REG_ALS_INT_THRESHOLDS_SIZE = 4;
    static constexpr size_t REG_PROXIMITY_INT_THRESHOLDS_SIZE = 4;
    static constexpr size_t REG_RGBC_DATA_SIZE = 8;
    static constexpr size_t REG_PROXIMITY_DATA_SIZE = 4;
    static constexpr size_t REG_GESTURE_DATA_SIZE = 4;

    // ENABLE Register Bits
    static constexpr uint8_t ENABLE_GEN = 6;
    static constexpr uint8_t ENABLE_PIEN = 5;
    static constexpr uint8_t ENABLE_AIEN = 4;
    static constexpr uint8_t ENABLE_WEN = 3;
    static constexpr uint8_t ENABLE_PEN = 2;
    static constexpr uint8_t ENABLE_AEN = 1;
    static constexpr uint8_t ENABLE_PO = 0;

    // PERSISTENCE Register MASKS
    static constexpr uint8_t PERSISTENCE_PPERS_MASK = 0xF0;
    static constexpr uint8_t PERSISTENCE_APERS_MASK = 0x0F;

    // CONFIG1 Register Bits
    static constexpr uint8_t CONFIG1_AGL = 2;
    static constexpr uint8_t CONFIG1_WLONG = 1;
    static constexpr uint8_t CONFIG1_PDL = 0;

    // CONFIG2 Register Bits
    static constexpr uint8_t CONFIG2_PSIEN = 7;
    static constexpr uint8_t CONFIG2_CPSIEN = 6;
    static constexpr uint8_t CONFIG2_SIEN_MASK = 0b1100000;
    static constexpr uint8_t CONFIG2_LEDBOOST_MASK = 0b0011000;

    // CONFIG3 Register Bits
    static constexpr uint8_t CONFIG3_PCMP = 5;
    static constexpr uint8_t CONFIG3_SAI = 4;
    static constexpr uint8_t CONFIG3_PMASK_U = 3;
    static constexpr uint8_t CONFIG3_PMASK_D = 2;
    static constexpr uint8_t CONFIG3_PMASK_L = 1;
    static constexpr uint8_t CONFIG3_PMASK_R = 0;
    static constexpr uint8_t CONFIG3_PBITS_MASK = 0b00101111;

    // GESTURE_CONFIG4 Register Bits
    static constexpr uint8_t GESTURE_CONFIG4_GFIFO_CLEAR = 2;
    static constexpr uint8_t GESTURE_CONFIG4_GIEN = 1;
    static constexpr uint8_t GESTURE_CONFIG4_GMODE = 0;


    static constexpr float MAX_TIME_ADC_MS = 712.0f;
    static constexpr float MIN_TIME_ADC_MS = MS_ADC_TIME_QUOTUM;
    static constexpr float CONV_TIME_ADC_RATIO = 1.0f / MS_ADC_TIME_QUOTUM;

    uint8_t getReg(uint8_t regAddress)
    {
        _wire.beginTransmission(I2C_ADDRESS);
        _wire.write(regAddress);
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
        _wire.write(regAddress);
        _wire.write(regValue);
        _lastError = _wire.endTransmission();
    }

    uint16_t getWord(uint8_t regAddress)
    {
        _wire.beginTransmission(I2C_ADDRESS);
        _wire.write(regAddress);
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

    void setWord(uint8_t regAddress, uint16_t wordValue)
    {
        _wire.beginTransmission(I2C_ADDRESS);
        _wire.write(regAddress);
        _wire.write(wordValue & 0xff);
        _wire.write(wordValue >> 8);
        _lastError = _wire.endTransmission();
    }

    uint8_t msToTimeReg(float msTime) const
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

    float timeRegToMs(uint8_t time) const
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
        // WAIT TIME = 2.73ms (minimum)
        setReg(REG_WTIME, 0xff);
        // Proximity Pulse Count = 8, Pulse Length = 8us 
        setReg(REG_PPULSE, 0x87);

        SetAnalogControl(LedDriveCurrent_Default,
            ProximityGain_Default,
            AlsGain_Default);
    }

#ifdef ADPS_DEBUG
    void printReg(uint8_t reg, uint8_t value)
    {
        Serial.print(" 0x");
        Serial.print(reg, HEX);
        Serial.print(", 0b");
        Serial.println(value, BIN);
    }

    void printGestureRegisters(uint8_t enableRegValue)
    {
        constexpr uint8_t registers[] = { 0x8D, 0x90, 0x93,
                0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAB, 0xAE, 0xAF,
                0xFC, 0xFD, 0xFE, 0xFF };
        constexpr uint8_t registersCount = sizeof(registers) / sizeof(registers[0]);
            
        Serial.println("Gesture Registers: ");

        printReg(REG_ENABLE, enableRegValue);

        for (uint8_t regIndex = 0; regIndex < registersCount; regIndex++)
        {
            uint8_t reg = registers[regIndex];
            uint8_t value = getReg(reg);
            printReg(reg, value);
        }
    }
#endif

};

} // namespace Adps9960