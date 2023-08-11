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

namespace ADPS9960
{

enum Feature
{
    Feature_None,
    Feature_Proximity = 0b00000001,
    Feature_AmbiantLightSensor = 0b00000010,
    Feature_Proximity_Als = 0b00000011,
    Feature_Gesture = 0b00000100,
    Feature_Gesture_Proximity = 0b00000101,
    Feature_Gesture_Als = 0b00000110,
    Feature_Gesture_Proximity_Als = 0b00000111,
};

enum ProximityPulseLength
{
    ProximityPulseLength_4us,
    ProximityPulseLength_8us,
    ProximityPulseLength_16us,
    ProximityPulseLength_32us,

    ProximityPulseLength_Default = ProximityPulseLength_8us
};

enum LedDriveCurrent
{
    // 100%
    LedDriveCurrent_100mA,
    LedDriveCurrent_50mA,
    LedDriveCurrent_25mA,
    LedDriveCurrent_12_5mA,
    // 150%
    LedDriveCurrent_150mA = 0x10,
    LedDriveCurrent_75mA,
    LedDriveCurrent_37_5mA,
    LedDriveCurrent_18_75mA,
    // 200%
    LedDriveCurrent_200mA = 0x20,
    LedDriveCurrent_100mA_200p,
    LedDriveCurrent_50mA_200p,
    LedDriveCurrent_25mA_200p,
    // 300%
    LedDriveCurrent_300mA = 0x30,
    LedDriveCurrent_150mA_300P,
    LedDriveCurrent_75mA_300p,
    LedDriveCurrent_37_5mA_300p,

    LedDriveCurrent_Default = LedDriveCurrent_100mA,
    LedDriveCurrent_GestureDefault = LedDriveCurrent_300mA
};

enum ProximityGain
{
    ProximityGain_1x,
    ProximityGain_2x,
    ProximityGain_4x,
    ProximityGain_8x,

    ProximityGain_Default = ProximityGain_2x
};

enum AlsGain
{
    AlsGain_1x,
    AlsGain_4x,
    AlsGain_16x,
    AlsGain_64x,

    AlsGain_Default = AlsGain_4x
};

enum GestureGain
{
    GestureGain_1x,
    GestureGain_2x,
    GestureGain_4x,
    GestureGain_8x,

    GestureGain_Default = GestureGain_2x
};

enum PhotoDiode
{
    PhotoDiode_None = 0,
    PhotoDiode_R = 0b00000001,
    PhotoDiode_L = 0b00000010,
    PhotoDiode_D = 0b00000100,
    PhotoDiode_U = 0b00001000,
    PhotoDiode_RL_Pair = 0b00000011,
    PhotoDiode_DU_Pair = 0b00001100,
    PhotoDiode_All = 0xb00001111,
};

enum GestureFifoThreshold
{
    GestureFifoThreshold_1,
    GestureFifoThreshold_4,
    GestureFifoThreshold_8,
    GestureFifoThreshold_16,

    GestureFifoThreshold_Default = GestureFifoThreshold_4
};

enum GestureExitPresistence
{
    GestureExitPresistence_1st,
    GestureExitPresistence_2nd,
    GestureExitPresistence_4th,
    GestureExitPresistence_7th,

    GestureExitPresistence_Default = GestureExitPresistence_1st
};

enum GestureWaitTime
{
    GestureWaitTime_0ms,
    GestureWaitTime_2_8ms,
    GestureWaitTime_5_6ms,
    GestureWaitTime_8_4ms,
    GestureWaitTime_14ms,
    GestureWaitTime_22_4ms,
    GestureWaitTime_30_8ms,
    GestureWaitTime_39_2ms,

    GestureWaitTime_Default = GestureWaitTime_2_8ms
};

enum GestureDirection
{
    GestureDirection_Up,
    GestureDirection_Down,
    GestureDirection_Left,
    GestureDirection_Right,
    GestureDirection_None,
};

struct GestureDirectionData
{
    GestureDirectionData(GestureDirection primary = GestureDirection_None,
        GestureDirection secondary = GestureDirection_None) :
        Primary(primary),
        Secondary(secondary)
    {
    }

    bool IsInconclusive()
    {
        return (Primary == GestureDirection_None);
    }

    GestureDirection Primary;
    GestureDirection Secondary;
};

constexpr float MS_ADC_TIME_QUOTUM = 2.78f;
constexpr uint8_t ALS_ADC_TIME_DEFAULT = 0xf6; // 27.8ms
constexpr float MS_ALS_ADC_TIME_DEFAULT = MS_ADC_TIME_QUOTUM * (256 - ALS_ADC_TIME_DEFAULT);

struct Status
{
    Status(uint8_t status = 0) :
        _status(status)
    {
    }

    bool IsClearPhotodiodeSaturated() const
    {
        return (_status & _BV(STATUS_CPSAT));
    }

    bool IsProximityGestureSaturated() const
    {
        return (_status & _BV(STATUS_PGSAT));
    }

    bool IsProximityIntAsserted() const
    {
        return (_status & _BV(STATUS_PINT));
    }

    bool IsAlsIntAsserted() const
    {
        return (_status & _BV(STATUS_AINT));
    }

    bool IsGestureIntAsserted() const
    {
        return (_status & _BV(STATUS_GINT));
    }

    bool IsProximityDataValid() const
    {
        return (_status & _BV(STATUS_PVALID));
    }

    bool IsAlsDataValid() const
    {
        return (_status & _BV(STATUS_AVALID));
    }

private:
    uint8_t _status;

    // STATUS Register Bits
    static constexpr uint8_t STATUS_CPSAT = 6;
    static constexpr uint8_t STATUS_PGSAT = 6;

    static constexpr uint8_t STATUS_PINT = 5;
    static constexpr uint8_t STATUS_AINT = 4;
    static constexpr uint8_t STATUS_GINT = 2;
    static constexpr uint8_t STATUS_PVALID = 1;
    static constexpr uint8_t STATUS_AVALID = 0;
};

struct GestureStatus
{
    GestureStatus(uint8_t status = 0) :
        _status(status)
    {
    }

    bool IsFifoOverflow() const
    {
        return (_status & _BV(STATUS_GFOV));
    }

    bool IsDataValid() const
    {
        return (_status & _BV(STATUS_GVALID));
    }

private:
    uint8_t _status;

    // STATUS Register Bits
    static constexpr uint8_t STATUS_GFOV = 1;
    static constexpr uint8_t STATUS_GVALID = 0;
};

struct LuxCoefficientsOpenAir
{
    static constexpr float GA = 0.49f; // glass attenuation factor
    static constexpr float B = 1.862f;
    static constexpr float C = 0.746f;
    static constexpr float D = 1.291f;
};

struct AlsData
{
    AlsData(uint16_t clear = 0,
        uint16_t red = 0,
        uint16_t green = 0,
        uint16_t blue = 0) :
        C(clear),
        R(red),
        G(green),
        B(blue)
    {

    }

    template <typename T_LUX_COEFFICIENTS> float CalcLux(
        AlsGain alsGain = AlsGain_Default,
        float msAlsAdcTime = MS_ALS_ADC_TIME_DEFAULT)
    {
        constexpr float DeviceFactor = 52.0f;

        float C1 = (R + G + B) / 3.0f;
        float gain = alsGainToFloat(alsGain);
        float iac1 = C - T_LUX_COEFFICIENTS::B * C1;
        float iac2 = T_LUX_COEFFICIENTS::C * C - T_LUX_COEFFICIENTS::D * C1;
        float iac = (iac1 > iac2) ? iac1 : iac2;
        if (iac < 0.0f)
        {
            iac = 0.0f;
        }
        float lpc = T_LUX_COEFFICIENTS::GA * DeviceFactor / (gain * msAlsAdcTime);
        float lux = iac * lpc;
        return lux;
    }

    uint16_t C;
    uint16_t R;
    uint16_t G;
    uint16_t B;

private:
    float alsGainToFloat(AlsGain alsGain)
    {
        const float alsGainTable[] = { 1.0f, 4.0f, 16.0f, 64.0f };

        return alsGainTable[alsGain];
    }
};


struct MinMaxGestureValues
{
    size_t MinIndex;
    size_t MaxIndex;
    uint8_t MinValue;
    uint8_t MaxValue;
};

struct GestureData
{
    GestureData(uint8_t up = 0,
        uint8_t down = 0,
        uint8_t left = 0,
        uint8_t right = 0) :
        Up(up),
        Down(down),
        Left(left),
        Right(right)
    {
    }

    // ------------------------------------------------------------------------
    // operator [] - readonly
    // access elements in order by index rather than Up,Down,Left,Right
    // GestureDirection can be used for idx 
    // ------------------------------------------------------------------------
    uint8_t operator[](size_t idx) const
    {
        switch (idx)
        {
        case 0:
            return Up;
        case 1:
            return Down;
        case 2:
            return Left;
        default:
            return Right;
        }
    }

    MinMaxGestureValues FindMinMax() const
    {
        MinMaxGestureValues result = { 0,0,255,0 };

        for (uint8_t index = 0; index < Count; index++)
        {
            uint8_t value = (*this)[index];

            if (value < result.MinValue)
            {
                result.MinValue = value;
                result.MinIndex = index;
            }
            if (value > result.MinValue)
            {
                result.MaxValue = value;
                result.MaxIndex = index;
            }
        }

        return result;
    };

    uint8_t Up;
    uint8_t Down;
    uint8_t Left;
    uint8_t Right;

    static constexpr size_t Count = 4; // elements in []
};

} // namespace