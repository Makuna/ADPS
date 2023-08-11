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

namespace ADPS9930
{

enum Feature
{
    Feature_Proximity = 1,
    Feature_AmbiantLightSensor,
    Feature_Proximity_Als,
};

enum LedDriveCurrent
{
    LedDriveCurrent_100mA,
    LedDriveCurrent_50mA,
    LedDriveCurrent_25mA,
    LedDriveCurrent__12mA,

    LedDriveCurrent_11mA = 0x80,
    LedDriveCurrent_5mA,
    LedDriveCurrent_2mA,
    LedDriveCurrent_1mA,

    LedDriveCurrent_Default = LedDriveCurrent_100mA
};

enum ProximityGain
{
    ProximityGain_1x,
    ProximityGain_2x,
    ProximityGain_4x,
    ProximityGain_8x,

    ProximityGain_Default = ProximityGain_1x
};

enum AlsGain
{
    AlsGain_1x,
    AlsGain_8x,
    AlsGain_16x,
    AlsGain_120x,

    AlsGain_1_6x = 0x80, // 1/6 = 0.16x
    AlsGain_8_6x, // 8/6 = 1.28x
    AlsGain_16_6x, // 16/6
    AlsGain_120_6x, // 120/6

    AlsGain_Default = AlsGain_1x,
};

constexpr float MS_ADC_TIME_QUOTUM = 2.73f;
constexpr uint8_t ALS_ADC_TIME_DEFAULT = 0xf6; // 27.3ms
constexpr float MS_ALS_ADC_TIME_DEFAULT = MS_ADC_TIME_QUOTUM * (256 - ALS_ADC_TIME_DEFAULT);

struct Status
{
    Status(uint8_t status = 0) :
        _status(status)
    {
    }

    bool IsProximityThresholdSaturated() const
    {
        return (_status & _BV(STATUS_PSAT));
    }

    bool IsProximityIntAsserted() const
    {
        return (_status & _BV(STATUS_PINT));
    }

    bool IsAlsIntAsserted() const
    {
        return (_status & _BV(STATUS_AINT));
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
    static constexpr uint8_t STATUS_PSAT = 6;
    static constexpr uint8_t STATUS_PINT = 5;
    static constexpr uint8_t STATUS_AINT = 4;
    static constexpr uint8_t STATUS_PVALID = 1;
    static constexpr uint8_t STATUS_AVALID = 0;
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
    AlsData() :
        _ch0(0),
        _ch1(0)
    {
    }

    AlsData(uint16_t ch0,
        uint16_t ch1) :
        _ch0(ch0),
        _ch1(ch1)
    {
    }

    uint16_t Ch0() const
    {
        return _ch0;
    }

    uint16_t Ch1() const
    {
        return _ch1;
    }

    template <typename T_LUX_COEFFICIENTS> float CalcLux(
        AlsGain alsGain = AlsGain_Default,
        float msAlsAdcTime = MS_ALS_ADC_TIME_DEFAULT)
    {
        constexpr float DeviceFactor = 52.0f;

        float gain = alsGainToFloat(alsGain);
        float iac1 = _ch0 - T_LUX_COEFFICIENTS::B * _ch1;
        float iac2 = T_LUX_COEFFICIENTS::C * _ch0 - T_LUX_COEFFICIENTS::D * _ch1;
        float iac = (iac1 > iac2) ? iac1 : iac2;
        if (iac < 0.0f)
        {
            iac = 0.0f;
        }
        float lpc = T_LUX_COEFFICIENTS::GA * DeviceFactor / (gain * msAlsAdcTime);
        float lux = iac * lpc;
        return lux;
    }

private:
    uint16_t _ch0;
    uint16_t _ch1;


    float alsGainToFloat(AlsGain alsGain)
    {
        const float alsGainTable[] = { 1.0f, 8.0f, 16.0f, 120.0f };

        float scale = 1.0f;
        uint8_t gain = alsGain;
        if (gain & AlsGain_1_6x)
        {
            gain &= 0x03;
            scale = 1.0f / 6.0f;
        }

        float value = alsGainTable[gain] * scale;
        return value;
    }
};



} // namespace