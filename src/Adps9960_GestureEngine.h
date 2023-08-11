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

#include "Gesture_types.h"

namespace ADPS9960
{

template<class T_ADPS, uint8_t V_SAMPLE_DEPTH = 4> class GestureEngine
{
public:
    GestureEngine(uint32_t minTimeMs = 44, uint32_t holdTimeMs = 1000, uint32_t maxTimeMs = 1400) :
        c_MinGestureLengthMs(minTimeMs),
        c_HoldGestureLengthMs(holdTimeMs),
        c_MaxGestureLengthMs(maxTimeMs),
        _state(State_None),
        _entryMs(0),
        _queueSamples(V_SAMPLE_DEPTH),
        _xFirstClass(0),
        _yFirstClass(0)
    {
    }

    void Process(T_ADPS& adps, GestureCallback callback)
    {
        uint32_t processStartMs = millis();
        uint8_t dataCount = adps.GetGestureFifoCount();

        if (adps.LastError() == WIRE_UTIL::Error_None)
        {
            while (dataCount--)
            {
                GestureData data = adps.GetNextGestureData();
                if (adps.LastError() == WIRE_UTIL::Error_None)
                {
                    processGestureData(processStartMs, data);
                }
                    
            }

            // processGestureData may have reset _entryMs, so we need to
            // calc delta after it but before we use it
            uint32_t deltaMs = processStartMs - _entryMs;

            if (_state < State_Held)
            {
                if (deltaMs > c_MaxGestureLengthMs)
                {
#ifdef ADPS_DEBUG
                    Serial.print("  too long (");
                    Serial.print(deltaMs);
                    Serial.print(") ");
                    Serial.print(" {");
                    Serial.print(_state);
                    Serial.println("}");
#endif
                    _state = State_Exit;
                }
                else if (deltaMs > c_HoldGestureLengthMs)
                {
                    _state = State_Held;
                    processGestureDataEnd(callback);
                    _state = State_Exit;
                }
            }

            GestureStatus gestureStatus = adps.GetGestureStatus();
            if (adps.LastError() == WIRE_UTIL::Error_None)
            {
                if (!gestureStatus.IsDataValid())
                {
                    if (deltaMs < c_MinGestureLengthMs)
                    {
#ifdef ADPS_DEBUG
                        Serial.print("  too short (");
                        Serial.print(deltaMs);
                        Serial.println(")");
#endif
                    }
                    else
                    {
                        // exited gesture mode
#ifdef ADPS_DEBUG
                        Serial.println("[GEND] ");
#endif
                        processGestureDataEnd(callback);
                    }

                    _state = State_None;

                    Status status = adps.GetStatus();
                    if (adps.LastError() == WIRE_UTIL::Error_None)
                    {
                        if (status.IsGestureIntAsserted())
                        {
                            adps.LatchInterrupt(Feature_Gesture);
#ifdef ADPS_DEBUG
                            Serial.println("ADPS in int assert with no data?");
#endif
                        }
                    }
                }

            }
        }
    }

    void Poll(T_ADPS& adps, GestureCallback callback, uint32_t pollIntervalMs)
    {
        static uint32_t LastPollTime = 0;
        uint32_t now = millis();
        uint32_t delta = (now - LastPollTime);

        if (delta >= pollIntervalMs)
        {
            LastPollTime = now;

            GestureStatus gestureStatus = adps.GetGestureStatus();
            if (adps.LastError() == WIRE_UTIL::Error_None)
            {
                if (gestureStatus.IsDataValid())
                {
                    Process(adps, callback);
                }
            }
        }
    }

protected:
    enum State
    {
        State_None,
        State_Entry_1st,
        State_Entry_Last = State_Entry_1st + V_SAMPLE_DEPTH - 1, // captured n first entries
        State_Over_1st,
        State_Over_Last = State_Over_1st + V_SAMPLE_DEPTH - 1, // captured n consective last entries
        State_Held,
        State_Exit,
    };

    const uint32_t c_MinGestureLengthMs;
    const uint32_t c_HoldGestureLengthMs;
    const uint32_t c_MaxGestureLengthMs;

    uint8_t _state;
    uint32_t _entryMs;
    
    CircularQueue<GestureData> _queueSamples;
    int8_t _xFirstClass;
    int8_t _yFirstClass;

    void processGestureDataEnd(GestureCallback callback)
    {
        if (_state == State_Over_Last)
        {
            // we have collected enough to make an informed guess at the gesture
            //

            updateExitFirstClassification();

            // second level classification and
            // convert into GestureVector for callback
            GestureVector gesture = GestureVector_Unknown;
            int8_t epsilon = 3;
            int8_t absX = abs(_xFirstClass);
            int8_t absY = abs(_yFirstClass);

            if (absY > absX + epsilon)
            {
                // primarily vertical
                if (_yFirstClass < 0)
                {
                    gesture = GestureVector_Down;
                }
                else
                {
                    gesture = GestureVector_Up;
                }
            }
            else if (absX > absY + epsilon)
            {
                // primarily horizontal
                if (_xFirstClass < 0)
                {
                    gesture = GestureVector_Left;
                }
                else
                {
                    gesture = GestureVector_Right;
                }
            }

            callback(gesture);
        }
        else if (_state == State_Held)
        {
            callback(GestureVector_Hold);
        }
    }

    void processGestureData(uint32_t processStartMs, GestureData data)
    {
        if (_state < State_Held)
        {
            if (_state == State_None)
            {
#ifdef ADPS_DEBUG
                Serial.println("[GBEGIN] ");
#endif
                _entryMs = processStartMs;

                // prepare queue for gesture entry samples
                _queueSamples.Clear();
                _state++;
            }

            if (_state <= State_Entry_Last)
            {
                _queueSamples.Enqueue(data);
                if (_state == State_Entry_Last)
                {
                    // reset first classification
                    _xFirstClass = 0;
                    _yFirstClass = 0;

                    updateEntryFirstClassification();

                    // prepare queue for gesture exit samples
                    _queueSamples.Clear();
                }
                _state++;
            }
            else if (_state <= State_Over_Last)
            {
                _queueSamples.Enqueue(data);
                if (_state < State_Over_Last)
                {
                    _state++;
                }
            }
        }
    }

    void updateEntryFirstClassification()
    {
        // include some from first gesture samples in the classification
        for (size_t iQueue = 0; iQueue < _queueSamples.Count; iQueue++)
        {
            auto minmax = _queueSamples[iQueue].FindMinMax();
            // importance decreases toward last
            uint8_t importance = _queueSamples.Count - iQueue; 

            switch (minmax.MinIndex)
            {
            case GestureDirection_Up:
                _yFirstClass += importance;
                break;
            case GestureDirection_Down:
                _yFirstClass -= importance;
                break;
            case GestureDirection_Left:
                _xFirstClass -= importance;
                break;
            case GestureDirection_Right:
                _xFirstClass += importance;
                break;
            }
        }
    }

    void updateExitFirstClassification()
    {
        // include the last samples into the classification
        for (size_t iQueue = 0; iQueue < _queueSamples.Count; iQueue++)
        {
            auto minmax = _queueSamples[iQueue].FindMinMax();
            // importance increases toward last
            uint8_t importance = iQueue + 2; 

            switch (minmax.MinIndex)
            {
            case GestureDirection_Up:
                _yFirstClass -= importance;
                break;
            case GestureDirection_Down:
                _yFirstClass += importance;
                break;
            case GestureDirection_Left:
                _xFirstClass += importance;
                break;
            case GestureDirection_Right:
                _xFirstClass -= importance;
                break;
            }
        }
    }
};

}