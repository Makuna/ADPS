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

template <typename T_VALUE> class CircularQueue
{
public:
    CircularQueue(size_t count) :
        Count(count)
    {
        _queue = new T_VALUE[count];
        Clear();
    }

    ~CircularQueue()
    {
        delete[] _queue;
    }

    void Enqueue(T_VALUE value)
    {
        _queue[_back++] = value;
        if (_back >= Count)
        {
            _back = 0;
        }
    }

    void Clear(T_VALUE value = 0)
    {
        _back = 0;
        for (size_t idx = 0; idx < Count; idx++)
        {
            _queue[idx] = value;
        }
    }

    T_VALUE operator[](size_t idx) const
    {
        if (idx >= Count)
        {
            idx = Count - 1;
        }
        return _queue[idx];
    }

    const size_t Count;

protected:

    size_t _back;
    T_VALUE* _queue;
};

/*
template <typename T_VALUE, size_t V_COUNT> struct ValueVector
{
    template<typename ...T_ARGS>
    ValueVector(T_ARGS... arg)
    {
        T_VALUE args[]{ arg... }; //unpack 
        size_t argc = sizeof(args) / sizeof(args[0]);
        size_t argi = 0;

        for (size_t idx = 0; idx < Count; idx++)
        {
            Values[idx] = args[argi];

            if (argi < (argc - 1))
            {
                argi++;
            }
        }
    }

    T_VALUE operator[](size_t idx) const
    {
        if (idx >= Count)
        {
            idx = Count - 1;
        }
        return Values[idx];
    }

    static constexpr size_t Count = V_COUNT; // elements in []

    T_VALUE Values[V_COUNT];
};

template <typename T_VALUE>
using Quad = ValueVector<T_VALUE, 4>;

template <typename T_VALUE>
using Pair = ValueVector<T_VALUE, 2>;

*/



enum GestureVector
{ 
    GestureVector_Up,
    GestureVector_Down,
    GestureVector_Left,
    GestureVector_Right,
    GestureVector_Hold,
    GestureVector_Unknown,
};

typedef void(*GestureCallback)(GestureVector gesture);

}
