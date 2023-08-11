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

namespace WIRE_UTIL
{
    // While WIRE has return codes, there is no standard definition of what they are
    // within any headers; they are only documented on the website here
    // https://www.arduino.cc/reference/en/language/functions/communication/wire/endtransmission/
    // So we define our own "standard" for this library that match those
    //
    enum Error
    {
        Error_None = 0,
        Error_TxBufferOverflow,
        Error_NoAddressableDevice,
        Error_UnsupportedRequest,
        Error_Unspecific,
        Error_CommunicationTimeout
    };
}