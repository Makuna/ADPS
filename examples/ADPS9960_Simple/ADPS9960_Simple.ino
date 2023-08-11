
// CONNECTIONS:
// ADPS9960 SDA --> SDA
// ADPS9960 SCL --> SCL
// ADPS9960 VCC --> 3.3v
// ADPS9960 GND --> GND

/* for software wire use below
#include <SoftwareWire.h>  // must be included here so that Arduino library object file references work
#include <Adps9960.h>

using namespace ADPS9960;

SoftwareWire myWire(SDA, SCL);

Adps9960<SoftwareWire> Rtc(myWire);
 for software wire use above */

/* for normal hardware wire use below */
#include <Wire.h> // must be included here so that Arduino library object file references work
#include <Adps9960.h>

using namespace ADPS9960;

Adps9960<TwoWire> Adps(Wire);
/* for normal hardware wire use above */


// handy routine to return true if there was an error
// but it will also print out an error message with the given topic
bool wasError(const char* errorTopic = "")
{
    uint8_t error = Adps.LastError();

    if (error != WIRE_UTIL::Error_None)
    {
        // we have a communications error
        // see https://www.arduino.cc/reference/en/language/functions/communication/wire/endtransmission/
        // for what the number means
        Serial.print("[");
        Serial.print(errorTopic);
        Serial.print("] WIRE communications error (");
        Serial.print(error);
        Serial.print(") : ");

        switch (error)
        {

        case WIRE_UTIL::Error_TxBufferOverflow:
            Serial.println("transmit buffer overflow");
            break;

        case WIRE_UTIL::Error_NoAddressableDevice:
            Serial.println("no device responded");
            break;

        case WIRE_UTIL::Error_UnsupportedRequest:
            Serial.println("device doesn't support request");
            break;

        case WIRE_UTIL::Error_Unspecific:
            Serial.println("unspecified error");
            break;

        case WIRE_UTIL::Error_CommunicationTimeout:
            Serial.println("communications timed out");
            break;

        default:
            Serial.println("(unknown?!)");
            break;
        }
        return true;
    }
    return false;
}

void setup () 
{
    Serial.begin(115200);

    //--------ADPS SETUP ------------
    // if you are using ESP-01 then uncomment the line below to reset the pins to
    // the available pins for SDA, SCL
    // Wire.begin(0, 2); // due to limited pins, use pin 0 and 2 for SDA, SCL
    
    Adps.Begin();
#if defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(3000 /* us */, true /* reset_on_timeout */);
#endif

    Serial.println("Initializing...");

    uint8_t id = Adps.GetId();
    if (wasError("setup GetId"))
    {
        // Common Causes:
        //    1) SDA and SCL pins are not correctly set
        //    2) Wiring between Arduino and ADPS is not correct,
        //       make sure the GND is connected between them
    }
    else
    {
        if (!Adps.IsIdValid(id))
        {
            Serial.print("Device ID doesn't match known IDs for ADPS9960!? ");
        }
    }
    Serial.print(" (");
    Serial.print(id, HEX);
    Serial.println(") ");

    Adps.Start(Feature_Proximity_Als);
    if (wasError("setup Start"))
    {
        // Common Causes:
        //    1) WIRE default speed is either too fast or too slow
        //    2) Wiring between Arduino and ADPS is not stable
    }

    Serial.println("Running...");
}

void loop () 
{
    AlsData alsData;
    uint16_t proximity = 0;
    Status status;
    
    status = Adps.GetStatus();
    wasError("loop GetStatus");

    if (status.IsProximityDataValid())
    {
        proximity = Adps.GetProximityData();
        wasError("loop GetProximityData");

        Serial.print(proximity);
    }
    else
    {
        Serial.print("X");
    }
    Serial.print(", ");

    if (status.IsAlsDataValid())
    {
        alsData = Adps.GetAlsData();
        wasError("loop GetAlsData");

        Serial.print(alsData.C);
        Serial.print(", ");
        Serial.print(alsData.R);
        Serial.print(", ");
        Serial.print(alsData.G);
        Serial.print(", ");
        Serial.print(alsData.B);
    }
    else
    {
        Serial.print("X, X, X, X");
    }
    
    Serial.print(", ");
    Serial.print(alsData.CalcLux<LuxCoefficientsOpenAir>());
    Serial.println();

    delay(30); // ~30 times a second
}
