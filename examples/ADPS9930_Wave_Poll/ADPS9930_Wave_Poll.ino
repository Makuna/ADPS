
// CONNECTIONS:
// ADPS9930 SDA --> SDA
// ADPS9930 SCL --> SCL
// ADPS9930 VCC --> 3.3v
// ADPS9930 GND --> GND

/* for software wire use below
#include <SoftwareWire.h>  // must be included here so that Arduino library object file references work
#include <Adps9930.h>

using namespace ADPS9930;

SoftwareWire myWire(SDA, SCL);

Adps9930<SoftwareWire> Rtc(myWire);
 for software wire use above */

/* for normal hardware wire use below */
#include <Wire.h> // must be included here so that Arduino library object file references work
#include <Adps9930.h>

using namespace ADPS9930;

Adps9930<TwoWire> Adps(Wire);
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

uint16_t Wave_HighTrigger = 120;
uint16_t Wave_LowTrigger = 80;

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
            Serial.print("Device ID doesn't match known IDs for ADPS9930!? ");
        }
    }
    Serial.print(" (");
    Serial.print(id, HEX);
    Serial.println(") ");

    // just using the interrupt flag and not the pin
    // trigger interrupt when value crosses above high trigger
    Adps.SetProximityIntThresholds(0, Wave_HighTrigger);
    wasError("setup SetProximityIntThresholds");

    // trigger interrupt only if the reading persists for 8 readings
    Adps.SetThresholdPersistenceFilterCounts(8, 8);
    wasError("setup SetThresholdPersistenceFilterCounts");

    // proximity only, and enable threshold interrupts
    Adps.Start(Feature_Proximity, true);
    wasError("setup Start");

    // wait for proximity data to stabilize and become valid
    while (!Adps.GetStatus().IsProximityDataValid())
    {
        wasError("setup IsProximityDataValid");
        delay(11);
    }

    Serial.println("Running...");
}

// track our two stage wave detection
// 
enum WaveState
{
    WaveState_None,
    WaveState_Over,
};

WaveState State = WaveState_None;

void loop () 
{
    // polling the int flag asserted rather than using an ISR
    // to set a flag in our code, while less effecient on time, if
    // you don't have an extra INT pin this works well
    if (Adps.GetStatus().IsProximityIntAsserted())
    {
        bool waved = false;
        Adps.LatchInterrupt(Feature_Proximity);
        
        uint16_t value = Adps.GetProximityData();
        if (value > Wave_HighTrigger)
        {
            if (State == WaveState_None)
            {
                // trigger next interrupt when value crosses below low trigger
                Adps.SetProximityIntThresholds(Wave_LowTrigger, 0);
                State = WaveState_Over;
            }
        }
        else if (value < Wave_LowTrigger)
        {
            if (State == WaveState_Over)
            {
                // trigger next interrupt when value crosses above high trigger
                Adps.SetProximityIntThresholds(0, Wave_HighTrigger);
                State = WaveState_None;
                waved = true;
            }
        }

        if (waved)
        {
            Serial.println(" Thanks for waving");
        }
    }
}
