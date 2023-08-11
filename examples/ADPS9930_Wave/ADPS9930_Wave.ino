
// CONNECTIONS:
// ADPS9930 SDA --> SDA
// ADPS9930 SCL --> SCL
// ADPS9930 VCC --> 3.3v
// ADPS9930 GND --> GND
// ADPS9930 INT --> (Pin19) Don't forget to pullup (4.7k to 10k to VCC)

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


// Interrupt Pin Lookup Table
// (copied from Arduino Docs)
//
// CAUTION:  The interrupts are Arduino numbers NOT Atmel numbers
//   and may not match (example, Mega2560 int.4 is actually Atmel Int2)
//   this is only an issue if you plan to use the lower level interupt features
//
// Board           int.0    int.1   int.2   int.3   int.4   int.5
// ---------------------------------------------------------------
// Uno, Ethernet    2       3
// Mega2560         2       3       21      20     [19]      18 
// Leonardo         3       2       0       1       7

#define ThresholdIntPin 19 // Mega2560
#define ThresholdInterrupt 4 // Mega2560

// marked volatile so interrupt can safely modify them and
// other code can safely read and modify them
volatile uint16_t interuptCount = 0;
volatile bool interuptFlag = false;

void ISR_ATTR InteruptServiceRoutine()
{
    // since this interupted any other running code,
    // don't do anything that takes long and especially avoid
    // any communications calls within this routine
    interuptCount++;
    interuptFlag = true;
}

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

    // set the interupt pin to input mode
    pinMode(ThresholdIntPin, INPUT);

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

#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_MEGAAVR)
    // setup external interupt 
    // for some Arduino hardware they use interrupt number for the first param
    attachInterrupt(ThresholdInterrupt, InteruptServiceRoutine, FALLING);
#else
    // for some Arduino hardware they use interrupt pin for the first param
    attachInterrupt(ThresholdIntPin, InteruptServiceRoutine, FALLING);
#endif

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
    if (interuptFlag)
    {
        bool waved = false;
        interuptFlag = false;
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
            Serial.print(interuptCount);
            Serial.println(" Thanks for waving");
        }
    }
}
