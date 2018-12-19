
#include <SerialCommunication.h>
#include <MAX31856.h>
#include <PID_modified.h>
#include "TDTSandwich.h"
#include <stdlib.h> // For atof function
/* Arduino Uno program for communicating with C# program in computer to control TDT sandwiches.
 * 
 * 
 *  Commands from computer:
 *  
 *  Start DAQ for heater only:          g{0}                  where 'g' means 'go'. Replace {0} with thermocouple type from this list [ B E J K N R S T ]
 *  Start DAQ for heater and sample:    b{0}                  where 'b' means 'both'. Replace {0} with thermocouple type from this list [ B E J K N R S T ]
 *  Get sandwich ID:                    w                     where 'w' means 'who?'.
 *  Blink LED:                          l                     where 'l' means 'LED'.
 *  Stop DAQ:                           s                     where 's' means 'stop'
 *  Start heating:                      h{0}{1}{2}{3}{4}{5}   where 'h' means 'heat'. Replace {0} with setpoint desired in the format xxx.x                                                                         (include leading and trailing zeroes)
 *                                                                                    Replace {1} with heating rate in the format xxx.xx                                                                            (include leading and trailing zeroes)
 *                                                                                    Replace {2} with the duration for each temperature step in the format xx.xx                                                   (include leading and trailing zeroes)
 *                                                                                    Replace {3} with PID proportional constant desired in the format xxxx.xx                                                      (include leading and trailing zeroes)
 *                                                                                    Replace {4} with PID integration constant desired in the format xxxx.xx                                                       (include leading and trailing zeroes)
 *                                                                                    **REMOVE THIS** Replace {5} with duration (ms) for the heating for countdown timer. Leave as 0 if no countdown desired. Format is xxxxxxxxxx  (include leading and trailing zeroes)
 *  Stop heating:                       c                     where 'c' means 'cool'
 *  Change setpoint:                    t{0}                  where 't' means 'temperature'. Replace {0} with setpoint desired in the format xxx.xx    (include leading and trailing zeroes)
 *  
 *  
 *  
 *  The pin for SS must be left empty. From SPI.cpp:
 *  // Warning: if the SS pin ever becomes a LOW INPUT then SPI
 *  // automatically switches to Slave, so the data direction of
 *  // the SS pin MUST be kept as OUTPUT.
 *  
 *  
 *  
 *  created 2018
 *  by Soon Kiat Lau
 */


// Sandwich ID
// THIS MUST BE DIFFERENT FOR EVERY SANDWICH. This allows the C# identify the ports the sandwich are connected to. Must be more than 0 and less than 100
const uint8_t sandwichID = 1;

// Number of heaters
const uint8_t heaterCount = 2;

// Pins
const uint8_t heatLEDPin      = 7;
const uint8_t flasherPin      = A0;
const uint8_t buzzerPin       = A1;
const uint8_t heaterSSRPin[heaterCount]   = { 9, 8 };
const uint8_t heaterCSPin[heaterCount]    = { 4, 5 };
const uint8_t heaterDRDYPin[heaterCount]  = { A3, A5 };
const uint8_t heaterFaultPin[heaterCount] = { A2, A4 };
const uint8_t sampleCSPin     = 6;
const uint8_t sampleDRDYPin   = 3;
const uint8_t sampleFaultPin  = 2;

// Min and max temperatures for the system. This is for safety - the heaters will be shut-off if the readings violate the min/max temperatures
// All numbers are in °C
// Note the cold-junction temperature are integers because the registers for these temperatures are 8-bit
const double minHeaterTemperature = 1;
const double maxHeaterTemperature = 200;
const double minSampleTemperature = 1;
const double maxSampleTemperature = 200;
const int8_t minCJTemperature = 10;
const int8_t maxCJTemperature = 40;

// Serial communication
const unsigned long baudRate = 9600;

SerialCommunication communicator = SerialCommunication();
TDTSandwich sandwich = TDTSandwich( communicator,
                                    heatLEDPin,
                                    flasherPin,
                                    buzzerPin,
                                    heaterSSRPin,
                                    heaterCSPin,
                                    heaterDRDYPin,
                                    heaterFaultPin,
                                    sampleCSPin,
                                    sampleDRDYPin,
                                    sampleFaultPin,
                                    minHeaterTemperature,
                                    maxHeaterTemperature,
                                    minSampleTemperature,
                                    maxSampleTemperature,
                                    minCJTemperature,
                                    maxCJTemperature);


void setup() {
  sandwich.init();
  communicator.init(baudRate);
}

void loop()
{
  // TODO: Connection check

  sandwich.run();
}


// Function that is called whenever serial data is received
void serialEvent() {
  /* The Serial buffer is only checked at the end of each loop() iteration:
  * https://forum.arduino.cc/index.php?topic=166650.0
  * This means more than one command could accumulate in the buffer during a single loop() iteration.
  * It is thus necessary to go thru the entire buffer until we are sure it is empty.
  * This can be done by re-checking the buffer [Serial.available()] after processing one command from the buffer:
  * https://arduino.stackexchange.com/a/26416
  */
  while (Serial.available())
  {
    if (communicator.processIncoming())
    {
      char commandType = communicator.getFragmentChar(0);
      // Check the type of command
      switch (commandType)
      {
      case SerialCommunication::SERIAL_CMD_CONNECTION:
      {
        //TODO Set flag to remain connected
        /*********************************
        *   ARDUINO CONNECTION CHECK     *
        * *******************************/
        /* Computer checking for connection to Arduino. This doesn't have command IDs.
        * Format:
        * ^c@
        * where    ^    is SERIAL_CMD_START
        *          w    is SERIAL_CMD_CONNECTION
        *          @    is SERIAL_CMD_END
        */
        communicator.sendSandwichID(sandwichID);
        break;
      }
      case SerialCommunication::SERIAL_CMD_DAQ_START_HEATER:
      case SerialCommunication::SERIAL_CMD_DAQ_START_SAMPLE:
      {
        /*********************************
        *           START DAQ            *
        * *******************************/
        /* Begin data acquisition of temperature of heaters only or of sample thermocouple too.
        * Format:
        * ^d/b|T|A@
        * where    ^    is SERIAL_CMD_START
        *          d/b  is SERIAL_CMD_DAQ_START_HEATER  or  SERIAL_CMD_DAQ_START_SAMPLE, depending on measuring heaters only or sample thermocouple too
        *          |    is SERIAL_CMD_SEPARATOR
        *          T    is type of thermocouple, in capital letters. Choose among B, E, J, K, N, R, S, or T
        *          A    is amount of samples to be averaged over, choose among 1, 2, 4, 8, or 16.
        *          @    is SERIAL_CMD_END
        */
        uint8_t incomingTCType      = communicator.getFragmentChar(1);
        int incomingSampleAveraging = communicator.getFragmentInt(2);
        MAX31856_TCType actualTCType;
        MAX31856_SampleAveraging actualSampleAveraging;

        switch (incomingTCType)
        {
        case 'B':
          actualTCType = TCTYPE_B;
          break;
        case 'E':
          actualTCType = TCTYPE_E;
          break;
        case 'J':
          actualTCType = TCTYPE_J;
          break;
        case 'K':
          actualTCType = TCTYPE_K;
          break;
        case 'N':
          actualTCType = TCTYPE_N;
          break;
        case 'R':
          actualTCType = TCTYPE_R;
          break;
        case 'S':
          actualTCType = TCTYPE_S;
          break;
        case 'T':
        default:
          actualTCType = TCTYPE_T;
          break;
        // Not allowing direct voltage measurement
        }

        switch (incomingSampleAveraging)
        {
        case 2:
          actualSampleAveraging = AVERAGE_2;
          break;
        case 4:
          actualSampleAveraging = AVERAGE_4;
          break;
        case 8:
          actualSampleAveraging = AVERAGE_8;
          break;
        case 16:
          actualSampleAveraging = AVERAGE_16;
          break;
        case 1:
        default:
          actualSampleAveraging = AVERAGE_1;
          break;
        }

        // Measure sample T depending on given command
        if (commandType == SerialCommunication::SERIAL_CMD_DAQ_START_SAMPLE)
        {
          sandwich.startDAQ(actualTCType, actualSampleAveraging, true);
          communicator.sendCommandResponse(SerialCommunication::SERIAL_CMD_DAQ_START_SAMPLE, true);
        }
        else
        {
          sandwich.startDAQ(actualTCType, actualSampleAveraging, false);
          communicator.sendCommandResponse(SerialCommunication::SERIAL_CMD_DAQ_START_HEATER, true);
        }
        break;
      }
      case SerialCommunication::SERIAL_CMD_DAQ_STOP:
      {
        /*********************************
        *            STOP DAQ            *
        * *******************************/
        /* Stop data acquisition of temperature.
        * Format:
        * ^s@
        * where    ^    is SERIAL_CMD_START
        *          s    is SERIAL_CMD_DAQ_STOP
        *          @    is SERIAL_CMD_END
        */
        sandwich.stopDAQ();
        communicator.sendCommandResponse(SerialCommunication::SERIAL_CMD_DAQ_STOP, true);
        break;
      }
      case SerialCommunication::SERIAL_CMD_HEAT_START:
      {
        /*********************************
        *          START HEAT            *
        * *******************************/
        /* Begin the heating process.
        * Format:
        * ^h|s|r|d|kp|ki@
        * where    ^    is SERIAL_CMD_START
        *          h    is SERIAL_CMD_HEAT_START
        *          |    is SERIAL_CMD_SEPARATOR
        *          s    is the temperature setpoint, in °C
        *          r    is the heating rate, in °C/min
        *          d    is the heating duration, in seconds (to be converted into ms before going into startHeat() function)
        *          kp   is the proportional constant for PI algorithm
        *          ki   is the integral constant for PI algorithm
        *          @    is SERIAL_CMD_END
        */
        sandwich.startHeat(communicator.getFragmentDouble(1), communicator.getFragmentDouble(2), communicator.getFragmentULong(3) * 1000, communicator.getFragmentDouble(4), communicator.getFragmentDouble(5));
        communicator.sendCommandResponse(SerialCommunication::SERIAL_CMD_HEAT_START, true);
        break;
      }
      case SerialCommunication::SERIAL_CMD_HEAT_STOP:
      {
        /*********************************
        *            STOP HEAT           *
        * *******************************/
        /* Stop heating.
        * Format:
        * ^c@
        * where    ^    is SERIAL_CMD_START
        *          c    is SERIAL_CMD_HEAT_STOP
        *          @    is SERIAL_CMD_END
        */
        sandwich.stopHeat(true);
        communicator.sendCommandResponse(SerialCommunication::SERIAL_CMD_HEAT_STOP, true);
        break;
      }
      case SerialCommunication::SERIAL_CMD_BLINK:
      {
        /*********************************
        *             BLINK              *
        * *******************************/
        /* Blinks the flasher LED. Useful for visual identification of the sandwich.
        * Format:
        * ^l@
        * where    ^    is SERIAL_CMD_START
        *          l    is SERIAL_CMD_BLINK
        *          @    is SERIAL_CMD_END
        */
        sandwich.blinkFlasher();
        communicator.sendCommandResponse(SerialCommunication::SERIAL_CMD_BLINK, true);
        break;
      }
      default:
      {
        // Unrecognized command; send error
        communicator.sendCommandError(SerialCommunication::SERIAL_SEND_CORRUPTCMD_UNKNOWNCMD);
        break;
      }
      }
    }
  }
}
