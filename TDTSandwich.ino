/**********************************************************************

Copyright (C) 2019 Soon Kiat Lau

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
***********************************************************************/

#include "SerialCommunication.h"
#include "MAX31856.h"
#include "PID_modified.h"
#include "TDTSandwich.h"
#include <stdlib.h> // For atof function

// Number of heaters
const uint8_t HEATER_COUNT = 2;

// Pins
const uint8_t PIN_EXP_LATCH     = 7;
const uint8_t PIN_EXP_OE        = 8;
const uint8_t PIN_SEG_LATCH     = 9;
const uint8_t PIN_HEATER_SSR[HEATER_COUNT]    = { 6, 5 };
const uint8_t PIN_HEATER_CS[HEATER_COUNT]     = { 2, 3 };
const uint8_t PIN_HEATER_DRDY[HEATER_COUNT]   = { A1, A3 };
const uint8_t PIN_HEATER_FAULT[HEATER_COUNT]  = { A0, A2 };
const uint8_t PIN_SAMPLE_CS     = 4;
const uint8_t PIN_SAMPLE_DRDY   = A5;
const uint8_t PIN_SAMPLE_FAULT  = A4;

// Min and max temperatures for the system. This is for safety - the heaters will be shut-off if the readings violate the min/max temperatures
// All numbers are in °C.
// Note the cold-junction temperature are integers because the registers for these temperatures are 8-bit
const double HEATER_TEMP_MIN  = -100;
const double HEATER_TEMP_MAX  = 200;
const double SAMPLE_TEMP_MIN  = -100;
const double SAMPLE_TEMP_MAX  = 200;
const int8_t CJ_TEMP_MIN      = -10;
const int8_t CJ_TEMP_MAX      = 80;

// Serial communication
const unsigned long baudRate = 9600;

SerialCommunication communicator = SerialCommunication();
TDTSandwich sandwich = TDTSandwich( communicator,
                                    PIN_HEATER_SSR,
                                    PIN_HEATER_CS,
                                    PIN_HEATER_DRDY,
                                    PIN_HEATER_FAULT,
                                    PIN_EXP_LATCH,
                                    PIN_EXP_OE,
                                    PIN_SEG_LATCH,
                                    PIN_SAMPLE_CS,
                                    PIN_SAMPLE_DRDY,
                                    PIN_SAMPLE_FAULT,
                                    HEATER_TEMP_MIN,
                                    HEATER_TEMP_MAX,
                                    SAMPLE_TEMP_MIN,
                                    SAMPLE_TEMP_MAX,
                                    CJ_TEMP_MIN,
                                    CJ_TEMP_MAX);


void setup() {
  // Use hardware SPI (supposedly faster than software SPI). The pins of the amplifier
  // should be connected as follows (    Notation: (amplifier pin) -> (Arduino Uno pin) [(Arduino Uno pin number)]    )
  // CS ->  any   [Can be any digital output pin. The hardware SPI requires that pin 10 (SS) is left untouched. If the SS pin ever becomes a LOW INPUT then SPI automatically switches to Slave, so the data direction of the SS pin MUST be kept as OUTPUT.]
  // SDI -> MOSI  [pin 11]
  // SDO -> MISO  [pin 12]
  // SCK -> SCK   [pin 13]
  SPI.begin();
  
  sandwich.init();
  communicator.init(baudRate);
}

void loop()
{
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
          * ^w@
          * where    ^    is SERIAL_CMD_START
          *          w    is SERIAL_CMD_CONNECTION
          *          @    is SERIAL_CMD_END
          */
          communicator.sendSandwichID(sandwich.id);
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
          * ^d/b|[commandID]|[TCType]|[AveCount]@
          * where    ^            is SERIAL_CMD_START
          *          d/b          is SERIAL_CMD_DAQ_START_HEATER  or  SERIAL_CMD_DAQ_START_SAMPLE, depending on measuring heaters only or sample thermocouple too
          *          |            is SERIAL_CMD_SEPARATOR
          *          [commandID]  is the ID of this command
          *          [TCType]     is type of thermocouple, in capital letters. Choose among B, E, J, K, N, R, S, or T
          *          [AveCount]   is number of readings to be averaged over, choose among 1, 2, 4, 8, or 16.
          *          @            is SERIAL_CMD_END
          */
          uint8_t incomingTCType      = communicator.getFragmentChar(2);
          int incomingSampleAveraging = communicator.getFragmentInt(3);
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
            communicator.sendCommandResponse(communicator.getFragmentInt(1), SerialCommunication::SERIAL_CMD_DAQ_START_SAMPLE, true);
          }
          else
          {
            sandwich.startDAQ(actualTCType, actualSampleAveraging, false);
            communicator.sendCommandResponse(communicator.getFragmentInt(1), SerialCommunication::SERIAL_CMD_DAQ_START_HEATER, true);
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
          * ^s|[commandID]@
          * where    ^            is SERIAL_CMD_START
          *          s            is SERIAL_CMD_DAQ_STOP
          *          |            is SERIAL_CMD_SEPARATOR
          *          [commandID]  is the ID of this command
          *          @            is SERIAL_CMD_END
          */
          sandwich.stopDAQ();
          communicator.sendCommandResponse(communicator.getFragmentInt(1), SerialCommunication::SERIAL_CMD_DAQ_STOP, true);
          break;
        }
        case SerialCommunication::SERIAL_CMD_HEAT_START:
        {
          /*********************************
          *          START HEAT            *
          * *******************************/
          /* Begin the heating process.
          * Format:
          * ^h|[commandID]|s|r|d|kp|ki@
          * where    ^    is SERIAL_CMD_START
          *          h    is SERIAL_CMD_HEAT_START
          *          |            is SERIAL_CMD_SEPARATOR
          *          [commandID]  is the ID of this command
          *          s    is the temperature setpoint, in °C
          *          r    is the heating rate, in °C/min
          *          d    is the heating duration, in seconds (to be converted into ms before going into startHeat() function)
          *          kp   is the proportional constant for PI algorithm
          *          ki   is the integral constant for PI algorithm
          *          @    is SERIAL_CMD_END
          */
          sandwich.startHeat( communicator.getFragmentDouble(2),
                              communicator.getFragmentDouble(3),
                              communicator.getFragmentULong(4) * 1000,
                              communicator.getFragmentDouble(5),
                              communicator.getFragmentDouble(6));
          communicator.sendCommandResponse(communicator.getFragmentInt(1), SerialCommunication::SERIAL_CMD_HEAT_START, true);
          break;
        }
        case SerialCommunication::SERIAL_CMD_HEAT_STOP:
        {
          /*********************************
          *            STOP HEAT           *
          * *******************************/
          /* Stop heating.
          * Format:
          * ^c|[commandID]@
          * where    ^            is SERIAL_CMD_START
          *          c            is SERIAL_CMD_HEAT_STOP
          *          |            is SERIAL_CMD_SEPARATOR
          *          [commandID]  is the ID of this command
          *          @            is SERIAL_CMD_END
          */
          sandwich.stopHeat(true);
          communicator.sendCommandResponse(communicator.getFragmentInt(1), SerialCommunication::SERIAL_CMD_HEAT_STOP, true);
          break;
        }
        case SerialCommunication::SERIAL_CMD_BLINK:
        {
          /*********************************
          *             BLINK              *
          * *******************************/
          /* Blinks the flasher LED. Useful for visual identification of the sandwich.
          * Format:
          * ^l|[commandID]@
          * where    ^            is SERIAL_CMD_START
          *          l            is SERIAL_CMD_BLINK
          *          |            is SERIAL_CMD_SEPARATOR
          *          [commandID]  is the ID of this command
          *          @            is SERIAL_CMD_END
          */
          sandwich.blinkFlasher();
          communicator.sendCommandResponse(communicator.getFragmentInt(1), SerialCommunication::SERIAL_CMD_BLINK, true);
          break;
        }
        case SerialCommunication::SERIAL_CMD_SHUTDOWN:
        {
          /*********************************
          *            SHUTDOWN            *
          * *******************************/
          /* Stop all operations of the sandwich
          * Format:
          * ^x@
          * where    ^            is SERIAL_CMD_START
          *          x            is SERIAL_CMD_SHUTDOWN
          *          @            is SERIAL_CMD_END
          */
          sandwich.shutdown();
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
