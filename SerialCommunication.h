/**********************************************************************
Copyright 2018 Soon Kiat Lau

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http ://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#ifndef _SERIALCOMMUNICATION_h
#define _SERIALCOMMUNICATION_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class SerialCommunication
{
  public:
    // Communication standards
    static const char SERIAL_CMD_START            = '^';
    static const char SERIAL_CMD_CONNECTION       = 'w';
    static const char SERIAL_CMD_DAQ_START_HEATER = 'd';
    static const char SERIAL_CMD_DAQ_START_SAMPLE = 'b';
    static const char SERIAL_CMD_DAQ_STOP         = 's';
    static const char SERIAL_CMD_HEAT_START       = 'h';
    static const char SERIAL_CMD_HEAT_STOP        = 'c';
    static const char SERIAL_CMD_BLINK            = 'l';
    static const char SERIAL_CMD_SHUTDOWN         = 'x';
    const char SERIAL_CMD_SEPARATOR               = '|';
    static const char SERIAL_CMD_END              = '@';
    static const char SERIAL_CMD_EOL              = '\n';

    static const char SERIAL_SEND_START                     = '^';
    static const char SERIAL_SEND_CONNECTION                = 'w';  // Used to respond to SERIAL_CMD_CONNECTION_CONTROL
    static const char SERIAL_SEND_TEMP_HEATER               = 't';
    static const char SERIAL_SEND_TEMP_HEATERSAMPLE         = 'b';
    static const char SERIAL_SEND_TEMP_ERROR                = 'e';
    static const char SERIAL_SEND_HEATINGDONE               = 'c';
    static const char SERIAL_SEND_CORRUPTCMD             = 'x';
      static const char SERIAL_SEND_CORRUPTCMD_START         = 's';  // The command string does not have a start flag
      static const char SERIAL_SEND_CORRUPTCMD_END           = 'e';  // The command string does not have an end flag
      static const char SERIAL_SEND_CORRUPTCMD_UNKNOWNCMD    = 'c';  // Unknown command received
      static const char SERIAL_SEND_CORRUPTCMD_PARAM_LESS    = 'l';  // There are fewer fragments in a command string than expected
      static const char SERIAL_SEND_CORRUPTCMD_PARAM_MORE    = 'm';  // There are more fragments in a command string than expected
      static const char SERIAL_SEND_CORRUPTCMD_PARAM_NONE    = 'n';  // There are no fragments in a command string, even though some are expected
    static const char SERIAL_SEND_CHAMBER_CMDRESPONSE    = 'r';  // Used to indicate execution status of a received command
      static const char SERIAL_SEND_CHAMBER_CMDRESPONSE_SUCC = 'y';  // Success
      static const char SERIAL_SEND_CHAMBER_CMDRESPONSE_FAIL = 'n';  // Failed
    static const char SERIAL_SEND_ERROR                   = 'e';
    static const char SERIAL_SEND_SEPARATOR               = '|';
    static const char SERIAL_SEND_END                     = '@';
    static const char SERIAL_SEND_EOL                     = '\n';


    SerialCommunication();
    void init(unsigned long baudRate);

    // Enable/disable sending to computer
    void enableSending();
    void disableSending();

    // Process incoming strings into fragments
    bool processIncoming();

    // Functions for obtaining the extracted fragments from an incoming string
    int           getFragmentInt(uint8_t fragmentIndex);
    char          getFragmentChar(uint8_t fragmentIndex);
    double        getFragmentDouble(uint8_t fragmentIndex);
    unsigned long getFragmentULong(uint8_t fragmentIndex);

    // Functions for sending strings to computer
    void sendSandwichID(uint8_t sandwichID);
    void sendHeaterTemperature(bool heater1TOK, double heater1T, bool heater2TOK, double heater2T);
    void sendHeaterSampleTemperature(bool heater1TOK, double heater1T, bool heater2TOK, double heater2T, bool sampleTOK, double sampleT);
    void sendError(uint8_t deviceID, uint8_t errorCode);
    void sendHeatingDone();
    void sendCommandError(char errorType);
    void sendCommandResponse(uint8_t commandID, char commandType, bool success);




  private:
    // Number of parameters in every command sent by computer
    const uint8_t MAXPARAM_CONNECTION       = 0;
    const uint8_t MAXPARAM_DAQ_START_HEATER = 3;
    const uint8_t MAXPARAM_DAQ_START_SAMPLE = 3;
    const uint8_t MAXPARAM_DAQ_STOP         = 1;
    const uint8_t MAXPARAM_HEAT_START       = 6;
    const uint8_t MAXPARAM_HEAT_STOP        = 1;
    const uint8_t MAXPARAM_BLINK            = 1;
    const uint8_t MAXPARAM_SHUTDOWN         = 0;



    bool serialActive_;
    // Serial communication and buffers
    // Must manually define and add fragmentBufferElement to the char pointer array fragmentBuffer, because the arrays points
    // to char arrays which must be defined.
    static const uint8_t serialBufferLength = 128; // Should be always sufficient for command strings sent by the C# program
    static const uint8_t fragmentMaxCount = 8;        // Max possible of fragments in a command sent by computer.
    static const uint8_t fragmentBufferLength = 20;   // Should be always sufficient for fragments in command strings sent by the C# program
    char serialBuffer[serialBufferLength];
    char fragmentBufferElement0[fragmentBufferLength];
    char fragmentBufferElement1[fragmentBufferLength];
    char fragmentBufferElement2[fragmentBufferLength];
    char fragmentBufferElement3[fragmentBufferLength];
    char fragmentBufferElement4[fragmentBufferLength];
    char fragmentBufferElement5[fragmentBufferLength];
    char fragmentBufferElement6[fragmentBufferLength];
    char fragmentBufferElement7[fragmentBufferLength];
    // This array of char array pointers will be temp storage for fragments extracted from incoming command lines
    // Only parts of the buffer are used for each command; see the constants defined above for MAXPARAM
    char* fragmentBuffer[fragmentMaxCount] = {fragmentBufferElement0 ,
                                              fragmentBufferElement1,
                                              fragmentBufferElement2,
                                              fragmentBufferElement3,
                                              fragmentBufferElement4,
                                              fragmentBufferElement5,
                                              fragmentBufferElement6,
                                              fragmentBufferElement7};
    char trimmedSerialBuffer[serialBufferLength]; // For holding the unextracted fragments portion of the command string
};

#endif