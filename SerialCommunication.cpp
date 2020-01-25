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

#include "SerialCommunication.h"

SerialCommunication::SerialCommunication() {}

void SerialCommunication::init(unsigned long baudRate)
{
  Serial.begin(baudRate);
  serialActive_ = true;
}

void SerialCommunication::enableSending()
{
  serialActive_ = true;
}

void SerialCommunication::disableSending()
{
  serialActive_ = false;
}

bool SerialCommunication::processIncoming()
{
  // All communication must start with SERIAL_CMD_START
  if (Serial.peek() == SERIAL_CMD_START)
  {
    // Extract this command string
    memset(serialBuffer, '\0', serialBufferLength);
    Serial.readBytesUntil(SERIAL_CMD_EOL, serialBuffer, (serialBufferLength - 1));

    // Get the pointer to SERIAL_CMD_END 
    char* endFlag = strchr(serialBuffer, SERIAL_CMD_END);

    // All communication must end with SERIAL_CMD_END 
    // Check if it isn't missing and that there's nothing coming after it
    if (endFlag && serialBuffer[endFlag - serialBuffer + 1] == '\0')
    {
      uint8_t endPos = endFlag - serialBuffer;// Index of SERIAL_CMD_END

      // Remove the start and end flags, so we are only left with the inner string
      // First, clear the processing buffer
      memset(trimmedSerialBuffer, '\0', serialBufferLength);

      // inputBuffer + 1 to start after SERIAL_CMD_START
      // endPos - 1 because array is indexed 0, so only minus 1 for one of the start/end flags
      strncpy(trimmedSerialBuffer, serialBuffer + 1, endPos - 1);

      
      // Extract first fragment
      uint8_t fragmentsExtracted = 0;
      char *tokenPtr = strtok(trimmedSerialBuffer, &SERIAL_CMD_SEPARATOR);

      if (tokenPtr)
      {// There was at least one command fragment
        // The first (or maybe only)  is always the command itself. Based on the command,
        // we can expect the number of parameters that is associated with it
        uint8_t paramsCount;

        switch (tokenPtr[0])
        {
          case SERIAL_CMD_CONNECTION:
            paramsCount = MAXPARAM_CONNECTION;
            break;
          case SERIAL_CMD_DAQ_START_HEATER:
            paramsCount = MAXPARAM_DAQ_START_HEATER;
            break;
          case SERIAL_CMD_DAQ_START_SAMPLE:
            paramsCount = MAXPARAM_DAQ_START_SAMPLE;
            break;
          case SERIAL_CMD_DAQ_STOP:
            paramsCount = MAXPARAM_DAQ_STOP;
            break;
          case SERIAL_CMD_HEAT_START:
            paramsCount = MAXPARAM_HEAT_START;
            break;
          case SERIAL_CMD_HEAT_STOP:
            paramsCount = MAXPARAM_HEAT_STOP;
            break;
          case SERIAL_CMD_BLINK:
            paramsCount = MAXPARAM_BLINK;
            break;
          case SERIAL_CMD_SHUTDOWN:
            paramsCount = MAXPARAM_SHUTDOWN;
			break;
          default:
            // Command unknown; stop everything and send error
            sendCommandError(SERIAL_SEND_CORRUPTCMD_UNKNOWNCMD);
            return(false);
        }

        do
        {// Process the extracted fragment. Notice this is a do-while loop.
          fragmentsExtracted++;

          // Clear the output buffer
          memset(fragmentBuffer[fragmentsExtracted - 1], '\0', fragmentBufferLength);
          strcpy(fragmentBuffer[fragmentsExtracted - 1], tokenPtr); // IMPORTANT: Make sure that fragments[fragmentsExtracted - 1] is a char array with at least as many elements as tokenPtr
          tokenPtr = strtok(NULL, &SERIAL_CMD_SEPARATOR);
          // Repeat the loop while we haven't exceeded the max number of params for this command and while there are more params available
        } while (fragmentsExtracted - 1 < paramsCount && tokenPtr);

        // Check if the number of extracted command params are as expected
        // (because we might exit the while loop above due to tokenPtr being null)
        // Minus one to account for the command itself being one of the fragments
        if (fragmentsExtracted - 1 < paramsCount)
        {
          // Not enough params were extracted
          sendCommandError(SERIAL_SEND_CORRUPTCMD_PARAM_LESS);
          return false;
        }

        // Check if there are still params remaining
        // (because we might exit the while loop above due to fragmentsExtracted >= paramsCount)
        if (tokenPtr)
        {
          // Too many params command params were received, though not all were extracted
          sendCommandError(SERIAL_SEND_CORRUPTCMD_PARAM_MORE);
          return false;
        }

        // Extracted command and associated params, all good to go.
        return true;
      }
      else
      {
        // Not a single fragment is available
        sendCommandError(SERIAL_SEND_CORRUPTCMD_PARAM_NONE);
        return false;
      }  
    }
    else
    {// SERIAL_CMD_END wasn't seen or in wrong location, so assume this is garbage
      // Let C# program know
      sendCommandError(SERIAL_SEND_CORRUPTCMD_END);
      return false;
    }
  }
  else
  {// SERIAL_CMD_START wasn't seen, so assume this is garbage
    // Grab the buffer, but don't store it anywhere. This helps to clear the buffer
    Serial.read();
    // Let C# program know
    sendCommandError(SERIAL_SEND_CORRUPTCMD_START);
    return false;
  }
}

int SerialCommunication::getFragmentInt(uint8_t fragmentIndex)
{
  return atoi(fragmentBuffer[fragmentIndex]);
}

char SerialCommunication::getFragmentChar(uint8_t fragmentIndex)
{
  char* fragmentEntry = fragmentBuffer[fragmentIndex];

  // Return only the first character
  return fragmentEntry[0];
}

double SerialCommunication::getFragmentDouble(uint8_t fragmentIndex)
{
  return atof(fragmentBuffer[fragmentIndex]);
}

unsigned long SerialCommunication::getFragmentULong(uint8_t fragmentIndex)
{
  // This relies on the fragment not being larger than max of long and not being negative.
  return (unsigned long) atol(fragmentBuffer[fragmentIndex]);
}

void SerialCommunication::sendSandwichID(uint8_t sandwichID)
{
  Serial.print(SERIAL_SEND_START);
  Serial.print(SERIAL_SEND_CONNECTION);
  Serial.print(SERIAL_SEND_SEPARATOR);
  Serial.print(sandwichID);
  Serial.print(SERIAL_SEND_END);
  Serial.print(SERIAL_SEND_EOL);
}

void SerialCommunication::sendHeaterTemperature(bool heater1TOK, double heater1T, bool heater2TOK, double heater2T)
{
  Serial.print(SERIAL_SEND_START);
  Serial.print(SERIAL_SEND_TEMP_HEATER);
  Serial.print(SERIAL_SEND_SEPARATOR);
  if (heater1TOK)
  {
    Serial.print(heater1T, 1);  // Only send 1 decimal place
  }
  else
  {
    Serial.print(SERIAL_SEND_TEMP_ERROR);
  }
  Serial.print(SERIAL_SEND_SEPARATOR);
  if (heater2TOK)
  {
    Serial.print(heater2T, 1);  // Only send 1 decimal place
  }
  else
  {
    Serial.print(SERIAL_SEND_TEMP_ERROR);
  }
  Serial.print(SERIAL_SEND_END);
  Serial.print(SERIAL_SEND_EOL);
}

void SerialCommunication::sendHeaterSampleTemperature(bool heater1TOK, double heater1T, bool heater2TOK, double heater2T, bool sampleTOK, double sampleT)
{
  Serial.print(SERIAL_SEND_START);
  Serial.print(SERIAL_SEND_TEMP_HEATERSAMPLE);
  Serial.print(SERIAL_SEND_SEPARATOR);
  if (heater1TOK)
  {
    Serial.print(heater1T, 1);  // Only send 1 decimal place
  }
  else
  {
    Serial.print(SERIAL_SEND_TEMP_ERROR);
  }
  Serial.print(SERIAL_SEND_SEPARATOR);
  if (heater2TOK)
  {
    Serial.print(heater2T, 1);  // Only send 1 decimal place
  }
  else
  {
    Serial.print(SERIAL_SEND_TEMP_ERROR);
  }
  Serial.print(SERIAL_SEND_SEPARATOR);
  if (sampleTOK)
  {
    Serial.print(sampleT, 1);  // Only send 1 decimal place
  }
  else
  {
    Serial.print(SERIAL_SEND_TEMP_ERROR);
  }
  Serial.print(SERIAL_SEND_END);
  Serial.print(SERIAL_SEND_EOL);
}

void SerialCommunication::sendError(uint8_t deviceID, uint8_t errorCode)
{
  Serial.print(SERIAL_SEND_START);
  Serial.print(SERIAL_SEND_ERROR);
  Serial.print(SERIAL_SEND_SEPARATOR);
  Serial.print(deviceID);
  Serial.print(SERIAL_SEND_SEPARATOR);
  Serial.print(errorCode);
  Serial.print(SERIAL_SEND_END);
  Serial.print(SERIAL_SEND_EOL);
}

void SerialCommunication::sendHeatingDone()
{
  Serial.print(SERIAL_SEND_START);
  Serial.print(SERIAL_SEND_HEATINGDONE);
  Serial.print(SERIAL_SEND_END);
  Serial.print(SERIAL_SEND_EOL);
}

// Inform C# program that a given command is not in correct format or has an error.
// We could also send the actual command for better debugging, but that would
// take more communication time. Leave this feature out for now.
void SerialCommunication::sendCommandError(char errorType)
{
  Serial.print(SERIAL_SEND_START);
  Serial.print(SERIAL_SEND_CORRUPTCMD);
  Serial.print(SERIAL_SEND_SEPARATOR);
  Serial.print(errorType);
  Serial.print(SERIAL_SEND_END);
  Serial.print(SERIAL_SEND_EOL);
}

// Inform C# program on the status of a command for a specific chamber
void SerialCommunication::sendCommandResponse(uint8_t commandID, char commandType, bool success)
{
  Serial.print(SERIAL_SEND_START);
  Serial.print(SERIAL_SEND_CHAMBER_CMDRESPONSE);
  Serial.print(SERIAL_SEND_SEPARATOR);
  Serial.print(commandID);
  Serial.print(SERIAL_SEND_SEPARATOR);
  Serial.print(commandType);
  Serial.print(SERIAL_SEND_SEPARATOR);
  if (success)
  {
    Serial.print(SERIAL_SEND_CHAMBER_CMDRESPONSE_SUCC);
  }
  else
  {
    Serial.print(SERIAL_SEND_CHAMBER_CMDRESPONSE_FAIL);
  }
  Serial.print(SERIAL_SEND_END);
  Serial.print(SERIAL_SEND_EOL);
}