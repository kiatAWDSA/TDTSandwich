#include "TDTSandwich.h"
#include <Adafruit_MAX31856.h>
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
 *                                                                                    Replace {5} with duration (ms) for the heating for countdown timer. Leave as 0 if no countdown desired. Format is xxxxxxxxxx  (include leading and trailing zeroes)
 *  Stop heating:                       c                     where 'c' means 'cool'
 *  Change setpoint:                    t{0}                  where 't' means 'temperature'. Replace {0} with setpoint desired in the format xxx.xx    (include leading and trailing zeroes)
 *  
 *  
 *  A "PID phase" is defined as the period of time when the PID params are fixed and Arduino is sending pulses to the SSR based on these PID params. Once this period of time is over, the PID params will be recalculated and the next PID phase begins.
 *  
 *  created 2017
 *  by Soon Kiat Lau
 */

// Sandwich characteristic variables
const uint8_t sandwichID = 13; // THIS MUST BE DIFFERENT FOR EVERY SANDWICH. This allows the C# identify the ports the sandwich are connected to. Must be more than 0 and less than 100
const uint8_t heaterCount = 2;

// Digital pins
const uint8_t heater2TPin = 2;
const uint8_t speakerPin = 3; // MUST be a PWM pin because we are using analogWrite for speaker
const uint8_t speakerGroundPin = 4;
const uint8_t SSRPin[heaterCount] = { 5, 6 }; // Heater 1 is pin 5; heater 2 is pin 6
const uint8_t sampleTPin = 7;
const uint8_t heater1TPin = 10;

// Analog pins (used as digital outputs)
const uint8_t LEDRedPin = 14; // analog pin A0
const uint8_t LEDGreenPin = 15; // analog pin A1
const uint8_t sampleTVoltagePin = 16; // analog pin A2
const uint8_t sampleTGroundPin = 17; // analog pin A3

// TODO: DELETE; test for include .h
MAX31856 test = MAX31856(heater1TPin, true);

// Thermocouple amplifier vars
// The pins of the amplifier should be connected as follows (    Notation: (amplifier pin) -> (Arduino Uno pin) [(Arduino Uno pin number)]    )
// CS -> SS     [Can be any digital output pin. For the heater thermocouple amplifier, it should be 10 (because the breakout board physically has the CS pin on top of the Arduino Uno's pin 10)]
// SDI -> MOSI  [pin 11]
// SDO -> MISO  [pin 12]
// SCK -> SCK   [pin 13]
// If only the first argument is provided, the remaining pins will be defaulted to the values listed above (i.e. using hardware SPI which is supposedly faster than software SPI). 
Adafruit_MAX31856 heater1Amplifier = Adafruit_MAX31856(heater1TPin, true); // true for auto-conversion
Adafruit_MAX31856 heater2Amplifier = Adafruit_MAX31856(heater2TPin, true); // true for auto-conversion
Adafruit_MAX31856 sampleAmplifier = Adafruit_MAX31856(sampleTPin, true); // true for auto-conversion
max31856_oversampleCount overSamplingMode;  // Number of samples to be measured by the thermocouple amplifiers that will be averaged


// Common vars
bool DAQSample;
bool blinkLED;    // LED blinking is only allowed when the sandwich is idle. Once DAQ is started, it should be forced to false and the C# program should disable the blink button
bool blinkLEDOn;
bool measuring;
bool heating;
bool countDown;
bool speakerOn;
bool heatingComplete;
uint8_t blinkLEDCount = 10;       // Number of times the LED should blink when asked by computer program
uint8_t blinkLEDCounter;          // Keeps track of how many times the LED has blinked
uint8_t sampleErr;
float TSetpoint;
float sampleTReading;
unsigned long cycleTime = 250;          // Each temperature measurement/PID cycle lasts 250 ms. Used as a delay between each temperature reading when heating is not active.
unsigned long blinkLEDDuration = 200;   // The period (ms) for one LED blink
unsigned long blinkLEDOnDuration = 100; // During blinking, the LED will be on for 100 ms, and then off for (blinkLEDDuration - blinkLEDOnDuration)ms
unsigned long blinkLEDEndTime;          // The time at which blinking should stop
unsigned long curTime;
unsigned long endTime;
unsigned long heatDuration; // ms
unsigned long countDownBeepDuration = 100; // The speaker will beep for about 100 ms during countdown. This value must be between 0 and 1000.
unsigned long heatingCompleteBeepDuration = 3000; // Duration (ms) that the speaker will beep once the heating duration has been completed.
unsigned long countDownMax = 10000; // start countdown at 10,000 ms (10 s)
unsigned long countDownCounter; // Units are in ms
unsigned long nextTReadingTime; // The time to take the next temperature reading. Only used when heating is not active.

// Communication char buffers
char TSetpointChar[6];      // xxx.xx\0 Dummy for storing setpoint before conversion to float. +1 for null character.
char heatRateChar[7];       // xxx.xx\0 Dummy for storing heating rate before conversion to float. +1 for null character.
char TStepDurationChar[6];      // xx.xx\0 Dummy for storing temperature step duration before conversion to float. +1 for null character.
char KcChar[8];             // xxxx.xx\0 Dummy for storing PID proportional constant before conversion to float. +1 for null character.
char TiChar[8];             // xxxx.xx\0 Dummy for storing PID integral constant before conversion to float. +1 for null character.
char heatDurationChar[11];  // xxxxxxxxxx\0 Dummy for storing duration (ms) of heating. Maximum duration is 4,294,967,295 ms. +1 for null character
char comCommandType;        // This would be g, s, h, c, t, or i (idle state; to indicate that there are no commands from computer. This is set by Arduino itself).
char comCommandParams[41]; // The characters following the command type. Maximum is 40 chars, contributed by the 'h' state. +1 for null character.

// Shared PID variables
float heatRate; // The heating rate in °C/s. Arduino actually receives this in °C/min, but will convert to /s automatically.
float TStepDuration;
float Kc;
float Ti;
uint8_t PIDPWMDuration = 120; // Mains is 60 Hz, and each cycle has 2 halves. Each "bump" of a mains cycle will last for 1/(60*2) = 8.33 ms. Add some SSR delay and we get about 10 ms per bump. A PID duration of 120 ms gives approx 12 bumps, therefore we are limited to 12 kinds of duty cycles because of the SSR zero-crossing. 
uint8_t PIDPWMCycleCount = 2; // The more bumps, the finer control we get, but the delay between each PWM duty cycle will increase. A good compromise is to use same amount of bumps, but increase the number of PWM cycles.

// Heater-specific PID variables
bool PIDActive[heaterCount];
bool highStarted[heaterCount]; // Has the high portion of a single PID PWM cycle started yet?
bool lowStarted[heaterCount]; // Has the low portion of a single PID PWM cycle been started yet?
bool freshTReading[heaterCount];
bool firstHeatingCycle[heaterCount]; // Are we doing the first PID/PWM heating cycle for the current heating step? (used to check if we should reset ui or not)
bool heatStarted[heaterCount];
bool TStepping[heaterCount];
uint8_t heaterErr[heaterCount];
float heaterTReading[heaterCount];
float TTarget[heaterCount];
float err[heaterCount]; // This is the PID error between setpoint and PV, NOT error from the thermocouple amplifiers
float errPrev[heaterCount];
float up[heaterCount];
float ui[heaterCount];
float uiPrev[heaterCount];
float output[heaterCount];
uint8_t highTime[heaterCount];
uint8_t lowTime[heaterCount];
uint8_t PIDPWMCycleCounter[heaterCount]; // For keeping track of how many PWM cycles have been completed so far in a single PID phase
unsigned long PIDPWMEndHighTime[heaterCount];
unsigned long PIDPWMEndLowTime[heaterCount];
unsigned long TStepTimeStart[heaterCount];
unsigned long TStepTimeEnd[heaterCount];


void setup() {
  blinkLED = false;
  measuring = false;
  heating = false;
  countDown = false;
  speakerOn = false;
  heatingComplete = false;
  nextTReadingTime = 0;
  sampleErr = 0;

  for (int h = 0; h < heaterCount; h++)
  {
    highStarted[h] = false;
    PIDActive[h] = false;
    lowStarted[h] = false;
    freshTReading[h] = false;
    firstHeatingCycle[h] = false;
    heatStarted[h] = false;
    heaterErr[h] = 0;
    TStepTimeStart[h] = 0;
    TStepTimeEnd[h] = 0;
    pinMode(SSRPin[h], OUTPUT);
  }

  // The heater and sample amplifier pins are autoset to OUTPUT by the MAX31856 class
  pinMode(LEDRedPin, OUTPUT);
  pinMode(LEDGreenPin, OUTPUT);
  pinMode(speakerPin, OUTPUT);
  pinMode(sampleTVoltagePin, OUTPUT);
  digitalWrite(sampleTVoltagePin, HIGH);
  pinMode(sampleTGroundPin, OUTPUT);
  digitalWrite(sampleTGroundPin, LOW);
  pinMode(speakerGroundPin, OUTPUT);
  digitalWrite(speakerGroundPin, LOW);
  
  Serial.begin(9600);
}

void loop() {


  
  if ( !measuring )
  {
    if ( comCommandType == 'g' || comCommandType == 'b' )
    { // Start temperature measurement
      // Set thermocouple type. The last two are not thermocouple types, they're just 'plain' voltage readings (check the datasheet for more details, these modes are not used in the library)
      switch ( comCommandParams[1] )
      {
        case 1: overSamplingMode = MAX31856_OVERSAMPLE_1; break;
        case 2: overSamplingMode = MAX31856_OVERSAMPLE_2; break;
        case 4: overSamplingMode = MAX31856_OVERSAMPLE_4; break;
        case 8: overSamplingMode = MAX31856_OVERSAMPLE_8; break;
        default: overSamplingMode = MAX31856_OVERSAMPLE_1; break;
      }

      // Use hardware SPI (supposedly faster than software SPI). The pins of the amplifier
      // should be connected as follows (    Notation: (amplifier pin) -> (Arduino Uno pin) [(Arduino Uno pin number)]    )
      // CS -> SS     [Can be any digital output pin. For the heater thermocouple amplifier, it should be 10 (because the breakout board physically has the CS pin on top of the Arduino Uno's pin 10)]
      // SDI -> MOSI  [pin 11]
      // SDO -> MISO  [pin 12]
      // SCK -> SCK   [pin 13]
      heater1Amplifier.open();
      heater2Amplifier.open();
      
      switch ( comCommandParams[0] )
      {
        case 'B': heater1Amplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_B, overSamplingMode); heater2Amplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_B, overSamplingMode); break;
        case 'E': heater1Amplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_E, overSamplingMode); heater2Amplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_E, overSamplingMode); break;
        case 'J': heater1Amplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_J, overSamplingMode); heater2Amplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_J, overSamplingMode); break;
        case 'K': heater1Amplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_K, overSamplingMode); heater2Amplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_K, overSamplingMode); break;
        case 'N': heater1Amplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_N, overSamplingMode); heater2Amplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_N, overSamplingMode); break;
        case 'R': heater1Amplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_R, overSamplingMode); heater2Amplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_R, overSamplingMode); break;
        case 'S': heater1Amplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_S, overSamplingMode); heater2Amplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_S, overSamplingMode); break;
        case 'T': heater1Amplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_T, overSamplingMode); heater2Amplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_T, overSamplingMode); break;
        //case MAX31856_VMODE_G8: Serial.println("Voltage x8 Gain mode"); break;
        //case MAX31856_VMODE_G32: Serial.println("Voltage x8 Gain mode"); break;
        default: heater1Amplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_T, overSamplingMode); heater2Amplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_T, overSamplingMode); Serial.write("e001\n"); break;
      }

      // Begin temperature conversion
      heater1Amplifier.startAutoConversion();
      heater2Amplifier.startAutoConversion();

      if (comCommandType == 'b')
      { // DAQ for both the heater and sample thermocouples
        DAQSample = true;
        sampleAmplifier.open();

        switch ( comCommandParams[0] )
        {
          case 'B': sampleAmplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_B, overSamplingMode); break;
          case 'E': sampleAmplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_E, overSamplingMode); break;
          case 'J': sampleAmplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_J, overSamplingMode); break;
          case 'K': sampleAmplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_K, overSamplingMode); break;
          case 'N': sampleAmplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_N, overSamplingMode); break;
          case 'R': sampleAmplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_R, overSamplingMode); break;
          case 'S': sampleAmplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_S, overSamplingMode); break;
          case 'T': sampleAmplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_T, overSamplingMode); break;
          //case MAX31856_VMODE_G8: Serial.println("Voltage x8 Gain mode"); break;
          //case MAX31856_VMODE_G32: Serial.println("Voltage x8 Gain mode"); break;
          default: sampleAmplifier.setThermocoupleTypeAndOversampling(MAX31856_TCTYPE_T, overSamplingMode); Serial.write("e031\n"); break;
        }
        
        sampleAmplifier.startAutoConversion();
      }
      else
      {
        DAQSample = false;
      }

      measuring = true;
      comCommandType = 'i';
      blinkLED = false;
      digitalWrite(LEDGreenPin, HIGH); // Constant green light indicates DAQ is ongoing

      delay(250); // First conversion takes a while to provide a measurement; 250 ms should be more than enough for oversampling=2.
      // Without this delay, the Arduino will send its most recent conversion to the computer. If the amplifier haven't done any conversion (i.e. equipment was just started),
      // the Arduino will send a value of 0, thus causing a false alarm for temperature reading of 0.
      // If more than 2 samples are being taken, see pg 20 of manual to add additional time for this first measurement.
      
      // Next step is to step out of this if/else structure and start measuring
    }
    else if ( comCommandType == 'w' )
    { // Computer is asking for the block ID
      Serial.write('w');
      Serial.print(sandwichID);
      Serial.write('\n');
      comCommandType = 'i';
    }
    else if ( comCommandType == 'l' )
    {
      blinkLED = true;
      blinkLEDOn = false;
      blinkLEDCounter = blinkLEDCount;
      blinkLEDEndTime = millis() + blinkLEDCount * blinkLEDDuration;
      comCommandType = 'i';
    }
  }
  else if ( comCommandType != 'i' )
  {
    switch ( comCommandType )
    {
      case 'h':
        // Start heating
        // First, parse the char array into float numbers
        for (int i = 0; i < 5; i++)
        {
          TSetpointChar[i] = comCommandParams[i];
        }
        for (int i = 0; i < 6; i++)
        {
          heatRateChar[i] = comCommandParams[i + 5];
        }
        for (int i = 0; i < 5; i++)
        {
          TStepDurationChar[i] = comCommandParams[i + 5 + 6];
        }
        for (int i = 0; i < 7; i++)
        {
          KcChar[i] = comCommandParams[i + 5 + 6 + 5];
        }
        for (int i = 0; i < 7; i++)
        {
          TiChar[i] = comCommandParams[i + 5 + 6 + 5 + 7];
        }
        for (int i = 0; i < 10; i++)
        {
          heatDurationChar[i] = comCommandParams[i + 5 + 6 + 5 + 7 + 7];
        }

        TSetpoint = atof(TSetpointChar);
        heatRate = atof(heatRateChar) / 60; // Convert to °C/s
        TStepDuration = atof(TStepDurationChar);
        Kc = atof(KcChar);
        Ti = atof(TiChar);
        heatDuration = atof(heatDurationChar);

        // Initiate countdown variables
        // Note that the heating might not start immediately because the program might still be waiting for a fresh T reading. So the countdown has an error of up to 250 ms.
        if (heatDuration > 0)
        {
          countDown = true;
          heatingComplete = false;
          endTime = millis() + heatDuration;
          countDownCounter = countDownMax;
        }
        else
        {
          countDown = false;
        }

        heating = true;

        for (int h = 0; h < heaterCount; h++)
        {
          heatStarted[h] = true;
          TStepping[h] = true;
          PIDActive[h] = false;
          
          // Reset integral params
          ui[h] = 0;
          uiPrev[h] = 0;
          errPrev[h] = 0;
        }
        
        comCommandType = 'i';

        digitalWrite(LEDRedPin, HIGH); // Constant green and red light indicates heater and PID controls are on
        break;
      case 'c':
        // Stop heating
        digitalWrite(speakerPin, LOW);
        heating = false;
        countDown = false;

        for (int h = 0; h < heaterCount; h++)
        {
          PIDActive[h] = false;
          digitalWrite(SSRPin[h], LOW);
        }
        
        comCommandType = 'i';

        digitalWrite(LEDRedPin, LOW); // Constant green light indicates DAQ is ongoing
        break;
      case 's':
        // Stop measuring temperature
        measuring = false;
        heating = false;

        // Close connections to the thermocouple amplifiers (which automatically stops auto-conversion too)
        heater1Amplifier.close();
        heater2Amplifier.close();

        if (DAQSample)
        {
          sampleAmplifier.close();
        }
        
        for (int h = 0; h < heaterCount; h++)
        {
          PIDActive[h] = false;
          digitalWrite(SSRPin[h], LOW);
        }
        
        comCommandType = 'i';

        digitalWrite(LEDGreenPin, LOW); // No lights are on; no DAQ ongoing
        break;
        /* // TODO: The setpoint changing functionality is removed for now because the C# implementation will cause some space constraints. User can always stop heating, adjust setpoint, and start heating again if needed.
      case 't':
        // Change setpoint
        digitalWrite(SSRPin, LOW);
        PIDActive = false; // Recalculate the PID params
        
        for (int i = 0; i < 6; i++)
        {
          TSetpointChar[i] = comCommandParams[i];
        }
        
        TSetpoint = atof(TSetpointChar);
        comCommandType = 'i';

        // If the new setpoint is outside of the PID zone (whether above or below), reset the PID integral params. If it is inside, let the current PID algorithm slowly stabilize to the setpoint
        if (abs(TSetpoint - heaterTReading) > PIDZone)
        {
          uiPrev = 0;
          errPrev = 0;
        }
        break;
        */
      default:
        Serial.print("e002"); // unrecognized command, or sending the 'g' signal even though system is already running
        Serial.print(comCommandType); // Send to computer the unrecognized command that was received
        Serial.write('\n');
        comCommandType = 'i';
        break;
    }
  }
  
  // This time will be used in all remaining calculations of this loop's instance
  curTime = millis();


  if (blinkLED)
  {
    if (curTime < blinkLEDEndTime)
    {
      if (blinkLEDOn && (curTime >= blinkLEDEndTime - (blinkLEDCounter * blinkLEDDuration - blinkLEDOnDuration)))
      {
        blinkLEDCounter = blinkLEDCounter - 1;
        blinkLEDOn = false;
        digitalWrite(LEDGreenPin, LOW);
      }
      else if (!blinkLEDOn && (curTime >= blinkLEDEndTime - (blinkLEDCounter * blinkLEDDuration)))
      {
        blinkLEDOn = true;
        digitalWrite(LEDGreenPin, HIGH);
      }
    }
    else
    {
      blinkLED = false;
      digitalWrite(LEDGreenPin, LOW);
    }
  }
 

  if ( measuring && !PIDActive[0] && !PIDActive[1] && curTime >= nextTReadingTime )
  { // Get temperature reading and send to computer if DAQ is active and not in PID phase
    // nextTReadingTime will be refreshed and a reading will be taken if
    // DAQ is active, PID is not currently active, and the current value of nextReadingTime has been surpassed.
    // This means that during heating, nextTReadingTime will only be updated after a PID cycle has been completed
    nextTReadingTime = curTime + cycleTime;

    // Only take T readings if the PID is not currently active
    // Note that the temperature obtained here, if the amplifier is set to auto mode, will be at most ~100 ms old. But the delay for obtaining a reading is minimal.
    // If the amplifier is set to one-shot mode, the reading is fresh, but there will be a ~250 ms delay for obtaining a reading (which then makes it older than auto mode?!)
    heaterTReading[0] = heater1Amplifier.readThermocoupleTemperature();
    heaterErr[0] = heater1Amplifier.readFault();
    heaterTReading[1] = heater2Amplifier.readThermocoupleTemperature();
    heaterErr[1] = heater2Amplifier.readFault();

    if (DAQSample)
    {
      sampleTReading = sampleAmplifier.readThermocoupleTemperature();
      sampleErr = sampleAmplifier.readFault();
    }
    
    
    if (heaterErr[0] || heaterErr[1] || (DAQSample && sampleErr)) // If we have heater smplifier error, or we are reading sample and have a sample amplifier error
    {
      if (heaterErr[0])
      {
        if (heaterErr[0] & MAX31856_FAULT_OPEN)    Serial.write("e012\n"); // Thermocouple Open Fault
        if (heaterErr[0] & MAX31856_FAULT_CJRANGE) Serial.write("e013\n"); // Cold Junction Range Fault
        if (heaterErr[0] & MAX31856_FAULT_TCRANGE) Serial.write("e014\n"); // Thermocouple Range Fault
        if (heaterErr[0] & MAX31856_FAULT_CJHIGH)  Serial.write("e015\n"); // Cold Junction High Fault
        if (heaterErr[0] & MAX31856_FAULT_CJLOW)   Serial.write("e016\n"); // Cold Junction Low Fault
        if (heaterErr[0] & MAX31856_FAULT_TCHIGH)  Serial.write("e017\n"); // Thermocouple High Fault
        if (heaterErr[0] & MAX31856_FAULT_TCLOW)   Serial.write("e018\n"); // Thermocouple Low Fault
        if (heaterErr[0] & MAX31856_FAULT_OVUV)    Serial.write("e019\n"); // Over/Under Voltage Fault
      }
      else if (heaterErr[1])
      {
        if (heaterErr[1] & MAX31856_FAULT_OPEN)    Serial.write("e022\n"); // Thermocouple Open Fault
        if (heaterErr[1] & MAX31856_FAULT_CJRANGE) Serial.write("e023\n"); // Cold Junction Range Fault
        if (heaterErr[1] & MAX31856_FAULT_TCRANGE) Serial.write("e024\n"); // Thermocouple Range Fault
        if (heaterErr[1] & MAX31856_FAULT_CJHIGH)  Serial.write("e025\n"); // Cold Junction High Fault
        if (heaterErr[1] & MAX31856_FAULT_CJLOW)   Serial.write("e026\n"); // Cold Junction Low Fault
        if (heaterErr[1] & MAX31856_FAULT_TCHIGH)  Serial.write("e027\n"); // Thermocouple High Fault
        if (heaterErr[1] & MAX31856_FAULT_TCLOW)   Serial.write("e028\n"); // Thermocouple Low Fault
        if (heaterErr[1] & MAX31856_FAULT_OVUV)    Serial.write("e029\n"); // Over/Under Voltage Fault
      }
      else
      { // Error from sample amplifier
        if (sampleErr & MAX31856_FAULT_OPEN)    Serial.write("e032\n"); // Thermocouple Open Fault
        if (sampleErr & MAX31856_FAULT_CJRANGE) Serial.write("e033\n"); // Cold Junction Range Fault
        if (sampleErr & MAX31856_FAULT_TCRANGE) Serial.write("e034\n"); // Thermocouple Range Fault
        if (sampleErr & MAX31856_FAULT_CJHIGH)  Serial.write("e035\n"); // Cold Junction High Fault
        if (sampleErr & MAX31856_FAULT_CJLOW)   Serial.write("e036\n"); // Cold Junction Low Fault
        if (sampleErr & MAX31856_FAULT_TCHIGH)  Serial.write("e037\n"); // Thermocouple High Fault
        if (sampleErr & MAX31856_FAULT_TCLOW)   Serial.write("e038\n"); // Thermocouple Low Fault
        if (sampleErr & MAX31856_FAULT_OVUV)    Serial.write("e039\n"); // Over/Under Voltage Fault
      }
    }
    else if (heaterTReading[0] <= 0 || heaterTReading[1] <= 0 || (DAQSample && sampleTReading <= 0))
    { // If there was a problem with any of the amplifiers, the readings would be exactly 0. If the thermocouple wires fudged up, sometimes they give negative readings (like -400)
      // This check is intended to prevent heating from continuously occuring when these situations give rise to temperature readings way below their actual value.
      // However, if the thermocouple is actually reading exactly 0 or below freezing (not possible for room temperature samples and experiments), then this safety check will give a false warning
      
      if (heaterTReading[0] <= 0)
      {
        Serial.write("e011\n"); // Connection error on heater 1 thermocouple amplifier
		    heaterErr[0] = 1; // Force an error so that the heaters will turn off.
      }
      else if (heaterTReading[1] <= 0)
      {
        Serial.write("e021\n"); // Connection error on heater 2 thermocouple amplifier
		    heaterErr[1] = 1; // Force an error so that the heaters will turn off.
      }
      else if (DAQSample && sampleTReading <= 0)
      {
        Serial.write("e030\n"); // Connection error on sample thermocouple amplifier
		    sampleErr = 1; // Force an error so that the heaters will turn off. TODO: Should we really kill the heaters if the sample T reading doesn't work? Because it shouldn't cause runaway heating in the heaters anyway...
      }

      // The PID loop should have set the freshTReading to false already, but JUST IN CASE....
      for (int h = 0; h < heaterCount; h++)
      {
        freshTReading[h] = false;
      }
    }
    else
    {
      for (int h = 0; h < heaterCount; h++)
      {
        freshTReading[h] = true;
      }
      
      Serial.write('t'); // temperature
      Serial.print(heaterTReading[0]);
      Serial.write(',');
      Serial.print(heaterTReading[1]);
      Serial.write(',');
      Serial.print(curTime); // milliseconds
      
      if (DAQSample)
      {
        Serial.write(',');
        Serial.print(sampleTReading);
      }
      
      Serial.write('\n');
    }
  }
  else
  { // If no T readings are taken, force the freshTReading to be false.
    // This section is necessary to prevent heating from starting when the if-check above fails (which will happen while waiting for the PID cycle to finish)
    for (int h = 0; h < heaterCount; h++)
    {
      freshTReading[h] = false;
    }
  }


  // Countdown code block should be before heating code block, otherwise a new heating cycle might be initiated before the countdown checks are reached.
  if (countDown)
  { // We have a time limit set on the heating
    // Do the time check for the countdown first so we can skip the rest of the if-else block if we are not even in the countdown region yet
    // Notice: the countdown region is defined as within the countDownMax time range, NOT the countDownCounter (which will keep decreasing)
    if (curTime >= (endTime - countDownMax)) // cannot use "curTime - endTime <= countDownMax" because the times are unsigned longs that will have unexpected behavior when having a negative number
    {
      if ( curTime < endTime )
      {
        if ( speakerOn && ( curTime >= (endTime - (countDownCounter  - countDownBeepDuration)) ) )
        { // Past the beeping duration; mute speaker and the red LED
          countDownCounter = countDownCounter - 1000; // Reduce by a second
          speakerOn = false;
          digitalWrite(speakerPin, LOW);
          digitalWrite(LEDRedPin, LOW);
        }
        else if ( !speakerOn && (curTime >= (endTime - countDownCounter)) )
        { // Just passed a one second mark; beep speaker and turn on red LED
          speakerOn = true;
          analogWrite(speakerPin, 60);
          digitalWrite(LEDRedPin, HIGH);
        }
      }
      else
      {
        // End of the heating duration. The instructions here should be similar to when the Arduino receives the 'c' command
        speakerOn = true;
        heating = false;
        countDown = false;
        heatingComplete = true;
        analogWrite(speakerPin, 60);
        digitalWrite(LEDRedPin, LOW);

        for (int h = 0; h < heaterCount; h++)
        {
          digitalWrite(SSRPin[h], LOW);
        }
        
        Serial.write('d'); // done with heating
        Serial.write('\n');
      }
    }
  }

  if (heatingComplete)
  {
    if (curTime >= (endTime + heatingCompleteBeepDuration))
    {
      heatingComplete = false;
      speakerOn = false;
      digitalWrite(speakerPin, LOW);
    }
  }


  if (heating)
  {
    if (!heaterErr[0] && !heaterErr[1] && !sampleErr)
    {
      for (int h = 0; h < heaterCount; h++)
      {
        if (!PIDActive[h] && freshTReading[h])
        { // Haven't started a PID phase, or the previous PID phase has just ended; and we have a fresh temperature reading
          freshTReading[h] = false;
          
          if (TStepping[h])
          {
            if (curTime >= TStepTimeEnd[h] || heatStarted[h])
            {
              firstHeatingCycle[h] = true;
              TStepTimeStart[h] = curTime;
              ui[h] = 0;
              
              if (heatStarted[h])
              { // Initialize the target temperature using current temperature reading
                TTarget[h] = heaterTReading[h] + heatRate*TStepDuration;
                heatStarted[h] = false;
              }
              else
              { // Once the stepping has begun, all we have to do is add unto the previous target temperature
                TTarget[h] = TTarget[h] + heatRate*TStepDuration;
              }
    
              if (TTarget[h] >= TSetpoint)
              { // Scale down the target temperature to setpoint if we exceed it. Stop further stepping
                TStepping[h] = false;
                TTarget[h] = TSetpoint;
                TStepTimeEnd[h] = 0; // It doesn't matter what this value is, since if TStepping is false, we will totally ignore temperature stepping and won't be checking the time
              }
              else
              {
                TStepTimeEnd[h] = TStepTimeStart[h] + TStepDuration*1000; // This is in ms!
              }
            }
          }
          
          err[h] = TTarget[h] - heaterTReading[h];
          
          // Since our output will be duty cycle of PWM (0% to 100%), constrain the up within this range
          // TODO Maybe make up[h] and ui[h] dimensionless and ratio-ed so don't have to constrain the output..
          up[h] = constrain(Kc * err[h], 0, 100);
      
          if (firstHeatingCycle[h])
          {
            ui[h] = 0;
            firstHeatingCycle[h] = false;
          }
          else
          {
            if (Ti > 0)
            {
              // If the integral output causes the total output to be above 100, scale it down so we don't have wind-up
              // Force the lower limit to be non-negative so the system responds instantaneously when dipping below the setpoint
              
              ui[h] = constrain( uiPrev[h] + Kc / Ti * (err[h] + errPrev[h])/2 * cycleTime/1000      , 0     , 100 - up[h]);
            }
            else
            {
              ui[h] = 0;
            }
          }

          output[h] = up[h] + ui[h];
          errPrev[h] = err[h];
          uiPrev[h] = ui[h];
  
          highTime[h] = round(output[h] / 100 * PIDPWMDuration);
          lowTime[h] = PIDPWMDuration - highTime[h];
          PIDPWMEndHighTime[h] = curTime + highTime[h];
          PIDPWMEndLowTime[h] = curTime + lowTime[h];
    
          // Begin PID
          PIDActive[h] = true;
          highStarted[h] = false;
          lowStarted[h] = false;
          PIDPWMCycleCounter[h] = 0;
        }
        
        if (PIDActive[h])
        { // PID params have been recalculated or we are currently in a PID phase
          // The PID phase lasts for 240 ms. The missing 10 ms (to reach 250 ms) is allocated to thermocouple reading and the other if-else checks.
          // Bit-bang the output because the Arduino's PWM frequency is too high (~490 Hz for most pins) for the SSR (on-off time ~10 ms or 100 Hz)
    
          if (!highStarted[h])
          {
            highStarted[h] = true;
  
            if ( highTime[h] > 0)
            {
              digitalWrite(SSRPin[h], HIGH);
            }
          }
          else
          {
            if ( !lowStarted[h] && curTime >= PIDPWMEndHighTime[h] )
            {
                lowStarted[h] = true;
  
                if ( lowTime[h] > 0)
                {
                  digitalWrite(SSRPin[h], LOW);
                }
            }
            else if ( lowStarted[h] && curTime >= PIDPWMEndLowTime[h] )
            { // End of one PWM cycle
              // DO NOT force the SSRPin to be low here, because sometimes there are consecutive PID cycles where it is 100% high (e.g. ramping up for shortest come-up time).
              // If we force-write a low at the end of each cycle, then we will be interrupting those 100% high cycles.
              // If you are planning to force SSRPin to low here for safety purposes, it's instead better to improve the error-checking to ensure we always catch any weird behavior.
              highStarted[h] = false;
              lowStarted[h] = false;
              
              PIDPWMCycleCounter[h]++;
    
              if ( PIDPWMCycleCounter[h] >= PIDPWMCycleCount)
              {
                PIDActive[h] = false;
              }
              else
              {
                // Recalculate the timers for high and low for the next PID cycle
                PIDPWMEndHighTime[h] = curTime + highTime[h];
                PIDPWMEndLowTime[h] = curTime + (PIDPWMDuration - highTime[h]);
              }
            }
          }
        }

      }
    }
    else
    {
      // One of the thermocouples has an error. Stop the heaters to prevent runaway heating.

      // Reset the PID loop control variables
      for (int h = 0; h < heaterCount; h++)
      {
        digitalWrite(SSRPin[h], LOW);
        PIDActive[h] = false;
        highStarted[h] = false;
        lowStarted[h] = false;
      }
    }
  } // End heating section
  
} // End program loop


// Function that is called whenever serial data is received
void serialEvent() {
  comCommandType = Serial.read();
  Serial.readBytesUntil('\n', comCommandParams, 41);
  //Serial.write(comCommandType);
  //Serial.print(comCommandParams);
  //Serial.write('\n');
}
