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

#include "TDTSandwich.h"

TDTSandwich::TDTSandwich( SerialCommunication& communicator,
                          const uint8_t& heatLEDPin,
                          const uint8_t& flasherPin,
                          const uint8_t& buzzerPin,
                          const uint8_t heaterSSRPin[],
                          const uint8_t heaterCSPin[],
                          const uint8_t heaterDRDYPin[],
                          const uint8_t heaterFaultPin[],
                          const uint8_t& sampleCSPin,
                          const uint8_t& sampleDRDYPin,
                          const uint8_t& sampleFaultPin,
                          const double& minHeaterTemperature,
                          const double& maxHeaterTemperature,
                          const double& minSampleTemperature,
                          const double& maxSampleTemperature,
                          const int8_t& minCJTemperature,
                          const int8_t& maxCJTemperature)
                          :
                          communicator_(communicator),
                          heatLEDPin_(heatLEDPin),
                          flasherPin_(flasherPin),
                          buzzerPin_(buzzerPin),
                          minHeaterTemperature_(minHeaterTemperature),
                          maxHeaterTemperature_(maxHeaterTemperature),
                          minSampleTemperature_(minSampleTemperature),
                          maxSampleTemperature_(maxSampleTemperature),
                          minCJTemperature_(minCJTemperature),
                          maxCJTemperature_(maxCJTemperature)
{

  // Initialize pins for non-heater
  pinMode(heatLEDPin_, OUTPUT);
  pinMode(flasherPin_, OUTPUT);
  pinMode(buzzerPin_, OUTPUT);
  pinMode(sampleCSPin, OUTPUT);
  pinMode(sampleDRDYPin, INPUT);
  pinMode(sampleFaultPin, INPUT);
  deactivateHeatLED_();
  deactivateFlasher_();
  deactivateBuzzer_();

  // Initialize heaters
  for (uint8_t h = 0; h < heaterCount_; h++)
  {// Although the new keyword is not recommended in Arduino, it is OK here since we are only creating these class instances
    // at the beginning, and not on the fly (which could lead to memory issues if not disposed of properly).
    heaterTCAmplifier_[h] = new MAX31856(heaterCSPin[h], heaterDRDYPin[h], heaterFaultPin[h]);
    heaterSSRPin_[h] = heaterSSRPin[h];
    pinMode(heaterSSRPin[h], OUTPUT);
    deactivateHeater_(h);

    // Use dummy numbers for Kp and Ki; these will be set when startHeat command is sent.
    // On the other hand, Kd should always be 0 since we want to use a PI control.
    heaterPID_[h] = PID(&heaterTemperature_[h], &heaterControlOutput_[h], &heaterTempSetpoint_[h], 1, 1, 0, millis(), P_ON_M, DIRECT);
    heaterPID_[h].SetOutputLimits(-PIDLimit_, PIDLimit_);
  }

  // Sample thermocouple amplifier. Notice we don't set limits for thermocouple temperature here
  sampleTCAmplifier_ = new MAX31856(sampleCSPin, sampleDRDYPin, sampleFaultPin);

  // Set all flags to idle state
  DAQHeaterActive_ = false;
  DAQSampleActive_ = false;
  heatActive_ = false;
  flasherBlinking_ = false;
}

TDTSandwich::~TDTSandwich()
{
  stopHeat(true);
  stopDAQ();
}

void TDTSandwich::init()
{
  // Use hardware SPI (supposedly faster than software SPI). The pins of the amplifier
  // should be connected as follows (    Notation: (amplifier pin) -> (Arduino Uno pin) [(Arduino Uno pin number)]    )
  // CS ->  any   [Can be any digital output pin. The hardware SPI requires that pin 10 (SS) is left untouched. If the SS pin ever becomes a LOW INPUT then SPI automatically switches to Slave, so the data direction of the SS pin MUST be kept as OUTPUT.]
  // SDI -> MOSI  [pin 11]
  // SDO -> MISO  [pin 12]
  // SCK -> SCK   [pin 13]
  SPI.begin();

  // Initialize heater thermocouple amplifier
  for (uint8_t h = 0; h < heaterCount_; h++)
  {
    heaterTCAmplifier_[h]->init(false);
    heaterTCAmplifier_[h]->setTCFaultThresholds(minHeaterTemperature_, maxHeaterTemperature_);
    heaterTCAmplifier_[h]->setCJFaultThreshholds(minCJTemperature_, maxCJTemperature_);
  }

  // Initialize sample thermocouple amplifier
  sampleTCAmplifier_->init(false);
  sampleTCAmplifier_->setTCFaultThresholds(minSampleTemperature_, maxSampleTemperature_);
  sampleTCAmplifier_->setCJFaultThreshholds(minCJTemperature_, maxCJTemperature_);
}

void TDTSandwich::run()
{
  if (DAQHeaterActive_)
  {
    getTemperatureReading_();

    if (heatActive_)
    {
      if (!allSetpointReached_)
      {
        adjustTempSetpoint_();
      }
      
      handleHeating_();

      if (countdownActive_)
      {
        handleCountDown_();
      }

      if (countdownFinishing_)
      {
        signalHeatEnd_();
      }
    }
  }

  if (flasherBlinking_)
  {
    handleBlinking_();
  }
}

void TDTSandwich::startDAQ(MAX31856_TCType TCType, MAX31856_SampleAveraging averageSampleCount, bool measureSample)
{
  for (uint8_t h = 0; h < heaterCount_; h++)
  {
    // Set thermocouple type and sample averaging
    heaterTCAmplifier_[h]->setThermocoupleTypeAndOversampling(TCType, averageSampleCount);

    // Begin temperature conversion
    heaterTCAmplifier_[h]->startAutoConversion();

    // Initialize status flags
    heaterTCOK_[h] = true;
    heaterTReadySend_[h] = false;
    heaterTFreshReading[h] = false;
  }

  DAQHeaterActive_ = true;

  // Do the same for sample thermocouples, if needed
  if (measureSample)
  {
    sampleTCAmplifier_->setThermocoupleTypeAndOversampling(TCType, averageSampleCount);
    sampleTCAmplifier_->startAutoConversion();
    sampleTCOK_ = true;
    sampleTReadySend_ = false;
    DAQSampleActive_ = true;
  }
}

void TDTSandwich::stopDAQ()
{
  // If heating is ongoing, turn it off
  if (heatActive_)
  {
    stopHeat(true);
  }

  // Stop temperature conversions for the heater thermocouples
  for (uint8_t h = 0; h < heaterCount_; h++)
  {
    heaterTCAmplifier_[h]->stopAutoConversion();
  }
  DAQHeaterActive_ = false;
  heatActive_ = false;

  // Do the same for the sample thermocouple
  if (DAQSampleActive_)
  {
    sampleTCAmplifier_->stopAutoConversion();
    DAQSampleActive_ = false;
  }
}

// Start heating.
// setPoint must be in �C
// heatingRate must be in �C/min
// heatDuration must be in milliseconds.
void TDTSandwich::startHeat(double setpoint, double heatingRate, unsigned long heatDuration, double Kp, double Ki)
{
  if (heatDuration > 0)
  {
    unsigned long curTime = millis();

    // Heater setpoint adjustment should begin immmediately after we start heating
    nextSetpointAdjustmentTime_ = curTime;

    // Calculate the time at which heating should end.
    // heatDuration should be less than approx. 50 days because that's the time at which
    // unsigned long rolls over.
    heatEndTime_ = curTime + heatDuration;

    for (uint8_t h = 0; h < heaterCount_; h++)
    {
      // Set up the setpoints.
      // Since adjustTempSetpoint_() will need a starting setpoint to modify upon,
      // use the current heater temperature.
      heaterSetpoint_ = setpoint;
      heaterTempSetpoint_[h] = heaterTemperature_[h];

      // Set up PID
      // Leave Kd as 0 since we are using a PI control.
      heaterPID_[h].SetTunings(Kp, Ki, 0);
      heaterPID_[h].Reset();
      heaterPID_[h].setLastTime(curTime);

      // Initialize status flags
      heaterOn_[h] = false;
      setpointReached_[h] = false;
    }

    // Calculate timing for the countdown ticker
    if (heatDuration >= countdownTickTotal_ * countdownTickDuration_)
    {// The heating duration is at least the entire countdown duration.
      countdownTickCounter_ = countdownTickTotal_;
    }
    else
    {// Heating duration is less than the entire countdown duration
      // Need to reduce number of countdown ticks to account for this.
      countdownTickCounter_ = heatDuration / countdownTickDuration_;
    }

    // Convert the heating rate to �C/ms because we utilize millis(), and multiply by the
    // interval of setpoint adjustment to avoid frequent division operations
    setpointAdjustment_ = heatingRate * setpointAdjustmentInterval_ / 60000;

    // Visual indication that heating is ongoing
    activateHeatLED_();

    // Deactivate all other visual/audible indications
    flasherBlinking_ = false;
    deactivateBuzzer_();
    deactivateFlasher_();

    // Initialize status flags
    allSetpointReached_     = false;
    countdownTickComplete_  = false;
    countdownActive_        = true;
    countdownFinishing_     = false;
    heatActive_             = true;
  }
  else
  {// No duration for heating...
    stopHeat(true);
  }
}

// Stop heating
void TDTSandwich::stopHeat(bool deactivateAlert)
{
  if (deactivateAlert)
  {
    deactivateBuzzer_();
    deactivateFlasher_();
  }

  heatActive_ = false;

  // Turn off all the heaters
  for (uint8_t h = 0; h < heaterCount_; h++)
  {
    deactivateHeater_(h);
  }

  // Visual indication that heating is off
  deactivateHeatLED_();
}

// Blink to flasher as visual indication that this sandwich is the one of interest
void TDTSandwich::blinkFlasher()
{
  if (!heatActive_)
  {// Only allow blinking when heating is not ongoing, otherwise it might
    // clash with the countdown blinking.
    flasherBlinking_ = true;
    blinkEndTime_ = millis() + blinkTotal_ * blinkDuration_;
    blinkCounter_ = blinkTotal_;

    // Initialize status flag
    blinkPeriodComplete_ = true;
  }
}

// Stop all sandwich operations
void TDTSandwich::shutdown()
{
  if (DAQHeaterActive_ || DAQSampleActive_)
  {
    if (heatActive_)
    {
      stopHeat(true);
    }

    stopDAQ();
  }
}

// Fetch and store temperature readings from all thermocouple amplifiers,
// provided that they are available and have no faults.
void TDTSandwich::getTemperatureReading_()
{
  for (uint8_t h = 0; h < heaterCount_; h++)
  {
    if (!heaterTReadySend_[h])
    {// Need to get the heater temperature reading; otherwise just chill.
      if (heaterTCAmplifier_[h]->isFaultless())
      {// The heater thermocouple amplifier did not detect any errors
        heaterTCOK_[h] = true;

        if (heaterTCAmplifier_[h]->dataIsReady())
        {// There is new temperature data available
          heaterTemperature_[h] = heaterTCAmplifier_[h]->readTCTemperature();
          heaterTReadySend_[h] = true;
          heaterTFreshReading[h] = true;

          // Trigger open-circuit detection for the next conversion
          heaterTCAmplifier_[h]->checkOpenCircuit(MODE_1);
        }
        // Failing the check for DRDY means no new temperature data, therefore no need to do anything else but wait
      }
      else
      {// The heater thermocouple amplifier detected an error
        // Note: FAULT pin does not assert for "out-of-range" (p15 of datasheet) situations
        // i.e. the converted value falls outside of the thermocouple rated range.
        // It is thus recommended to manually set the high and low thresholds for the hot
        // and cold junctions (since we prolly won't go up to the limits of the rated range).
        // The high/low thresholds do trigger the FAULT pin.
        if (heaterTCOK_[h])
        {// Only take action if this is the first time fault was encountered.
          // Otherwise, do nothing, at least until the fault is cleared, in which case would trigger
          // the previous code block
          heaterTCOK_[h] = false;
          heaterTReadySend_[h] = false;
          handleTCAmplifierFault(heaterTCAmplifier_[h], true, h);
        }
      }
    }
  }
  
  if (DAQSampleActive_)
  {// Only check for sample thermocouple readings if we are measuring sample temperature
    if (!sampleTReadySend_)
    {// Need to get a sample temperature reading; otherwise just chill.
      if (sampleTCAmplifier_->isFaultless())
      {// The sample thermocouple amplifier did not detect any errors
        sampleTCOK_ = true;

        if (sampleTCAmplifier_->dataIsReady())
        {// There is new temperature data available
          sampleTemperature_ = sampleTCAmplifier_->readTCTemperature();
          sampleTReadySend_ = true;

          // Trigger open-circuit detection for the next conversion
          sampleTCAmplifier_->checkOpenCircuit(MODE_1);
        }
        // Failing the check for DRDY means no new temperature data, therefore no need to do anything else but wait
      }
      else
      {// The sample thermocouple amplifier detected an error
        if (sampleTCOK_)
        {// Only take action if this is the first time fault was encountered.
          // Otherwise, do nothing, at least until the fault is cleared, in which case would trigger
          // the previous code block
          sampleTCOK_ = false;
          sampleTReadySend_ = false;
          handleTCAmplifierFault(sampleTCAmplifier_, false, 0);
        }
      }
    }

    // Only send data to computer if ALL thermocouples obey one of the following rules:
    // a) TReadySend_ AND TCOK    : new temperature reading available, and the TC amplifier is OK
    // b) !TReadySend_ AND !TCOK  : there was an error with the TC amplifier
    // c) TReadySend_ AND !TCOK   : this case will never occur due to the logic of the code
    if (!((!heaterTReadySend_[0] && heaterTCOK_[0]) || (!heaterTReadySend_[1] && heaterTCOK_[1]) || (!sampleTReadySend_ && sampleTCOK_)))
    {// Send temperature reading/error notice to computer
      communicator_.sendHeaterSampleTemperature(heaterTCOK_[0], heaterTemperature_[0], heaterTCOK_[1], heaterTemperature_[1], sampleTCOK_, sampleTemperature_);

      // Prepare for new readings
      heaterTReadySend_[0] = false;
      heaterTReadySend_[1] = false;
      sampleTReadySend_ = false;
    }
  }
  else
  {// Not reading temperature of sample; send only heater T readings.
    // Only send data to computer if the HEATER thermocouples obey one of the following rules:
    // a) TReadySend_ AND TCOK    : new temperature reading available, and the TC amplifier is OK
    // b) !TReadySend_ AND !TCOK  : there was an error with the TC amplifier
    // c) TReadySend_ AND !TCOK   : this case will never occur due to the logic of the code
    if (!((!heaterTReadySend_[0] && heaterTCOK_[0]) || (!heaterTReadySend_[1] && heaterTCOK_[1])))
    {// Send temperature reading/error notice to computer
      communicator_.sendHeaterTemperature(heaterTCOK_[0], heaterTemperature_[0], heaterTCOK_[1], heaterTemperature_[1]);

      // Prepare for new readings
      heaterTReadySend_[0] = false;
      heaterTReadySend_[1] = false;
    }
  }
}

void TDTSandwich::adjustTempSetpoint_()
{
  // Check if it is time to adjust the temporary setpoint.
  // Add setpointAdjustmentIntervalBuffer_ to solve problems of "negative" numbers coming from unsigned long subtraction
  if (nextSetpointAdjustmentTime_ + setpointAdjustmentIntervalBuffer_ - millis() < setpointAdjustmentIntervalBuffer_)
  {
    // Set this flag to true first. This assumes the temp setpoints of all heaters have reached
    // the actual setpoint. During the subsequent code block, this flag will be changed to false (and never
    // back to true) if any heater hasn't reached the actual setpoint.
    allSetpointReached_ = true;

    for (uint8_t h = 0; h < heaterCount_; h++)
    {
      if (!setpointReached_[h])
      {// Only move on if the temp setpoint of this heater hasn't reached the temperature setpoint
        if (heaterTemperature_[h] < heaterSetpoint_)
        {
          // The heating rate was transformed to a constant setpointAdjustment_ when startHeat() was called
          // This avoids repeatedly re-calculating the appropriate setpoint adjustment everytime this function is called.
          heaterTempSetpoint_[h] += setpointAdjustment_;

          if (heaterTempSetpoint_[h] >= heaterSetpoint_)
          {// Heater temp setpoint has already reached or exceeded setpoint; fix temp setpoint to actual setpoint
            heaterTempSetpoint_[h] = heaterSetpoint_;
            setpointReached_[h] = true;
          }
          else
          {// The temp setpoint for this heater hasn't reached/exceeded the actual setpoint, therefore
            // not all heaters have achieved the actual setpoint. Set the flag appropriately.
            // Note that this flag will not be modified to true by heaters that have achieved the actual setpoint.
            allSetpointReached_ = false;
          }
        }
        else
        {// Heater temperature has already reached or exceeded setpoint (this could even
         // be at the start of the process); fix temp setpoint to actual setpoint
          heaterTempSetpoint_[h] = heaterSetpoint_;
          setpointReached_[h] = true;
        }
      }
    }

    // Determine when the next setpoint adjustment will be done
    nextSetpointAdjustmentTime_ += setpointAdjustmentInterval_;
  }
  // If the if-check fails, that means it's not time to change setpoint yet
}

// Control the heater output based on temperature readings and keeps
// track of the heating process.
void TDTSandwich::handleHeating_()
{
  for (uint8_t h = 0; h < heaterCount_; h++)
  {
    if (heaterTCOK_[h])
    {// Perform heating only if the temperature readings are OK
      if (heaterTFreshReading[h])
      {// Only perform PID updates if we have new temperature readings
        unsigned long curTime = millis();
        heaterTFreshReading[h] = false;
        heaterPID_[h].Compute(curTime);
        // Serial.print("Output: ");
        // Serial.println(heaterControlOutput_[h]);
        if (heaterControlOutput_[h] > HEATER_ON_MIN)
        {// Outside of minimum output treshold; The treshold exists so that if the on time is extremely small,
          // the heater can just be left off 100% instead of turning on for short time between each PID cycle.

          // Calculate the appropriate time for turning on/off the heater
          // Remember, we can only reach here if the value of heaterControlOutput_ is already bigger
          // than HEATER_ON_MIN, that's why we are turning on the heat
          activateHeater_(h);

          // Calculate timing. Since we are performing heating controls depending on when temperature readings are available,
          // it is difficult to know ahead of time the exact duration between each temperature reading, and thus the PID period.
          // If we assume that the period between each reading is roughly the same, then we can estimate the PID period based on
          // the duration between the previous temperature reading and the current one.
          unsigned long PIDPeriod = curTime - heaterPrevPIDPeriodStartTime_[h];
          heatOnTime_[h] = curTime;
          heatOnDuration_[h] = constrain(heaterControlOutput_[h], HEATER_ON_MIN, PIDLimit_) / PIDLimit_ * PIDPeriod;
          heaterOn_[h] = true;

          // Check if the off duration is above the minimum threshold to prevent dead time between subsequent near-100% dutycycles.
          if (PIDPeriod - heatOnDuration_[h] >= HEATER_OFF_MIN)
          {
            heatOffMinSatisfied[h] = true;
          }
          else
          {
            heatOffMinSatisfied[h] = false;
          }


          // Prepare for next PID cycle
          heaterPrevPIDPeriodStartTime_[h] = curTime;
        }
        else
        {// Output is not sufficient to overcome HEATER_ON_MIN, so turn off the heater
          deactivateHeater_(h);
        }
      }

      // Turn off heat if it is time
      if (heaterOn_[h]	// Only need to switch off if the heater is already on
        &&
        heatOffMinSatisfied[h]	// Only turn off the heater if the off duration is larger than a certain threshold
        &&
        (millis() - heatOnTime_[h]) >= heatOnDuration_[h])	// Timing check
      {
        // Time to turn off the heater
        deactivateHeater_(h);
        heaterOn_[h] = false;
      }
    }
    else
    {// Shut down heating if there was a problem with the heater thermocouples.
      deactivateHeater_(h);
      heaterOn_[h] = false;
    }
  }
}

// Output visual and audible signals to indicate the heating process is almost complete.
void TDTSandwich::handleCountDown_()
{
  if (heatEndTime_ - millis() <= countdownTickCounter_ * countdownTickDuration_)
  {// Time to turn on the alarm system and tick off one from the counter.
    // Due to decreased countdownTickCounter_, subsequent calls to this function will
    // fall thru to the next code block (at least until the current time has reached
    // the next countdown tick)
    activateFlasher_();
    activateBuzzer_();
    countdownTickCounter_--;
    countdownTickComplete_ = false;
  }
  else if (!countdownTickComplete_ && heatEndTime_ - millis() <= countdownTickCounter_ * countdownTickDuration_ + alertOffDuration_)
  {// Time to turn off flasher
    // Notice how the timing check for condition accounts for the decrement in countdownTickCounter_ after the alarm system was turned on
    // by adding alertOffDuration_. This moved the time threshold back into the current countdown tick.
    deactivateFlasher_();
    deactivateBuzzer_();
    countdownTickComplete_ = true;

    if (countdownTickCounter_ == 0)
    {// Set the flags for signalling the end of the heating process
      countdownFinishing_ = true;
      countdownActive_ = false;
      countdownFinishedAlertActive_ = false;
    }
  }
}

// Visually and audibly alert user that the heating is complete, and tells the computer too
void TDTSandwich::signalHeatEnd_()
{
  // countdownEndBuffer_ aims to "push" the end time to a number further higher than the actual end time. This allows
  // proper subtraction of the unsigned longs. If we just relied on (heatEndTime_ - millis()), the moment millis exceeds
  // heatEndTime_, the subtraction blows up close to the max limit of unsigned long due to the unsigned nature of it.
  // By artificially pushing the end time ahead of the actual one, we can safely compare durations, unless the system
  // somehow hangs and did not perform this comparison until it's too late.
  if (!countdownFinishedAlertActive_ && heatEndTime_ + countdownEndBuffer_ - millis() <= countdownEndBuffer_)
  {// Signal the end of heating process and turn off heaters
    activateFlasher_();
    activateBuzzer_();
    stopHeat(false);
    countdownFinishedAlertActive_ = true;

    // Tell computer that heating is complete
    communicator_.sendHeatingDone();
  }
  else if (countdownFinishedAlertActive_ && heatEndTime_ + countdownEndBuffer_ - millis() <= countdownEndBuffer_ - heatEndAlertDuration_)
  {// Time to turn off the alert system
    deactivateFlasher_();
    deactivateBuzzer_();
    countdownFinishedAlertActive_ = false;

    // Stop calls to this function
    countdownFinishing_ = false;
  }
}

// Blink the alarm LED on request
void TDTSandwich::handleBlinking_()
{
  if (blinkEndTime_ - millis() <=  blinkCounter_ * blinkDuration_)
  {// Time to turn on the flasher and tick off one from the counter.
    // Due to decreased blinkCounter, subsequent calls to this function will
    // fall thru to the next code block (at least until the current time has reached
    // the next blinking period)
    activateFlasher_();
    blinkCounter_--;
    blinkPeriodComplete_ = false;
  }
  else if (!blinkPeriodComplete_ && blinkEndTime_ - millis() <= blinkCounter_ * blinkDuration_ + blinkOffDuration_)
  {// Time to turn off flasher
    // Notice how the timing check for condition accounts for the decrement in blinkCounter_ after the flasher was turned on
    // by adding blinkOffDuration_. This moved the time threshold back into the current period.
    deactivateFlasher_();
    blinkPeriodComplete_ = true;

    if (blinkCounter_ <= 0)
    {// No more blinks remaining
      flasherBlinking_ = false;
    }
  }
}

void TDTSandwich::activateHeatLED_()
{
  digitalWrite(heatLEDPin_, HIGH);
}

void TDTSandwich::deactivateHeatLED_()
{
  digitalWrite(heatLEDPin_, LOW);
}

void TDTSandwich::activateHeater_(uint8_t heaterIndex)
{
  digitalWrite(heaterSSRPin_[heaterIndex], HIGH);
}

void TDTSandwich::deactivateHeater_(uint8_t heaterIndex)
{
  digitalWrite(heaterSSRPin_[heaterIndex], LOW);
}

void TDTSandwich::activateFlasher_()
{
  digitalWrite(flasherPin_, HIGH);
}

void TDTSandwich::deactivateFlasher_()
{
  digitalWrite(flasherPin_, LOW);
}

void TDTSandwich::activateBuzzer_()
{
  digitalWrite(buzzerPin_, HIGH);
}

void TDTSandwich::deactivateBuzzer_()
{
  digitalWrite(buzzerPin_, LOW);
}


void TDTSandwich::handleTCAmplifierFault(MAX31856* TCAmplifier, bool heaterTCAmplifier, uint8_t index)
{
  uint8_t faultRegister = TCAmplifier->getFault();
  Device faultingDevice;

  if (heaterTCAmplifier)
  {
    switch (index)
    {
    case 0:
      faultingDevice = DEVICE_HEATER1TCAMPLIFIER;
      break;
    case 1:
      faultingDevice = DEVICE_HEATER2TCAMPLIFIER;
      break;
    default:
      faultingDevice = DEVICE_UNKNOWN;
      break;
    }
  }
  else
  {
    faultingDevice = DEVICE_SAMPLETCAMPLIFIER;
  }

  if (faultRegister)
  {// There could be one or more errors. Therefore, check for every fault and send separate error messages.
    if (faultRegister & MAX31856::MAX31856_FAULT_CJRANGE)
    {
      communicator_.sendError(faultingDevice, ERR_TCAMPLIFIER_CJRANGE);
    }

    if (faultRegister & MAX31856::MAX31856_FAULT_TCRANGE)
    {
      communicator_.sendError(faultingDevice, ERR_TCAMPLIFIER_TCRANGE);
    }

    if (faultRegister & MAX31856::MAX31856_FAULT_CJHIGH)
    {
      communicator_.sendError(faultingDevice, ERR_TCAMPLIFIER_CJHIGH);
    }

    if (faultRegister & MAX31856::MAX31856_FAULT_CJLOW)
    {
      communicator_.sendError(faultingDevice, ERR_TCAMPLIFIER_CJLOW);
    }

    if (faultRegister & MAX31856::MAX31856_FAULT_TCHIGH)
    {
      communicator_.sendError(faultingDevice, ERR_TCAMPLIFIER_TCHIGH);
    }

    if (faultRegister & MAX31856::MAX31856_FAULT_TCLOW)
    {
      communicator_.sendError(faultingDevice, ERR_TCAMPLIFIER_TCLOW);
    }

    if (faultRegister & MAX31856::MAX31856_FAULT_OVUV)
    {
      communicator_.sendError(faultingDevice, ERR_TCAMPLIFIER_OVUV);
    }

    if (faultRegister & MAX31856::MAX31856_FAULT_OPEN)
    {
      communicator_.sendError(faultingDevice, ERR_TCAMPLIFIER_OPEN);
    }
  }
  else
  {// The chip signalled a fault, yet none were detected on the fault status register...
    communicator_.sendError(faultingDevice, ERR_TCAMPLIFIER_UNKNOWN);
  }
}

