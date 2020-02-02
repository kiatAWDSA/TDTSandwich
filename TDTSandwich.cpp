#include "TDTSandwich.h"

TDTSandwich::TDTSandwich( SerialCommunication& communicator,
                          const uint8_t heaterSSRPin[],
                          const uint8_t heaterCSPin[],
                          const uint8_t heaterDRDYPin[],
                          const uint8_t heaterFaultPin[],
                          const uint8_t& expLatchPin,
                          const uint8_t& expOEPin,
                          const uint8_t& addLatchPin,
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
                          expLatchPin_(expLatchPin),
                          expOEPin_(expOEPin),
                          addLatchPin_(addLatchPin),
                          minHeaterTemperature_(minHeaterTemperature),
                          maxHeaterTemperature_(maxHeaterTemperature),
                          minSampleTemperature_(minSampleTemperature),
                          maxSampleTemperature_(maxSampleTemperature),
                          minCJTemperature_(minCJTemperature),
                          maxCJTemperature_(maxCJTemperature)
{

  // Initialize pins
  // Don't initialize OE pin for the expansion shift register yet; do that only in init function
  pinMode(expLatchPin, OUTPUT);
  pinMode(sampleCSPin, OUTPUT);
  pinMode(addLatchPin, OUTPUT);
  pinMode(sampleDRDYPin, INPUT);
  pinMode(sampleFaultPin, INPUT);

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
    heaterPID_[h].SetOutputLimits(PIDLowerLimit_, PIDUpperLimit_);
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
  // Initialize the expansion shift register, with LOW on all outputs
  expShiftRegState = 0; // LOW on all outputs
  activateAddOE_(); // Disable output of address shift register for now.
  digitalWrite(expOEPin_, LOW); // Enable outputs
  pinMode(expOEPin_, OUTPUT);

  // Get sandwich ID and display it
  refreshID();

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
    }

    // This is placed outside of check for heatActive_ because signalHeatEnd_() itself stops heat, and still needs to be called after heatActive_ is set to false.
    if (countdownFinishing_)
    {
      signalHeatEnd_();
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
    heaterTAcquired_[h] = false;
    heaterTFreshReading[h] = false;
  }

  DAQHeaterActive_ = true;
  haltSendingT_ = false;

  // Do the same for sample thermocouples, if needed
  if (measureSample)
  {
    sampleTCAmplifier_->setThermocoupleTypeAndOversampling(TCType, averageSampleCount);
    sampleTCAmplifier_->startAutoConversion();
    sampleTCOK_ = true;
    sampleTAcquired_ = false;
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

      // Set up and calculate parameters for PID
      reachedTargetThreshold_ = false;
      KpGiven_                = Kp;
      KiGiven_                = Ki;
      calcAndSetPIDTunings();
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

// Get sandwich ID and display it
void TDTSandwich::refreshID()
{
  // Prepare to grab the sandwich ID
  activateTransceiverOE_(); // Put the 3.3 V -> 5 V transceiver to high impedance output so that its 5 V MISO output doesn't disturb the MISO output of the address shift register
  activateAddLatch_();  // Latch input data
  deactivateAddLatch_();
  deactivateAddOE_();

  // SH/LD was previously low. Pull high to transfer latched data into storage register and prepare for serial shifting.
  // THIS MUST BE DONE just before shifting data out of the register, so that the data in the storage register is not
  // accidentally shifted out by calls to other functions that uses the SPI line.
  activateAddSHLD_();

  // Grab the sandwich ID
  SPI.beginTransaction(HC589A_spisettings);
  id = SPI.transfer(0);  // Send dummy data. We only need the response from the address shift register.
  if (id > ID_MAX)
  {// Limit the ID to prevent weird display on the 7-segment display
    id = ID_MAX;
  }
  SPI.endTransaction();
  deactivateAddSHLD_();
  activateAddOE_(); // Change output to high impedance
  deactivateTransceiverOE_(); // Re-enable the transceiver outputs

  // Display the sandwich ID on the 7-segment displays
  uint8_t digit1 = id / 10;
  uint8_t digit2 = id % 10;
  SPI.beginTransaction(HC595_spisettings);
  SPI.transfer(getSegmentDisplayCode(SEG2_MASKS, digit2));
  SPI.transfer(getSegmentDisplayCode(SEG1_MASKS, digit1));
  SPI.endTransaction();
  activateSegLatch_();
  deactivateSegLatch_();
}

// Stop all sandwich operations
void TDTSandwich::shutdown()
{
  if (DAQHeaterActive_ || DAQSampleActive_)
  {
    // Heat is stopped by calling stopDAQ too.
    stopDAQ();
  }
}

// Fetch and store temperature readings from all thermocouple amplifiers,
// provided that they are available and have no faults.
void TDTSandwich::getTemperatureReading_()
{
  // Acquire heater thermocouple readings
  for (uint8_t h = 0; h < heaterCount_; h++)
  {
    if (!heaterTAcquired_[h])
    {// Need to get the heater temperature reading.
      if (heaterTCAmplifier_[h]->isFaultless())
      {// The heater thermocouple amplifier did not detect any errors
        heaterTCOK_[h] = true;

        if (heaterTCAmplifier_[h]->dataIsReady())
        {// There is new temperature data available
          heaterTemperature_[h] = heaterTCAmplifier_[h]->readTCTemperature();
          heaterTAcquired_[h] = true;
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
          // Otherwise, do nothing until the fault is cleared which would trigger
          // the previous code block
          heaterTCOK_[h] = false;
          heaterTAcquired_[h] = false;
          handleTCAmplifierFault(heaterTCAmplifier_[h], true, h);
        }
      }
    }
  }
  
  // Acquire sample thermocouple readings
  if (DAQSampleActive_)
  {// Only check for sample thermocouple readings if we are measuring sample temperature
    if (!sampleTAcquired_)
    {// Need to get the sample temperature reading.
      if (sampleTCAmplifier_->isFaultless())
      {// The sample thermocouple amplifier did not detect any errors
        sampleTCOK_ = true;

        if (sampleTCAmplifier_->dataIsReady())
        {// There is new temperature data available
          sampleTemperature_ = sampleTCAmplifier_->readTCTemperature();
          sampleTAcquired_ = true;

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
          sampleTAcquired_ = false;
          handleTCAmplifierFault(sampleTCAmplifier_, false, 0);
        }
      }
    }

    // Send readings to computer once all readings/errors have been collected before preparing for next acquisition cycle.
    // Only send data to computer if ALL (including sample, if it is active) thermocouples obey one of the following rules:
    // a) TReadySend_ AND TCOK    : new temperature reading available, and the TC amplifier is OK
    // b) !TReadySend_ AND !TCOK  : there was an error with the TC amplifier.
    // Sending all data in one stream instead of separately saves some work, though the time of the measurements as received by the computer will be
    // slightly different than the actual time at which each measurement was taken.
    if (    ((heaterTAcquired_[0] && heaterTCOK_[0])  ||  (!heaterTAcquired_[0] && !heaterTCOK_[0]))
        &&  ((heaterTAcquired_[1] && heaterTCOK_[1])  ||  (!heaterTAcquired_[1] && !heaterTCOK_[1]))
        &&  ((sampleTAcquired_    && sampleTCOK_)     ||  (!sampleTAcquired_    && !sampleTCOK_)))
    {// Send temperature reading/error notice to computer

      if (haltSendingT_ && (heaterTCOK_[0] || heaterTCOK_[1] || sampleTCOK_))
      {// We previously halted sending data because all three thermocouples had error, but are resuming now because at least one thermocouple has recovered.
        haltSendingT_ = false;
      }
      
      if (!haltSendingT_)
      {// Only send data if it hasn't be halted due to all thermocouples having errors.
        communicator_.sendHeaterSampleTemperature(heaterTCOK_[0], heaterTemperature_[0], heaterTCOK_[1], heaterTemperature_[1], sampleTCOK_, sampleTemperature_);
      }

      if (!heaterTCOK_[0] && !heaterTCOK_[1] && !sampleTCOK_)
      {// If all thermocouples have error, stop sending data lines packed with errors. Sending will resume based on the checks previous to this.
        haltSendingT_ = true;
      }

      // Prepare for new readings
      heaterTAcquired_[0] = false;
      heaterTAcquired_[1] = false;
      sampleTAcquired_ = false;
    }
  }
  else
  {// Not reading temperature of sample; send only heater T readings.
   // Send readings to computer once all readings/errors have been collected before preparing for next acquisition cycle.
   // Only send data to computer if ALL (including sample, if it is active) thermocouples obey one of the following rules:
   // a) TReadySend_ AND TCOK    : new temperature reading available, and the TC amplifier is OK
   // b) !TReadySend_ AND !TCOK  : there was an error with the TC amplifier.
   // Sending all data in one stream instead of separately saves some work, though the time of the measurements as received by the computer will be
   // slightly different than the actual time at which each measurement was taken.
    if (    ((heaterTAcquired_[0] && heaterTCOK_[0]) || (!heaterTAcquired_[0] && !heaterTCOK_[0]))
        &&  ((heaterTAcquired_[1] && heaterTCOK_[1]) || (!heaterTAcquired_[1] && !heaterTCOK_[1])))
    {// Send temperature reading/error notice to computer

      if (haltSendingT_ && (heaterTCOK_[0] || heaterTCOK_[1]))
      {// We previously halted sending data because all three thermocouples had error, but are resuming now because at least one thermocouple has recovered.
        haltSendingT_ = false;
      }

      if (!haltSendingT_)
      {// Only send data if it hasn't be halted due to all thermocouples having errors.
        communicator_.sendHeaterTemperature(heaterTCOK_[0], heaterTemperature_[0], heaterTCOK_[1], heaterTemperature_[1]);
      }

      if (!heaterTCOK_[0] && !heaterTCOK_[1])
      {// If all thermocouples have error, stop sending data lines packed with errors. Sending will resume based on the checks previous to this.
        haltSendingT_ = true;
      }

      // Prepare for new readings
      heaterTAcquired_[0] = false;
      heaterTAcquired_[1] = false;
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
        calcAndSetPIDTunings();
        heaterPID_[h].Compute(curTime);
        /*
        Serial.print("Output ");
        Serial.print(h);
        Serial.print(": ");
        Serial.println(heaterControlOutput_[h]);
         */
        if (heaterControlOutput_[h] > HEATER_ON_MIN)
        {// Outside of minimum output threshold; The threshold exists so that if the on time is extremely small,
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
          heatOnDuration_[h] = constrain(heaterControlOutput_[h], HEATER_ON_MIN, PIDDutyUpperLimit_) / PIDDutyUpperLimit_ * PIDPeriod;
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

// Calculate the PID tunings (specifically, just the Kp) based on current temperature readings
// and update the PID class with the new values.
// NOTE: The function assumes that we are HEATING i.e. heater setpoint is higher than heater T readings.
// If the heater setpoint is set lower than the heater T reading, then we are simply stuck at Zone 3.
void TDTSandwich::calcAndSetPIDTunings()
{
  for (uint8_t h = 0; h < heaterCount_; h++)
  {
    if (!reachedTargetThreshold_ && heaterTemperature_[h] >= heaterSetpoint_ - KP_MAINTAIN_TRESHOLD_SIZE)
    {
      reachedTargetThreshold_ = true;
    }
    
    if (reachedTargetThreshold_)
    {
      Kp_ = KP_MAINTAIN;
    }
    else
    {
      Kp_ = KpGiven_;
    }

    if (heaterTemperature_[h] >= heaterSetpoint_ - KI_MAINTAIN_TRESHOLD_SIZE)
    {
      Ki_ = KI_MAINTAIN;
    }
    else
    {
      Ki_ = KiGiven_;
    }
    
    // Leave Kd as 0 since we are using a PI control.
    heaterPID_[h].SetTunings(Kp_, Ki_/1000, 0);
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

uint8_t TDTSandwich::getSegmentDisplayCode(const uint8_t segmentDisplayMasks[7], uint8_t displayedNumber)
{
  uint8_t displayCode = 0;
  switch (displayedNumber)
  {
  case 0:
    // Activate all segments except G 
    displayCode = 255;
    displayCode &= ~segmentDisplayMasks[6];
    break;
  case 1:
    // Activate segments B and C
    displayCode |= segmentDisplayMasks[1];
    displayCode |= segmentDisplayMasks[2];
    break;
  case 2:
    // Activate segments A, B, G, E and D
    displayCode |= segmentDisplayMasks[0];
    displayCode |= segmentDisplayMasks[1];
    displayCode |= segmentDisplayMasks[6];
    displayCode |= segmentDisplayMasks[4];
    displayCode |= segmentDisplayMasks[3];
    break;
  case 3:
    // Activate segments A, B, G, C and D
    displayCode |= segmentDisplayMasks[0];
    displayCode |= segmentDisplayMasks[1];
    displayCode |= segmentDisplayMasks[6];
    displayCode |= segmentDisplayMasks[2];
    displayCode |= segmentDisplayMasks[3];
    break;
  case 4:
    // Activate segments F, G, B and C
    displayCode |= segmentDisplayMasks[5];
    displayCode |= segmentDisplayMasks[6];
    displayCode |= segmentDisplayMasks[1];
    displayCode |= segmentDisplayMasks[2];
    break;
  case 5:
    // Activate segments A, F, G, C and D
    displayCode |= segmentDisplayMasks[0];
    displayCode |= segmentDisplayMasks[5];
    displayCode |= segmentDisplayMasks[6];
    displayCode |= segmentDisplayMasks[2];
    displayCode |= segmentDisplayMasks[3];
    break;
  case 6:
    // Activate all segments except B
    displayCode = 255;
    displayCode &= ~segmentDisplayMasks[1];
    break;
  case 7:
    // Activate segments A, B and C
    displayCode |= segmentDisplayMasks[0];
    displayCode |= segmentDisplayMasks[1];
    displayCode |= segmentDisplayMasks[2];
    break;
  case 8:
    // Activate all segments 
    displayCode = 255;
    break;
  case 9:
    // Activate all segments except E 
    displayCode = 255;
    displayCode &= ~segmentDisplayMasks[4];
    break;
  default:
    // Don't display anything at all
    displayCode = 0;
    break;
  }

  return displayCode;
}

void TDTSandwich::SPITransferExp(uint8_t byte)
{
  SPI.beginTransaction(HC595_spisettings);
  SPI.transfer(byte);
  SPI.endTransaction();

  // Transfer storage register to output
  digitalWrite(expLatchPin_, HIGH);
  digitalWrite(expLatchPin_, LOW);
}

void TDTSandwich::activateHeatLED_()
{
  expShiftRegState |= EXP_MASK_HEATLED;
  SPITransferExp(expShiftRegState);
}

void TDTSandwich::deactivateHeatLED_()
{
  expShiftRegState &= ~EXP_MASK_HEATLED;
  SPITransferExp(expShiftRegState);
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
  expShiftRegState |= EXP_MASK_FLASHER;
  SPITransferExp(expShiftRegState);
}

void TDTSandwich::deactivateFlasher_()
{
  expShiftRegState &= ~EXP_MASK_FLASHER;
  SPITransferExp(expShiftRegState);
}

void TDTSandwich::activateBuzzer_()
{
  expShiftRegState |= EXP_MASK_BUZZER;
  SPITransferExp(expShiftRegState);
}

void TDTSandwich::deactivateBuzzer_()
{
  expShiftRegState &= ~EXP_MASK_BUZZER;
  SPITransferExp(expShiftRegState);
}

void TDTSandwich::activateSegLatch_()
{
  digitalWrite(addLatchPin_, HIGH);
}

void TDTSandwich::deactivateSegLatch_()
{
  digitalWrite(addLatchPin_, LOW);
}

void TDTSandwich::activateAddOE_()
{
  expShiftRegState |= EXP_MASK_ADD_OE;
  SPITransferExp(expShiftRegState);
}

void TDTSandwich::deactivateAddOE_()
{
  expShiftRegState &= ~EXP_MASK_ADD_OE;
  SPITransferExp(expShiftRegState);
}

void TDTSandwich::activateAddLatch_()
{
  expShiftRegState |= EXP_MASK_ADD_LATCH;
  SPITransferExp(expShiftRegState);
}

void TDTSandwich::deactivateAddLatch_()
{
  expShiftRegState &= ~EXP_MASK_ADD_LATCH;
  SPITransferExp(expShiftRegState);
}

void TDTSandwich::activateAddSHLD_()
{
  expShiftRegState |= EXP_MASK_ADD_SHLD;
  SPITransferExp(expShiftRegState);
}

void TDTSandwich::deactivateAddSHLD_()
{
  expShiftRegState &= ~EXP_MASK_ADD_SHLD;
  SPITransferExp(expShiftRegState);
}

void TDTSandwich::activateTransceiverOE_()
{
  expShiftRegState |= EXP_MASK_TCVR_OE;
  SPITransferExp(expShiftRegState);
}

void TDTSandwich::deactivateTransceiverOE_()
{
  expShiftRegState &= ~EXP_MASK_TCVR_OE;
  SPITransferExp(expShiftRegState);
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

