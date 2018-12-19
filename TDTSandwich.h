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

#include <PID_modified.h>
#include <MAX31856.h>
#include <SerialCommunication.h>

typedef enum
{
  DEVICE_ARDUINO            = 0,  // The microcontroller itself
  DEVICE_HEATER1TCAMPLIFIER = 1,  // TC amplifier for thermocouple on heater 1
  DEVICE_HEATER2TCAMPLIFIER = 2,  // TC amplifier for thermocouple on heater 2
  DEVICE_SAMPLETCAMPLIFIER  = 3,  // TC amplifier for sample thermocouple
  DEVICE_UNKNOWN            = 4,  // Only used when the code is not written properly to account for all devices
} Device;

class TDTSandwich {
public:
  TDTSandwich(SerialCommunication& communicator,
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
              const int8_t& maxCJTemperature);
  ~TDTSandwich();
  void init();
  void run();

  // Begin measuring temperature using continuous mode of MAX31856. Sets the thermocouple types.
  void startDAQ(MAX31856_TCType TCType, MAX31856_SampleAveraging averageSampleCount, bool measureSample);
  void stopDAQ();

  void startHeat(double setpoint, double heatingRate, unsigned long heatDuration, double Kp, double Ki);
  void stopHeat(bool deactivateAlert);

  void blinkFlasher();


private:
  // List of error codes
  // Device: DEVICE_HEATER1TCAMPLIFIER, DEVICE_HEATER2TCAMPLIFIER, and DEVICE_SAMPLETCAMPLIFIER
  // Notes: These are all defined in p26 of MAX31856 datasheet
  static const uint8_t ERR_TCAMPLIFIER_CJRANGE  = 0;
  static const uint8_t ERR_TCAMPLIFIER_TCRANGE  = 1;
  static const uint8_t ERR_TCAMPLIFIER_CJHIGH   = 2;
  static const uint8_t ERR_TCAMPLIFIER_CJLOW    = 3;
  static const uint8_t ERR_TCAMPLIFIER_TCHIGH   = 4;
  static const uint8_t ERR_TCAMPLIFIER_TCLOW    = 5;
  static const uint8_t ERR_TCAMPLIFIER_OVUV     = 6;
  static const uint8_t ERR_TCAMPLIFIER_OPEN     = 7;
  static const uint8_t ERR_TCAMPLIFIER_UNKNOWN  = 8;  // MAX31856 signalled a FAULT, but had nothing on the fault status registering explaining the fault

  // Number of heaters
  static const uint8_t heaterCount_ = 2;

  // Minimum threshold times for heater to be on or off
  // TODO adjust accordingly
  static const uint8_t HEATER_ON_MIN = 0;
  static const uint8_t HEATER_OFF_MIN = 17;

  // Pinout
  const uint8_t heatLEDPin_;
  const uint8_t flasherPin_;
  const uint8_t buzzerPin_;
  uint8_t heaterSSRPin_[heaterCount_];

  SerialCommunication communicator_;

  PID heaterPID_[heaterCount_];
  MAX31856* heaterTCAmplifier_[heaterCount_];
  MAX31856* sampleTCAmplifier_;

  // Status flags
  bool DAQHeaterActive_;
  bool DAQSampleActive_;
  bool heatActive_;
  bool flasherBlinking_;

  // Heater thermocouple and heater
  bool heaterTCOK_[heaterCount_];
  bool freshTemperatureReading_[heaterCount_];
  bool heaterOn_[heaterCount_];
  bool heatOffMinSatisfied[heaterCount_];
  double heaterTemperature_[heaterCount_];
  double heaterControlOutput_[heaterCount_];
  double heaterSetpoint_;
  double heaterTempSetpoint_[heaterCount_];
  double setpointAdjustment_;
  unsigned long heatOnTime_[heaterCount_];
  unsigned long heatOnDuration_[heaterCount_];
  unsigned long heaterPrevPIDPeriodStartTime_[heaterCount_];
  unsigned long heatEndTime_;
  const double minHeaterTemperature_;
  const double maxHeaterTemperature_;
  const int8_t minCJTemperature_; // Shared with sample TC
  const int8_t maxCJTemperature_; // Shared with sample TC

  // Heating rate and setpoint adjustment
  const unsigned long setpointAdjustmentInterval_ = 500;          // Adjust the setpoint every 500 ms
  const unsigned long setpointAdjustmentIntervalBuffer_ = 100000;  // A time buffer to solve problems of "negative" numbers coming from unsigned long subtraction
  bool allSetpointReached_;
  bool setpointReached_[heaterCount_];
  unsigned long nextSetpointAdjustmentTime_;
  const double minSampleTemperature_;
  const double maxSampleTemperature_;
  const double PIDLimit_ = 100; // The max and min (apply a negative) limits for output of the PID algorithm


  // Sample thermocouple
  bool sampleTCOK_;
  double sampleTemperature_;

  // Flasher
  const uint8_t blinkTotal_ = 8;    // Number of times to blink
  const unsigned long blinkOnDuration_ = 250;   // During blinking, LED turns on for 0.5 s, followed by...
  const unsigned long blinkOffDuration_ = 250;  // ...an off duration of 0.5 s
  const unsigned long blinkDuration_ = blinkOnDuration_ + blinkOffDuration_;  // Total duration for a single blink
  bool blinkPeriodComplete_;
  uint8_t blinkCounter_;
  unsigned long blinkEndTime_;

  // Countdown
  const uint8_t countdownTickTotal_           = 10;                                         // Number of countdown ticks
  const unsigned long countdownTickDuration_  = 1000;                                       // Total duration a single countdown tick (1 s)
  const unsigned long alertOnDuration_        = 200;                                        // During blinking, LED turns on for 0.5 s, followed by...
  const unsigned long alertOffDuration_       = countdownTickDuration_ - alertOnDuration_;  // ...an off duration of 0.5 s
  const unsigned long heatEndAlertDuration_   = 2000;                                       // Duration for turning on alert at the end of the heating to signal end of heating
  const unsigned long countdownEndBuffer_     = 10 * heatEndAlertDuration_;                 // A time buffer to solve problems of "negative" numbers coming from unsigned long subtraction
  bool countdownTickComplete_;
  bool countdownActive_;
  bool countdownFinishing_;
  bool countdownFinishedAlertActive_;
  uint8_t countdownTickCounter_;
  unsigned long countdownTickStartTime_;


  void getTemperatureReading_();
  void adjustTempSetpoint_();
  void handleHeating_();
  void handleCountDown_();
  void signalHeatEnd_();
  void handleTCAmplifierFault(MAX31856* TCAmplifier, bool heaterTCAmplifier, uint8_t index);
  void handleBlinking_();

  // Outputs
  void activateHeatLED_();
  void deactivateHeatLED_();
  void activateHeater_(uint8_t heaterIndex);
  void deactivateHeater_(uint8_t heaterIndex);
  void activateFlasher_();
  void deactivateFlasher_();
  void activateBuzzer_();
  void deactivateBuzzer_();
};
