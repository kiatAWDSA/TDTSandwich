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
              const int8_t& maxCJTemperature);
  ~TDTSandwich();
  void init();
  void run();

  // Sandwich id
  uint8_t id;

  // Commands given to the sandwich
  void startDAQ(MAX31856_TCType TCType, MAX31856_SampleAveraging averageSampleCount, bool measureSample);
  void stopDAQ();
  void startHeat(double setpoint, double heatingRate, unsigned long heatDuration, double Kp, double Ki);
  void stopHeat(bool deactivateAlert);
  void blinkFlasher();
  void refreshID();
  void shutdown();


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

  // Binary masks for pin expansion shift register
  static const uint8_t EXP_MASK_BUZZER    = 0x02;
  static const uint8_t EXP_MASK_HEATLED   = 0x04;
  static const uint8_t EXP_MASK_TCVR_OE   = 0x08;
  static const uint8_t EXP_MASK_ADD_OE    = 0x10;
  static const uint8_t EXP_MASK_ADD_LATCH = 0x20;
  static const uint8_t EXP_MASK_ADD_SHLD  = 0x40;
  static const uint8_t EXP_MASK_FLASHER   = 0x80;

  // Binary masks for 7-segment display shift register, digit 1
  static const uint8_t SEG1_MASK_A  = 0x40;
  static const uint8_t SEG1_MASK_B  = 0x08;
  static const uint8_t SEG1_MASK_C  = 0x02;
  static const uint8_t SEG1_MASK_D  = 0x10;
  static const uint8_t SEG1_MASK_E  = 0x04;
  static const uint8_t SEG1_MASK_F  = 0x20;
  static const uint8_t SEG1_MASK_G  = 0x80;
  const uint8_t SEG1_MASKS[7] = { SEG1_MASK_A, SEG1_MASK_B, SEG1_MASK_C, SEG1_MASK_D, SEG1_MASK_E, SEG1_MASK_F, SEG1_MASK_G };

  // Binary masks for 7-segment display shift register, digit 2
  static const uint8_t SEG2_MASK_A = 0x40;
  static const uint8_t SEG2_MASK_B = 0x04;
  static const uint8_t SEG2_MASK_C = 0x08;
  static const uint8_t SEG2_MASK_D = 0x10;
  static const uint8_t SEG2_MASK_E = 0x20;
  static const uint8_t SEG2_MASK_F = 0x02;
  static const uint8_t SEG2_MASK_G = 0x80;
  const uint8_t SEG2_MASKS[7] = { SEG2_MASK_A, SEG2_MASK_B, SEG2_MASK_C, SEG2_MASK_D, SEG2_MASK_E, SEG2_MASK_F, SEG2_MASK_G };

  // Shift registers
  // 74HC595 datasheet: http://static6.arrow.com/aropdfconversion/d729a1ab5ce8ea9b05a5d0c48e79381a3cfd274b/40523416659797374hc_hct595.pdf
  // p10 of 74HC595 datasheet: Serial clock max frequency ranges between 30 and 91 MHz
  // p12 of MAX31856 datasheet: Data is sampled at trailing edge of clock pulse
  // MC74HC589A datasheet: http://static6.arrow.com/aropdfconversion/e2e85bd629a1a7d506fd49bee5e55ef3980ec3d9/66mc74hc589a-d.pdf
  // p4 of MC74HC589A datasheet: Serial clock max frequency is 30 MHz.
  // p6 of MC74HC589A datasheet, Fig. 3: Data is sampled at trailing edge of clock pulse
  // The SPI code would automatically choose a clock divider such that the SPI speed is less or equal to the number given.
  // See https://dorkbotpdx.org/blog/paul/spi_transactions_in_arduino
  const SPISettings HC595_spisettings = SPISettings(20000000, MSBFIRST, SPI_MODE0);
  const SPISettings HC589A_spisettings = SPISettings(20000000, LSBFIRST, SPI_MODE0);  // Due to wiring mistake, get the address LSB first
  uint8_t expShiftRegState;

  // Pins
  uint8_t heaterSSRPin_[heaterCount_];
  const uint8_t expLatchPin_;
  const uint8_t expOEPin_;
  const uint8_t addLatchPin_;

  // Other classes
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
  bool heaterTReadySend_[heaterCount_];
  bool heaterTFreshReading[heaterCount_];
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
  const double PIDLowerLimit_ = -4; // The lower limit for output of the PID algorithm
  const double PIDUpperLimit_ = 250; // The upper limit for output of the PID algorithm
  const double PIDDutyUpperLimit_ = 100; // The upper limit for actual heater duty cycle

  // Calculation of Kp and Ki in the PID algorithm, proportional on measurement
  bool reachedTargetThreshold_;
  const double KP_MAINTAIN = 5; // The value that Kp will be forced to when the heater temperature has reached a threshold from the setpoint.
  const double KP_MAINTAIN_TRESHOLD_SIZE = 0;
  const double KI_MAINTAIN = 0.8; // The value that Ki will be forced to when the heater temperature has reached a threshold from the setpoint.
  const double KI_MAINTAIN_TRESHOLD_SIZE = 0.3;
  double Kp_;
  double KpGiven_;
  double Ki_;
  double KiGiven_;
  double KiMaintainThreshold_;

  // Sample thermocouple
  bool sampleTCOK_;
  bool sampleTReadySend_;
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
  const unsigned long countdownEndBuffer_     = 20 * heatEndAlertDuration_;                 // A time buffer to solve problems of "negative" numbers coming from unsigned long subtraction
  bool countdownTickComplete_;
  bool countdownActive_;
  bool countdownFinishing_;
  bool countdownFinishedAlertActive_;
  uint8_t countdownTickCounter_;
  unsigned long countdownTickStartTime_;

  // Internal functions
  void getTemperatureReading_();
  void adjustTempSetpoint_();
  void handleHeating_();
  void calcAndSetPIDTunings();
  void handleCountDown_();
  void signalHeatEnd_();
  void handleTCAmplifierFault(MAX31856* TCAmplifier, bool heaterTCAmplifier, uint8_t index);
  void handleBlinking_();
  uint8_t getSegmentDisplayCode(const uint8_t segmentDisplayMasks[7], uint8_t displayedNumber);

  // Outputs
  void SPITransferExp(uint8_t byte);
  void activateHeatLED_();
  void deactivateHeatLED_();
  void activateHeater_(uint8_t heaterIndex);
  void deactivateHeater_(uint8_t heaterIndex);
  void activateFlasher_();
  void deactivateFlasher_();
  void activateBuzzer_();
  void deactivateBuzzer_();
  void activateSegLatch_();
  void deactivateSegLatch_();
  void activateAddOE_();
  void deactivateAddOE_();
  void activateAddLatch_();
  void deactivateAddLatch_();
  void activateAddSHLD_();
  void deactivateAddSHLD_();
  void activateTransceiverOE_();
  void deactivateTransceiverOE_();
};
