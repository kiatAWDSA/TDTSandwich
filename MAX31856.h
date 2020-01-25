/*************************************************** 
  This is a library for the Adafruit Thermocouple Sensor w/MAX31855K

  Designed specifically to work with the Adafruit Thermocouple Sensor
  ----> https://www.adafruit.com/products/269

  These displays use SPI to communicate, 3 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  Modified by Soon Kiat Lau 2017
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#ifndef ADAFRUIT_MAX31856_H
#define ADAFRUIT_MAX31856_H

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <stdlib.h>
#include <SPI.h>


typedef enum
{
  TCTYPE_B  = 0b0000,
  TCTYPE_E  = 0b0001,
  TCTYPE_J  = 0b0010,
  TCTYPE_K  = 0b0011,
  TCTYPE_N  = 0b0100,
  TCTYPE_R  = 0b0101,
  TCTYPE_S  = 0b0110,
  TCTYPE_T  = 0b0111,
  VMODE_G8  = 0b1000,
  VMODE_G32 = 0b1100
} MAX31856_TCType;

typedef enum
{
  /***********************************************************************************
  If desired, the MAX31856 can be told to average across multiple conversions before
  giving out the final temperature value. This helps in reducing noise, but increases
  the delay between each temperature reading.

  The delay calculation is given below:

  p20 of datasheet:
    Typical conversion times:
    1-shot or first conversion in Auto mode:
      = tCONV + (samples -1) x 33.33mS (60Hz rejection)
      = tCONV + (samples -1) x 40mS (50Hz rejection)
    2 thru n conversions in Auto mode
      = tCONV + (samples -1) x 16.67mS (60Hz rejection)
      = tCONV + (samples -1) x 20mS (50Hz rejection)

  p4 of datasheet:
    tCONV duration:
      1-Shot conversion or first conversion in auto-conversion mode (60Hz): 143 ms
      1-Shot conversion or first conversion in auto-conversion mode (50Hz): 169 ms
      Auto conversion mode, conversions 2 through n (60Hz):                 82 ms
      Auto conversion mode, conversions 2 through n (50Hz):                 98 ms
  ***********************************************************************************/
  AVERAGE_1  = 0x00,
  AVERAGE_2  = 0x10,
  AVERAGE_4  = 0x20,
  AVERAGE_8  = 0x30,
  AVERAGE_16 = 0x40
} MAX31856_SampleAveraging;

typedef enum
{
  // Modes for open-circuit detection; Table 4 at pg 14 of datasheet
  MODE_1 = 1,
  MODE_2 = 2,
  MODE_3 = 3
} MAX31856_OpenCircuitMode;

class MAX31856 {
  public:
    static const uint8_t MAX31856_FAULT_CJRANGE   = 0x80;
    static const uint8_t MAX31856_FAULT_TCRANGE   = 0x40;
    static const uint8_t MAX31856_FAULT_CJHIGH    = 0x20;
    static const uint8_t MAX31856_FAULT_CJLOW     = 0x10;
    static const uint8_t MAX31856_FAULT_TCHIGH    = 0x08;
    static const uint8_t MAX31856_FAULT_TCLOW     = 0x04;
    static const uint8_t MAX31856_FAULT_OVUV      = 0x02;
    static const uint8_t MAX31856_FAULT_OPEN      = 0x01;

    // Constructor for software SPI (bitbang)
    MAX31856(uint8_t spi_cs, uint8_t spi_mosi, uint8_t spi_miso, uint8_t spi_clk, uint8_t drdyPin, uint8_t faultPin);
    MAX31856(uint8_t spi_cs, uint8_t spi_mosi, uint8_t spi_miso, uint8_t spi_clk, uint8_t drdyPin);
    MAX31856(uint8_t spi_cs, uint8_t spi_mosi, uint8_t spi_miso, uint8_t spi_clk);

    // Constructor for hardware SPI
    MAX31856(uint8_t spi_cs, uint8_t drdyPin, uint8_t faultPin);
    MAX31856(uint8_t spi_cs, uint8_t drdyPin);
    MAX31856(uint8_t spi_cs);
    MAX31856();

    // Must be called after initialization but before the class is used
    void init(bool initSPI);

    // Temperature-related functions
    void setThermocoupleTypeAndOversampling(MAX31856_TCType type, MAX31856_SampleAveraging samples);
    MAX31856_TCType getThermocoupleType(void);
    void startAutoConversion(void);
    void stopAutoConversion(void);
    void oneShotTemperature(void);
    float readTCTemperature(void);
    float readCJTemperature(void);

    // Thresholds for the temperature readings
    void setTCFaultThresholds(float flow, float fhigh);
    void setCJFaultThreshholds(int8_t low, int8_t high);

    // Status checks
    bool isFaultless();
    bool dataIsReady();

    // Other functions
    void reject50Hz(void);
    void checkOpenCircuit(MAX31856_OpenCircuitMode mode);
    uint8_t getFault(void);

  private:
    // p5 of MAX31856 datasheet: Serial clock max frequency is 5 MHz
    // p15 of MAX31856 datasheet: "Address and data bytes are shifted MSB-first"
    // p6 of MAX31856 datasheet: SCLK CAN BE EITHER POLARITY
    // The SPI code would automatically choose a clock divider such that the SPI speed is less or equal to the number given
    // See https://dorkbotpdx.org/blog/paul/spi_transactions_in_arduino
    const SPISettings max31856_spisettings = SPISettings(4000000, MSBFIRST, SPI_MODE1);

    // Registers
    static const uint8_t MAX31856_CR0_REG       = 0x00;
    static const uint8_t MAX31856_CR1_REG       = 0x01;
    static const uint8_t MAX31856_MASK_REG      = 0x02;
    static const uint8_t MAX31856_CJHF_REG      = 0x03;
    static const uint8_t MAX31856_CJLF_REG      = 0x04;
    static const uint8_t MAX31856_LTHFTH_REG    = 0x05;
    static const uint8_t MAX31856_LTHFTL_REG    = 0x06;
    static const uint8_t MAX31856_LTLFTH_REG    = 0x07;
    static const uint8_t MAX31856_LTLFTL_REG    = 0x08;
    static const uint8_t MAX31856_CJTO_REG      = 0x09;
    static const uint8_t MAX31856_CJTH_REG      = 0x0A;
    static const uint8_t MAX31856_CJTL_REG      = 0x0B;
    static const uint8_t MAX31856_LTCBH_REG     = 0x0C;
    static const uint8_t MAX31856_LTCBM_REG     = 0x0D;
    static const uint8_t MAX31856_LTCBL_REG     = 0x0E;
    static const uint8_t MAX31856_SR_REG        = 0x0F;

     // These are all for configuring the register MAX31856_CR0_REG
    static const uint8_t MAX31856_CR0_AUTOCONVERT   = 0x80; // p19 of MAX31856 manual: Combine this with current address values using a bitwise OR and write to address MAX31856_CR0_REG to enable auto-conversion/measurement mode (1 measurement every ~100 ms)
    static const uint8_t MAX31856_CR0_1SHOT         = 0x40; // p19 of MAX31856 manual: Combine this with current address values using a bitwise OR and write to address MAX31856_CR0_REG to induce a one-shot temperature measurement
    static const uint8_t MAX31856_CR0_OCFAULT1      = 0x10; // See Table 4, p14
    static const uint8_t MAX31856_CR0_OCFAULT2      = 0x20; // See Table 4, p14
    static const uint8_t MAX31856_CR0_OCFAULT3      = 0x30; // See Table 4, p14; note that the inverse of this would be to disable open-circuit detection
    static const uint8_t MAX31856_CR0_CJ            = 0x08; // p19 of MAX31856 manual: Combine this with current address values using a bitwise OR and write to address MAX31856_CR0_REG to disable Cold-Junction Sensor
    static const uint8_t MAX31856_CR0_FAULT         = 0x04;
    static const uint8_t MAX31856_CR0_FAULTCLR      = 0x02;
    static const uint8_t MAX31856_CR0_FILTER50HZ    = 0x01; // ADDED. p19 of MAX31856 manual: Combine this with current address values using a bitwise OR and write to address MAX31856_CR0_REG to change filter rejection frequency to 50 Hz

    // Status flags
    bool hardwareSPI_;
    bool autoRead_;
    bool readDRDY_;
    bool readFAULT_;
  
    // Pins
    uint8_t sclkPin_, misoPin_, mosiPin_, csPin_, drdyPin_, faultPin_;

    // Shared buffers
    uint8_t readBuffer_, readRegister8Buffer_, rawReadRegister16Buffer_[2], rawReadRegister24Buffer_[3];
    uint16_t readRegister16Buffer_;
    uint32_t readRegister24Buffer_;

    // SPI read/write functions
    void readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n);
    uint8_t  readRegister8(uint8_t addr);
    uint16_t readRegister16(uint8_t addr);
    uint32_t readRegister24(uint8_t addr);
    void     writeRegister8(uint8_t addr, uint8_t reg);
    uint8_t spixfer(uint8_t addr);
};

#endif
