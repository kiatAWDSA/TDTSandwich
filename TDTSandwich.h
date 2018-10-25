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

#ifndef MAX31856_H
#define MAX31856_H

typedef enum
{
  aMAX31856_TCTYPE_B  = 0b0000,
  aMAX31856_TCTYPE_E  = 0b0001,
  aMAX31856_TCTYPE_J  = 0b0010,
  aMAX31856_TCTYPE_K  = 0b0011,
  aMAX31856_TCTYPE_N  = 0b0100,
  aMAX31856_TCTYPE_R  = 0b0101,
  aMAX31856_TCTYPE_S  = 0b0110,
  aMAX31856_TCTYPE_T  = 0b0111,
  aMAX31856_VMODE_G8  = 0b1000,
  aMAX31856_VMODE_G32 = 0b1100,
} Amax31856_thermocoupletype_t;

typedef enum
{
  aMAX31856_OVERSAMPLE_1  = 0x00,
  aMAX31856_OVERSAMPLE_2  = 0x10,
  aMAX31856_OVERSAMPLE_4  = 0x20,
  aMAX31856_OVERSAMPLE_8  = 0x30,
  aMAX31856_OVERSAMPLE_16 = 0x40,
} Amax31856_oversampleCount;

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

class MAX31856 {
 public:
  MAX31856(int8_t spi_cs, int8_t spi_mosi, int8_t spi_miso, int8_t spi_clk, boolean autoRead);
  MAX31856(int8_t spi_cs, boolean autoRead);
  MAX31856();

  boolean open(void);
  void startAutoConversion(void);
  void close(void);

  void setThermocoupleTypeAndOversampling(Amax31856_thermocoupletype_t type, Amax31856_oversampleCount samples);
  Amax31856_thermocoupletype_t getThermocoupleType(void);

  uint8_t readFault(void);
  void oneShotTemperature(void);
  void reject50Hz(void);

  float readCJTemperature(void);
  float readThermocoupleTemperature(void);

  void setTempFaultThreshholds(float flow, float fhigh);
  void setColdJunctionFaultThreshholds(int8_t low, int8_t high);

 private:
  boolean _autoRead;
  
  int8_t _sclk, _miso, _mosi, _cs;
  uint8_t _readBuffer, _readRegister8Buffer, _rawReadRegister16Buffer[2], _rawReadRegister24Buffer[3];
  uint16_t _readRegister16Buffer;
  uint32_t _readRegister24Buffer;

  void readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n);

  uint8_t  readRegister8(uint8_t addr);
  uint16_t readRegister16(uint8_t addr);
  uint32_t readRegister24(uint8_t addr);

  void     writeRegister8(uint8_t addr, uint8_t reg);
  uint8_t spixfer(uint8_t addr);
};

#endif
