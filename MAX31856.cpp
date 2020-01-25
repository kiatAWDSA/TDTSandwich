/*************************************************** 
  This is a library for the Adafruit Thermocouple Sensor w/MAX31856

  Designed specifically to work with the Adafruit Thermocouple Sensor
  ----> https://www.adafruit.com/product/3263
  
  These sensors use SPI to communicate, 4 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  Modified by Soon Kiat Lau 2017
  BSD license, all text above must be included in any redistribution
 ****************************************************/

 /*
	Directions:
	1. open()
	2. setThermocoupleTypeAndOversampling(type, samples)  [ set your thermocouple type and how many samples you average during conversion ]
	3. startAutoConversion()		OR		oneShotTemperature  +  readThermocoupleTemperature()
*/
 
#include "MAX31856.h"

// Software (bitbang) SPI. Read DRDY and FAULT pin
MAX31856::MAX31856(uint8_t spi_cs, uint8_t spi_mosi, uint8_t spi_miso, uint8_t spi_clk, uint8_t drdyPin, uint8_t faultPin) {
  sclkPin_      = spi_clk;
  csPin_        = spi_cs;
  misoPin_      = spi_miso;
  mosiPin_      = spi_mosi;
  drdyPin_      = drdyPin;
  faultPin_     = faultPin;
  hardwareSPI_  = false;
  readDRDY_     = true;
  readFAULT_    = true;

  digitalWrite(csPin_, HIGH);
  pinMode(csPin_, OUTPUT);
}

// Software (bitbang) SPI. Read DRDY pin
MAX31856::MAX31856(uint8_t spi_cs, uint8_t spi_mosi, uint8_t spi_miso, uint8_t spi_clk, uint8_t drdyPin) {
  sclkPin_      = spi_clk;
  csPin_        = spi_cs;
  misoPin_      = spi_miso;
  mosiPin_      = spi_mosi;
  drdyPin_      = drdyPin;
  hardwareSPI_  = false;
  readDRDY_     = true;
  readFAULT_    = false;

  digitalWrite(csPin_, HIGH);
  pinMode(csPin_, OUTPUT);
}

// Software (bitbang) SPI.
MAX31856::MAX31856(uint8_t spi_cs, uint8_t spi_mosi, uint8_t spi_miso, uint8_t spi_clk) {
  sclkPin_      = spi_clk;
  csPin_        = spi_cs;
  misoPin_      = spi_miso;
  mosiPin_      = spi_mosi;
  hardwareSPI_  = false;
  readDRDY_     = false;
  readFAULT_    = false;

  digitalWrite(csPin_, HIGH);
  pinMode(csPin_, OUTPUT);
}

// Hardware SPI. Read DRDY and FAULT pin
MAX31856::MAX31856(uint8_t spi_cs, uint8_t drdyPin, uint8_t faultPin) {
  csPin_        = spi_cs;
  drdyPin_      = drdyPin;
  faultPin_     = faultPin;
  hardwareSPI_  = true;
  readDRDY_     = true;
  readFAULT_    = true;

  digitalWrite(csPin_, HIGH);
  pinMode(csPin_, OUTPUT);
}

// Hardware SPI. Read DRDY pin.
MAX31856::MAX31856(uint8_t spi_cs, uint8_t drdyPin) {
  csPin_        = spi_cs;
  drdyPin_      = drdyPin;
  hardwareSPI_  = true;
  readDRDY_     = true;
  readFAULT_    = false;

  digitalWrite(csPin_, HIGH);
  pinMode(csPin_, OUTPUT);
}

// Hardware SPI. Added argument to choose to use the auto-conversion mode
MAX31856::MAX31856(uint8_t spi_cs) {
  csPin_        = spi_cs;
  hardwareSPI_  = true;
  readDRDY_     = false;
  readFAULT_    = false;

  digitalWrite(csPin_, HIGH);
  pinMode(csPin_, OUTPUT);
}

// For situations where we do not want to instantiate the class at the beginning of program.
// NOTE: Attempting to use the object instantiated with this constructor will surely give an error because none of the pins are set yet!
MAX31856::MAX31856() {}


// Opens a connection to the amplifier and set some default settings, but don't begin taking any temperature
void MAX31856::init(bool initSPI) {
  if (readDRDY_)
  {
    pinMode(drdyPin_, INPUT);
  }
  if (readFAULT_)
  {
    pinMode(faultPin_, INPUT);
  }

  if (initSPI)
  {
    // Would be set to -1 if we initialized with hardware SPI
    if (hardwareSPI_)
    { // Start and configure hardware SPI
      SPI.begin();
    }
    else
    { // define pin modes for software SPI
      pinMode(sclkPin_, OUTPUT);
      pinMode(mosiPin_, OUTPUT);
      pinMode(misoPin_, INPUT);
    }
  }

  // Assert on any fault (i.e. don't hide/mute any errors)
  writeRegister8(MAX31856_MASK_REG, 0x0);
  
  // Set a default thermocouple type in case user start measurement without setting a type first
  //  Default settings: type T, average 2 samples
  setThermocoupleTypeAndOversampling(TCTYPE_T, AVERAGE_2);
  
  // AX31856_CR0_OCFAULT1 allows open-circuit fault detection, mode "01" (see Table 4 in pg 14)
  // NOTE: This will set all the other bits in register 00 to 0 (including auto-conversion bit). By default, the other bits should be 0 anyway.
  writeRegister8(MAX31856_CR0_REG, MAX31856_CR0_OCFAULT1);
}

// ADDED. Allows us to start-conversion. The auto-conversion mode of the MAX31856 has less delay (~100ms) than one-shots after its first reading
void MAX31856::startAutoConversion(void) {
  readBuffer_ = readRegister8(MAX31856_CR0_REG);
  readBuffer_ &= ~MAX31856_CR0_1SHOT;       // turn off one-shot
  readBuffer_ |= MAX31856_CR0_AUTOCONVERT;  // turn on auto conversion
	writeRegister8(MAX31856_CR0_REG, readBuffer_);
  autoRead_ = true;
}


// ADDED a function that stops auto-conversion
void MAX31856::stopAutoConversion(void) {
  // First, stop all auto-conversion/one-shot
  readBuffer_ = readRegister8(MAX31856_CR0_REG);
  readBuffer_ &= 0x3F; // 0011 1111  ;  thise makes the bits for auto-conversion and one-shot 0
  writeRegister8(MAX31856_CR0_REG, readBuffer_);
  autoRead_ = false;
}

void MAX31856::setThermocoupleTypeAndOversampling(MAX31856_TCType type, MAX31856_SampleAveraging samples) {
  readBuffer_ = readRegister8(MAX31856_CR1_REG);
  readBuffer_ &= 0x80; // Force all bits except bit 7 to be 0
  readBuffer_ |= (uint8_t)type & 0x0F; // Thermocouple type
  readBuffer_ |= (uint8_t)samples; // Number of samples to read and average
  writeRegister8(MAX31856_CR1_REG, readBuffer_);
}

MAX31856_TCType MAX31856::getThermocoupleType(void) {
  readBuffer_ = readRegister8(MAX31856_CR1_REG);
  readBuffer_ &= 0x0F;

  return (MAX31856_TCType)(readBuffer_);
}

float MAX31856::readTCTemperature(void) {
  if (!autoRead_)
  {
	  oneShotTemperature();
  }

  int32_t temp24 = readRegister24(MAX31856_LTCBH_REG); // Use 24 because we want to read 3 registers (LTCBH_REG, LTCBM_REG, LTCBL_REG).
  // See the comments in the while loop in the function readRegisterN to understand how this works. Also, pg 13 of MAX31856 manual stresses that all three registers must be read together "to ensure all are from the same data update"
  
  if (temp24 & 0x800000) { // 1000 0000 0000 0000 0000 0000
    temp24 |= 0xFF000000;  // 1111 1111 0000 0000 0000 0000 // Sign bit is 1. Fix sign
  }

  temp24 >>= 5;  // bottom 5 bits are unused (see readRegister24 function)

  float tempfloat = temp24;
  tempfloat *= 0.0078125;

  return tempfloat;
}

float MAX31856::readCJTemperature(void) {
  oneShotTemperature();

  int16_t temp16 = readRegister16(MAX31856_CJTH_REG);
  float tempfloat = temp16;
  tempfloat /= 256.0;

  return tempfloat;
}

void MAX31856::setTCFaultThresholds(float flow, float fhigh) {
  int16_t low, high;

  flow *= 16;
  low = flow;

  fhigh *= 16;
  high = fhigh;

  writeRegister8(MAX31856_LTHFTH_REG, high >> 8);
  writeRegister8(MAX31856_LTHFTL_REG, high);

  writeRegister8(MAX31856_LTLFTH_REG, low >> 8);
  writeRegister8(MAX31856_LTLFTL_REG, low);
}

void MAX31856::setCJFaultThreshholds(int8_t low, int8_t high) {
  writeRegister8(MAX31856_CJLF_REG, low);
  writeRegister8(MAX31856_CJHF_REG, high);
}

void MAX31856::oneShotTemperature(void) {

  writeRegister8(MAX31856_CJTO_REG, 0x0); // Note: We can apply an offset to the cold junction thermocouple here. See pg 23 of MAX31856 manual

  readBuffer_ = readRegister8(MAX31856_CR0_REG);

  readBuffer_ |= MAX31856_CR0_1SHOT;

  writeRegister8(MAX31856_CR0_REG, readBuffer_);

  delay(250); // MEME FIX autocalculate based on oversampling. This is implemented by the Adafruit team so that the amplifier is guaranteed to give a stable reading (e.g. if the amplifier was told to sample a few readings and average them (p20 of manual), it might still be in the middle of doing this if the 250ms delay is not present). See https://forums.adafruit.com/viewtopic.php?f=19&t=108896&p=544342
}

// This can only be used if the class was initialized with a FAULT pin
// FAULT goes LOW when there is a problem (with exception to cold junction and TC range)
bool MAX31856::isFaultless()
{
  if (readFAULT_)
  {
    return digitalRead(faultPin_);
  }
  else
  {// If the fault pin is not set, then we don't know the status of the FAULT pin.
   // In this case, just return a "no fault".
    return true;
  }
}

// This can only be used if the class was initialized with a DRDY pin
// DRDY goes LOW when data is available
bool MAX31856::dataIsReady()
{
  if (readDRDY_)
  {
    return !digitalRead(drdyPin_);
  }
  else
  {// If the DRDY pin is not set, then we don't know the status of the DRDY pin.
   // In this case, just return a "data ready".
    return true;
  }
}

// ADDED. Sets the Noise Rejection Filter to reject 50 Hz and its harmonics
void MAX31856::reject50Hz(void) {
  readBuffer_ = readRegister8(MAX31856_CR0_REG);
  readBuffer_ |= MAX31856_CR0_FILTER50HZ; // Filter out 50 Hz
  writeRegister8(MAX31856_CR0_REG, readBuffer_);
}

/***********************************************************************
Triggers a check for open thermocouple circuit on the next conversion.
This follows the procedure outlined in p14 of datasheet:
If on-demand detection is desired, select
“detection disabled”(00), then select the setting for the
desired time constant. An open - circuit detection test will
be performed immediately after the current conversion is
completed.
***********************************************************************/
void MAX31856::checkOpenCircuit(MAX31856_OpenCircuitMode mode)
{
  readBuffer_ = readRegister8(MAX31856_CR0_REG);

  // Disable open-circuit detection by inversing mode 3
  writeRegister8(MAX31856_CR0_REG, readBuffer_ & ~MAX31856_CR0_OCFAULT3);

  // Se the desired open-circuit detection mode based on the given mode/time constant
  switch (mode)
  {
  case MODE_2:
    writeRegister8(MAX31856_CR0_REG, readBuffer_ | MAX31856_CR0_OCFAULT2);
    break;
  case MODE_3:
    writeRegister8(MAX31856_CR0_REG, readBuffer_ | MAX31856_CR0_OCFAULT3);
    break;
  case MODE_1:
  default:
    writeRegister8(MAX31856_CR0_REG, readBuffer_ | MAX31856_CR0_OCFAULT1);
    break;
  }
}

uint8_t MAX31856::getFault(void) {
  return readRegister8(MAX31856_SR_REG);
}

/**********************************************/

uint8_t MAX31856::readRegister8(uint8_t addr) {
  readRegister8Buffer_ = 0;
  readRegisterN(addr, &readRegister8Buffer_, 1);

  return readRegister8Buffer_;
}

uint16_t MAX31856::readRegister16(uint8_t addr) {
  readRegisterN(addr, rawReadRegister16Buffer_, 2);

  // Convert the array-form of raw buffer to a single uint
  readRegister16Buffer_ = rawReadRegister16Buffer_[0];
  readRegister16Buffer_ <<= 8;
  readRegister16Buffer_ |=  rawReadRegister16Buffer_[1];
  
  return readRegister16Buffer_;
}

uint32_t MAX31856::readRegister24(uint8_t addr) {
  readRegisterN(addr, rawReadRegister24Buffer_, 3);

  readRegister24Buffer_ = rawReadRegister24Buffer_[0];		// _readRegister24Buffer = 0000 0000 0000 0000 sxxx xxxx
  readRegister24Buffer_ <<= 8;								// _readRegister24Buffer = 0000 0000 sxxx xxxx 0000 0000
  readRegister24Buffer_ |=  rawReadRegister24Buffer_[1];	// _readRegister24Buffer = 0000 0000 sxxx xxxx yyyy yyyy
  readRegister24Buffer_ <<= 8;								// _readRegister24Buffer = sxxx xxxx yyyy yyyy 0000 0000
  readRegister24Buffer_ |=  rawReadRegister24Buffer_[2];	// _readRegister24Buffer = sxxx xxxx yyyy yyyy zzz? ????     // pg 24-25 of MAX31856 manual: The temperature is 19-bit, first bit is sign (s). There are some scrap entries at the end.
  
  return readRegister24Buffer_;
}


void MAX31856::readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n) {
  addr &= 0x7F; // 0111 1111 // make sure top bit is not set
  // p15 of MAX31856 manual: The address byte is always the first byte transferred after CS is driven low. The MSB (A7) of this byte determines whether the following byte will be written or read.
  // If A7 is 0, one or more byte reads will follow the address byte.
  //
  // p18 of MAX31856 manual: The registers are accessed using the 0Xh addresses for reads (X is replaced by number of register)

  if (hardwareSPI_)
    SPI.beginTransaction(max31856_spisettings);
  else 
    digitalWrite(sclkPin_, HIGH);

  delayMicroseconds(2); // Give a slight delay to ensure SPI line is steady
  digitalWrite(csPin_, LOW);

  spixfer(addr);

  //Serial.print("$"); Serial.print(addr, HEX); Serial.print(": ");
  while (n--) {
    // 0xFF is a dummy byte used to trigger a read. See the following statement from manual:
    // pg 15 of MAX31856 manual: Any transfer requires the address of the byte to specify a write or a read (done by spixfer(addr)),
    // followed by **one or more bytes of data**. [...] For a multiple-byte transfer, multiple bytes can be read or written after the address has been written (hence the while loop)
    buffer[0] = spixfer(0xFF);
	
	
    //Serial.print(" 0x"); Serial.print(buffer[0], HEX);
    buffer++;
  }
  //Serial.println();

  digitalWrite(csPin_, HIGH);

  if (hardwareSPI_)
    SPI.endTransaction();
}


void MAX31856::writeRegister8(uint8_t addr, uint8_t data) {
  addr |= 0x80; // 1000 0000 // make sure top bit is set
  // p15 of MAX31856 manual: The address byte is always the first byte transferred after CS is driven low. The MSB (A7) of this byte determines whether the following byte will be written or read.
  // If A7 is 1, one or more byte writes will follow the address byte.
  //
  // p18 of MAX31856 manual: The registers are accessed using the [...] 8Xh addresses for writes (X is replaced by number of register)

  if (hardwareSPI_)
    SPI.beginTransaction(max31856_spisettings);
  else 
    digitalWrite(sclkPin_, HIGH);

  delayMicroseconds(2); // Give a slight delay to ensure SPI line is steady
  digitalWrite(csPin_, LOW);

  spixfer(addr);
  spixfer(data);

  //Serial.print("$"); Serial.print(addr, HEX); Serial.print(" = 0x"); Serial.println(data, HEX);

  digitalWrite(csPin_, HIGH);

  if (hardwareSPI_)
    SPI.endTransaction();
}



uint8_t MAX31856::spixfer(uint8_t x) {
  if (hardwareSPI_)
    return SPI.transfer(x);

  // software spi (bitbang)
  //Serial.println("Software SPI");
  uint8_t reply = 0; // TODO: Declare this in the class variables so it uses static memory
  for (int i=7; i>=0; i--) {
    reply <<= 1;
    digitalWrite(sclkPin_, LOW);
    digitalWrite(mosiPin_, x & (1<<i));
    digitalWrite(sclkPin_, HIGH);
    if (digitalRead(misoPin_))
      reply |= 1;
  }
  return reply;
}
