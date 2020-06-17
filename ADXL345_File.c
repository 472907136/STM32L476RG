/**************************************************************************/
/*!
    @file     ADXL345_File.c
    @author   Lucien
*/
/**************************************************************************/

#include <limits.h>

#include "ADXL345_Header.h"

/**************************************************************************/
void i2cwrite(uint8_t x) {

  Wire.send(x);

}

/**************************************************************************/

/**
 * @brief  Abstract away SPI receiver & transmitter

 * @param clock The pin number for SCK, the SPI ClocK line
 * @param miso The pin number for MISO, the SPI Master In Slave Out line
 * @param mosi The pin number for MOSI, the SPI Master Out Slave In line
 * @param data The byte to send
 *
 * @return uint8_t The single byte response
 */
static uint8_t spixfer(uint8_t clock, uint8_t miso, uint8_t mosi,
                       uint8_t data) {
  uint8_t reply = 0;

  return reply;
}

/**************************************************************************/
/*!
    @brief  Writes one byte to the specified destination register
    @param reg The address of the register to write to
    @param value The value to set the register to
*/
/**************************************************************************/
void writeRegister(uint8_t reg, uint8_t value) {
  if (_i2c) {

  } else {

  }
}

/**************************************************************************/
/*!
    @brief Reads one byte from the specified register
    @param reg The address of the register to read from
    @returns The single byte value of the requested register
*/
/**************************************************************************/
uint8_t readRegister(uint8_t reg) {
	  uint8_t reply = 0;
  if (_i2c) {

    return (i2cread());
  } else {

    return reply;
  }
}

/**************************************************************************/
/*!
    @brief Reads two bytes from the specified register
    @param reg The address of the register to read from
    @return The two bytes read from the sensor starting at the given address
*/
/**************************************************************************/
int16_t read16(uint8_t reg) {
	  uint8_t reply = 0;
  if (_i2c) {

    return (uint16_t)(i2cread() | (i2cread() << 8));
  } else {

    return reply;
  }
}

/**************************************************************************/
/*!
    @brief  Reads the device ID (can be used to check connection)
    @return The Device ID of the connected sensor
*/
/**************************************************************************/
uint8_t getDeviceID(void) {
  // Check device ID register
  return readRegister(ADXL345_REG_DEVID);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent X axis value
    @return The raw `int16_t` unscaled x-axis acceleration value
*/
/**************************************************************************/
int16_t getX(void) {
  return read16(ADXL345_REG_DATAX0);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent Y axis value
    @return The raw `int16_t` unscaled y-axis acceleration value
*/
/**************************************************************************/
int16_t getY(void) {
  return read16(ADXL345_REG_DATAY0);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent Z axis value
    @return The raw `int16_t` unscaled z-axis acceleration value
*/
/**************************************************************************/
int16_t getZ(void) {
  return read16(ADXL345_REG_DATAZ0);
}

/**************************************************************************/
/*!
    @brief  Instantiates a new ADXL345 class
    @param sensorID A unique ID to use to differentiate the sensor from others
*/
/**************************************************************************/
void Adafruit_ADXL345_Unified(int32_t sensorID) {
  _sensorID = sensorID;
  _range = ADXL345_RANGE_2_G;
  _i2c = true;
}

/**************************************************************************/
/*!
    @brief  Instantiates a new ADXL345 class in SPI mode
    @param clock The pin number for SCK, the SPI ClocK line
    @param miso The pin number for MISO, the SPI Master In Slave Out line
    @param mosi The pin number for MOSI, the SPI Master Out Slave In line
    @param cs The pin number for CS, the SPI Chip Select line
    @param sensorID A unique ID to use to differentiate the sensor from others
*/
/**************************************************************************/
void Adafruit_ADXL345_Unified(uint8_t clock, uint8_t miso,
                                                   uint8_t mosi, uint8_t cs,
                                                   int32_t sensorID) {
  _sensorID = sensorID;
  _range = ADXL345_RANGE_2_G;
  _cs = cs;
  _clk = clock;
  _do = mosi;
  _di = miso;
  _i2c = false;
}

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
    @param i2caddr The I2C address to begin communication with
    @return true: success false: a sensor with the correct ID was not found
*/
/**************************************************************************/
ADXL345_StatusTypeDef begin(uint8_t i2caddr) {
  _i2caddr = i2caddr;

  if (_i2c)
    Wire.begin();
  else {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    pinMode(_clk, OUTPUT);
    digitalWrite(_clk, HIGH);
    pinMode(_do, OUTPUT);
    pinMode(_di, INPUT);
  }

  /* Check connection */
  uint8_t deviceid = getDeviceID();
  if (deviceid != 0xE5) {
    /* No ADXL345 detected ... return false */
    return ADXL345_ERROR;
  }

  // Enable measurements
  writeRegister(ADXL345_REG_POWER_CTL, 0x08);

  return ADXL345_OK;
}

/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
    @param range The new `range_t` to set the accelerometer to
*/
/**************************************************************************/
void setRange(range_t range) {
  /* Read the data format register to preserve bits */
  uint8_t format = readRegister(ADXL345_REG_DATA_FORMAT);

  /* Update the data rate */
  format &= ~0x0F;
  format |= range;

  /* Make sure that the FULL-RES bit is enabled for range scaling */
  format |= 0x08;

  /* Write the register back to the IC */
  writeRegister(ADXL345_REG_DATA_FORMAT, format);

  /* Keep track of the current range (to avoid readbacks) */
  _range = range;
}

/**************************************************************************/
/*!
    @brief  Gets the g range for the accelerometer
    @return The current `range_t` value
*/
/**************************************************************************/
range_t getRange(void) {
  /* Read the data format register to preserve bits */
  return (range_t)(readRegister(ADXL345_REG_DATA_FORMAT) & 0x03);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the ADXL345 (controls power consumption)
    @param dataRate The `dataRate_t` to set
*/
/**************************************************************************/
void setDataRate(dataRate_t dataRate) {
  /* Note: The LOW_POWER bits are currently ignored and we always keep
     the device in 'normal' mode */
  writeRegister(ADXL345_REG_BW_RATE, dataRate);
}

/**************************************************************************/
/*!
    @brief  Gets the data rate for the ADXL345 (controls power consumption)
    @return The current data rate
*/
/**************************************************************************/
dataRate_t getDataRate(void) {
  return (dataRate_t)(readRegister(ADXL345_REG_BW_RATE) & 0x0F);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
    @param event Pointer to the event object to fill
    @return true: success
*/
/**************************************************************************/
ADXL345_StatusTypeDef getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = 0;
  event->acceleration.x =
      getX() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  event->acceleration.y =
      getY() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  event->acceleration.z =
      getZ() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;

  return ADXL345_OK;
}

/**************************************************************************/
/*!
 */
/**************************************************************************/

/**
 * @brief Fill a `sensor_t` struct with information about the sensor
 *
 * @param sensor Pointer to a `sensor_t` struct to fill
 */
void getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "ADXL345", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay = 0;
  sensor->max_value = -156.9064F; /* -16g = 156.9064 m/s^2  */
  sensor->min_value = 156.9064F;  /*  16g = 156.9064 m/s^2  */
  sensor->resolution = 0.03923F;  /*  4mg = 0.0392266 m/s^2 */
}
