/**************************************************************************/
/*!
  *  @file     ADXL345_File.c
  *  @author   Lucien
*/
/**************************************************************************/

#include "ADXL345_Header.h"
#include <string.h>

/**************************************************************************/
/*!
    @brief  Reads the device ID (can be used to check connection)
    @return The Device ID of the connected sensor
*/
/**************************************************************************/
void getDeviceID(ADXL345_sensor *pa){
//	uint8_t * pdevid;
//	pdevid = &Dev_ID_Address;// Envoyer l'adresse de registre
//	HAL_I2C_Master_Transmit(&hi2c3,chip_id,pdevid,1,10);
//	HAL_I2C_Master_Receive(&hi2c3,chip_id,&pa->dev_Id,1,10);

}

/**************************************************************************/
/*!
    @brief  Gets the most recent X axis value
    @return The raw `int16_t` unscaled x-axis acceleration value
*/
/**************************************************************************/
//int16_t getX(void) {
//  return read16(ADXL345_REG_DATAX0);
//}

/**************************************************************************/
/*!
    @brief  Gets the most recent Y axis value
    @return The raw `int16_t` unscaled y-axis acceleration value
*/
/**************************************************************************/
//int16_t getY(void) {
//  return read16(ADXL345_REG_DATAY0);
//}

/**************************************************************************/
/*!
    @brief  Gets the most recent Z axis value
    @return The raw `int16_t` unscaled z-axis acceleration value
*/
/**************************************************************************/
//int16_t getZ(void) {
//  return read16(ADXL345_REG_DATAZ0);
//}

/**************************************************************************/
/*!
  * @brief  Setups the HW (reads coefficients values, etc.)
  * @param  address:register address,  val:value to write in register
  * @return 
*/
/**************************************************************************/
void AccellerometreConfigure (int8_t address, int8_t val){
  // Commencer la transmission à trois axes accéléromètre
	ADXL345_sensor my_sensor;
	my_sensor.write_device(ADXL345_ADDRESS, address, val);
}
/**
 * @brief Fill a `ADXL345_sensor` struct with information about the sensor
 *
 * @param sensor Pointer to a `ADXL345_sensor` struct to fill
 */
void init_Sensor(ADXL345_sensor *sensor) {
  /* Clear the ADXL345_sensor object */
  memset(sensor, 0, sizeof(ADXL345_sensor));
  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->sensor_name, DEV_NAME, sizeof(sensor->sensor_name) - 1);
  sensor->sensor_name[sizeof(sensor->sensor_name) - 1] = 0;
	sensor->read_device(ADXL345_ADDRESS, ADXL345_REG_DEVID, &sensor->dev_id,1);
//  sensor->min_delay = 0;
//  sensor->max_value = -156.9064F; /* -16g = 156.9064 m/s^2  */
//  sensor->min_value = 156.9064F;  /*  16g = 156.9064 m/s^2  */
//  sensor->resolution = 0.03923F;  /*  4mg = 0.0392266 m/s^2 */
}
/**************************************************************************/
/*!
  * @brief  
  * @param  my_sensor:  my_range:
  * @return true: success false: a sensor with the correct ID was not found
*/
/**************************************************************************/
ADXL345_StatusTypeDef ADXL345_begin(ADXL345_sensor *my_sensor, range_t my_range) {
	const char MesureMode   = 0x08; // mesure may begin
	init_Sensor(my_sensor); // Clear the ADXL345_sensor object
  if (my_sensor->dev_id != 0xE5) {
    /* No ADXL345 detected ... return false */
    return HAL_ERROR;
  }	
// Select range +/-xG writting my_range in register DATA_FORMAT.
	AccellerometreConfigure (ADXL345_REG_DATA_FORMAT, my_range);
// Mettre le ADXL345 en mode de mesure en écrivant 0x08 dans le registre POWER_CTL.
	AccellerometreConfigure (ADXL345_REG_POWER_CTL, MesureMode);
  /* Check device ID */

	return HAL_OK;	
}
/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
    @param range The new `range_t` to set the accelerometer to
*/
/**************************************************************************/
void setRange(range_t range) {
  /* Read the data format register to preserve bits */
//  uint8_t format = readRegister(ADXL345_REG_DATA_FORMAT);

//  /* Update the data rate */
//  format &= ~0x0F;
//  format |= range;

//  /* Make sure that the FULL-RES bit is enabled for range scaling */
//  format |= 0x08;

//  /* Write the register back to the IC */
//  writeRegister(ADXL345_REG_DATA_FORMAT, format);

//  /* Keep track of the current range (to avoid readbacks) */
//  _range = range;
}

/**************************************************************************/
/*!
    @brief  Gets the g range for the accelerometer
    @return The current `range_t` value
*/
/**************************************************************************/
range_t getRange(void) {
  /* Read the data format register to preserve bits */
//  return (range_t)(readRegister(ADXL345_REG_DATA_FORMAT) & 0x03);
	return ADXL345_RANGE_8_G; // A RETIRER
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
//  writeRegister(ADXL345_REG_BW_RATE, dataRate);
}

/**************************************************************************/
/*!
    @brief  Gets the data rate for the ADXL345 (controls power consumption)
    @return The current data rate
*/
/**************************************************************************/
dataRate_t getDataRate(void) {
//  return (dataRate_t)(readRegister(ADXL345_REG_BW_RATE) & 0x0F);
	return ADXL345_DATARATE_3200_HZ; // A RETIRER	
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
    @param event Pointer to the event object to fill
    @return true: success
*/
/**************************************************************************/
//ADXL345_StatusTypeDef getEvent(sensors_event_t *event) {
//  /* Clear the event */
//  memset(event, 0, sizeof(sensors_event_t));

//  event->version = sizeof(sensors_event_t);
//  event->sensor_id = _sensorID;
//  event->type = SENSOR_TYPE_ACCELEROMETER;
//  event->timestamp = 0;
//  event->acceleration.x =
//      getX() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
//  event->acceleration.y =
//      getY() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
//  event->acceleration.z =
//      getZ() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;

//  return ADXL345_OK;
//}

