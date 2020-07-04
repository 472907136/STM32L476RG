/**************************************************************************/
/*!
  *  @file     ADXL345_File.c
  *  @author   Lucien
*/
/**************************************************************************/

#include "ADXL345_Header.h"
#include <string.h>
#include "usart.h"

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
void AccellerometreConfigure (ADXL345_sensor *sensor, int8_t address, int8_t val){
  // Commencer la transmission à trois axes accéléromètre
	sensor->write_device(ADXL345_ADDRESS, address, val);
}
/**
 * @brief Feed an `ADXL345_sensor` struct with sensor features
 *
 * @param sensor Pointer to a `ADXL345_sensor` struct to feed
 */
void init_Sensor(ADXL345_sensor *sensor) {
	// store members set up by user, if so!
	ADXL345_read pread;
	ADXL345_write pwrite;
	ADXL345_delay pdelay;
	// store addresses
	pread = sensor->read_device;
	pwrite = sensor->write_device;
	pdelay = sensor->ms_delay;
  /* Now clear the ADXL345_sensor object */
  memset(sensor, 0, sizeof(ADXL345_sensor));
  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->sensor_name, DEV_NAME, sizeof(sensor->sensor_name) - 1);
  sensor->sensor_name[sizeof(sensor->sensor_name) - 1] = 0;
	// read stored addresses
	sensor->read_device = pread;
	sensor->write_device = pwrite;
	sensor->ms_delay = pdelay	;
	// get ID device
	sensor->read_device(ADXL345_ADDRESS, ADXL345_REG_DEVID, &sensor->dev_id,1);
	sensor->full_res = 0; // default not full resolution
//  sensor->min_delay = 0;
//  sensor->max_value = -156.9064F; /* -16g = 156.9064 m/s^2  */
//  sensor->min_value = 156.9064F;  /*  16g = 156.9064 m/s^2  */
//  sensor->resolution = 0.03923F;  /*  4mg = 0.0392266 m/s^2 */
}
/**************************************************************************/
/*!
  * @brief  set up sensor and make it ready to send data
  * @param  my_sensor:  my_range:
  * @return true: success false: a sensor with the correct ID was not found
*/
/**************************************************************************/
ADXL345_StatusTypeDef ADXL345_begin(ADXL345_sensor *my_sensor, range_t my_range) {
	const char MesureMode   = 0x08; // mesure may begin
	init_Sensor(my_sensor); // Clear the ADXL345_sensor object and set it up
	/* Check device ID */
  if (my_sensor->dev_id != 0xE5) {
    /* No ADXL345 detected ... return false */
    return HAL_ERROR;
  }	
// Select range +/-xG writting my_range in register DATA_FORMAT.
	my_sensor->write_device(ADXL345_ADDRESS, ADXL345_REG_DATA_FORMAT, my_range);
// Mettre le ADXL345 en mode de mesure en écrivant 0x08 dans le registre POWER_CTL.
	my_sensor->write_device(ADXL345_ADDRESS, ADXL345_REG_POWER_CTL, MesureMode);	

	return HAL_OK;	
}
/**
	* @brief set resolution
    @param sensor Pointer
					 resolution to set up	
 */
void set_resolution(ADXL345_sensor *sensor, resolution resol){
	uint8_t value; // value to write in bit D3 DATA_FORMAT register
	uint8_t data_format_value; // register value
	if (resol != sensor->full_res){
		sensor->read_device(ADXL345_ADDRESS, ADXL345_REG_DATA_FORMAT, &data_format_value,1);
		if (resol == FULL){
			value = 0x08; // D3 = 1 in DATA_FORMAT register
			data_format_value |= value;
		}
		else{
			value = 0xF7; // D3 = 0 in DATA_FORMAT register
			data_format_value &= value;
		}
		sensor->write_device(ADXL345_ADDRESS, ADXL345_REG_DATA_FORMAT, data_format_value);
		sensor->full_res = resol;			
	}
}
/**
	* @brief get rsolution
    @param sensor Pointer
    
 */
resolution get_resolution(ADXL345_sensor *sensor){
	
	return sensor->full_res;
};

void setOffset(ADXL345_sensor sensor, int8_t offset, three_axis axis){
	uint8_t read_range = sensor.dev_range;
	if (sensor.full_res != 1){ // not full resolution ?
		// the range bits determine the maximum g range and scale factor if FULL_RES bit is set to 0
		switch (read_range){
			case 0 : // +/- 2g
				
			break;
			case 1 : // +/- 4g
				
			break;
			case 2 : // +/- 8g
				
			break;
			case 3 : // +/- 16g
				
			break;		
		}
	}
};
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
    @brief  Gets the g range registered in the accelerometer structure
						Assume a setrange has already been executed
    @return The current `range_t` value
*/
/**************************************************************************/
range_t getRange(ADXL345_sensor *ps) {

	return ps->dev_range;
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
/**
* @brief Get X, Y, Z
 *
 */
void  Accel_LectureXYZ (ADXL345_sensor *sensor){
	uint8_t data[6];
	
	sensor->read_device(ADXL345_ADDRESS, ADXL345_REG_DATAX0, data,6);
	// Résolution is 10 bits, i.e. 2 bytes. LSB first
  // X, Y, Z are cast in 4 bytes with sign
	if (data[1] & 0x02) // check if <0 number
		sensor->x_value = ((0xFF << 24) | (0xFF << 16) | ((data[1]) << 8) | data[2]);
  else
		sensor->x_value = (((int)data[1]) << 8) | data[0];
	if (data[3] & 0x02)
		sensor->y_value = ((0xFF << 24) | (0xFF << 16) | ((data[3]) << 8) | data[2]);		
  else
		sensor->y_value = (((int)data[3]) << 8) | data[2];
	if (data[5] & 0x02)
		sensor->z_value = ((0xFF << 24) | (0xFF << 16) | ((data[5]) << 8) | data[4]);		
  else
		sensor->z_value = (((int)data[5]) << 8) | data[4];
}
