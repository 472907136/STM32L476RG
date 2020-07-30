/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "math.h"
#include "ADXL345_Header.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// print redefined
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define DEVID 0x00; // device ID
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
ADXL345_sensor accel_sensor;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
ADXL345_StatusTypeDef user_write_i2c(uint8_t sensor_address, int8_t reg_address, int8_t val);
ADXL345_StatusTypeDef user_read_i2c(uint8_t sensor_address, uint8_t reg_address, uint8_t *data, uint8_t len);
void user_ms_delay (uint32_t u32delay);

void ADXL345SelfTest(ADXL345_sensor *sensor);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1 ,1000);
	return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
	// init accelerometer sensor structure with user functions
	accel_sensor.read_device = user_read_i2c;
	accel_sensor.write_device = user_write_i2c;
	accel_sensor.ms_delay = user_ms_delay;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	if (ADXL345_begin(&accel_sensor, ADXL345_RANGE_16_G) != HAL_ERROR)
		printf ("Device ID : 0x%X\r\n",accel_sensor.dev_id);
	else
		printf ("Not device with ID = 0xE5 detected !");
  
	/* ---- self test sequence (no instruction to device before this line) ---- */
  /* -------------------- AN-1077 -------------------------------------------- */
  // Vs = ON - Vdd i/o = ON
  // wait 1ms
  // initial command sequence
  //    +-16g and 13-bit mode ( 0x0B in register DATA_FORMAT )
  //    start mesurement ( 0x08 in register POWER_CTL )
  //    enable data-ready interrupt ( 0x80 in register INT_ENABLE )
  // wait 1ms ( when ODR = 100hz)
  // take 100 data points and average (it's to minimize effects of noise)
  // activate self_test ( 0x8B in register DATA_FORMAT )
  // wait 1ms
  // take 100 data points and average (it's to minimize effects of noise)
  // inactivate self-test ( 0x0B in register DATA_FORMAT )
  // calculate self-test delta and compare it to datasheet limits

  printf("Test conditions are (refer to AN-1077) :\r\n");
  printf("\t * +-16g with 10 bits resolution\r\n");
  printf("\t * hundred mesures are averaged in 100hz\r\n");
  printf("\t * scale_factor with 3.3v : xy  1.77 - z   1.47\r\n");
  printf("Now testing...\r\n\r\n");
  ADXL345SelfTest(&accel_sensor); // already running in +-16g mode
  
  while (1)
  {

   HAL_Delay(200);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C3;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief  Write to the device
 * @param  Sensor address, register address to write, value to be written
 * @return true: success false: write failed
 */
ADXL345_StatusTypeDef user_write_i2c(uint8_t sensor_address, int8_t reg_address, int8_t val){
	uint8_t t_data[2];
	t_data[0] = reg_address; // Envoyer l'adresse de registre
	t_data[1] = val; // Envoyer la valeur à écrire.
	
	return HAL_I2C_Master_Transmit(&hi2c3,sensor_address,t_data,2,10);
};
/**
 * @brief Read from the device
 * @param Sensor address, register address to read, pointer to stored data, len : number of char to receive
 * @return true: success false: read failed
 */
ADXL345_StatusTypeDef user_read_i2c(uint8_t sensor_address, uint8_t reg_address, uint8_t *data, uint8_t len){
	uint8_t t_data[1];
	t_data[0] = reg_address; // Envoyer l'adresse du registre à lire
	
	if (HAL_I2C_Master_Transmit(&hi2c3,sensor_address,t_data,1,10) != HAL_OK)
	
		return HAL_ERROR;
	else

		return HAL_I2C_Master_Receive(&hi2c3,ADXL345_ADDRESS,(uint8_t*)data,len,10);
}
/**
  * @brief  This function is user delay function.
  * @retval None
  */
void user_ms_delay (uint32_t u32delay){
	HAL_Delay(u32delay);
};


/* https://www.best-microcontroller-projects.com/adxl345.html */

//static int8_t showAngles = 0; // Display operation

// Get readings for self test - A set of n averaged results.
void ADXL345GetAvgN(int16_t *xi,int16_t *yi,int16_t *zi,uint8_t n) {
int16_t x,y,z;
int32_t ax,ay,az;
uint8_t i;

    ax=0; ay=0; az=0;
    for(i=0;i<n;i++) {
			accel_lectureXYZ(&accel_sensor);
			x = accel_sensor.x_value;
			y = accel_sensor.y_value;
			z = accel_sensor.z_value;
      ax += x; ay += y; az +=z;
      HAL_Delay(100); // Don't read faster than 100Hz: (10*100ms=1s 1s/10=100ms)
    }
    *xi = ax/n; *yi = ay/n; *zi = az/n;
}

void print_minmax(int vmin,int vmax) {
  printf("(%d ~ %d )", vmin, vmax);
}

// This function setsup the relevant parameters while preserving them e.g. range
void ADXL345SelfTest(ADXL345_sensor *sensor) {

  int16_t avg_x=0, avg_y=0, avg_z=0;
  int16_t avg_stx=0, avg_sty=0, avg_stz=0;
  int16_t x,y,z;

  // Setup operating mode required for self test.
  set_resolution(sensor, DEFAULT);
  // power control : Start mesurement(0x08 -> 0x2D)
  start_mesurement(sensor);
  // defaultdata_rate(ADXL345_DATARATE_100_HZ) is ok
  //AN-1077 p22 the part must be in normal power operation (LOW_POWER bit = 0 in BW_RATE register, Address 0x2C)
  ADXL345GetAvgN(&avg_x, &avg_y, &avg_z, 100);
  self_test(sensor, ON); // now activate self-test
  HAL_Delay(2); // wait 1.1ms
  ADXL345GetAvgN(&avg_stx, &avg_sty, &avg_stz, 100);
  self_test(sensor, OFF); // now desactivate self-test

  // 3V3 error limit scale factors.
  #define scale_factor_xy  1.77
  #define scale_factor_z   1.47
  
  // Display results
  //x = (avg_stx/scale_factor_xy)-avg_x;
  x = avg_stx-avg_x; y = avg_sty-avg_y; z = avg_stz-avg_z;
  
  float vmin,vmax;

  //Value limits are from ADXL345 datasheet - Tables 14 and 18.
  //vmin = 6 * scale_factor_xy; vmax = 67 * scale_factor_xy;
  vmin = 6 * scale_factor_xy; vmax = 67 * scale_factor_xy;
  printf( (x>=vmin && x <vmax) ? "X PASS " : "X FAIL " );
  print_minmax(vmin,vmax);
  printf("\tdelta-x\t %d\t\r\n", x);

  vmin = -67 * scale_factor_xy; vmax = -6 * scale_factor_xy;
  printf( (y>=vmin && y <vmax) ? "Y PASS " : "Y FAIL " );
  print_minmax(vmin,vmax);
  printf("\tdelta-y\t %d\t\r\n", y);
  
  vmin = 10 * scale_factor_z; vmax = 110 * scale_factor_z;
  printf( (z>=vmin && z <vmax) ? "Z PASS " : "Z FAIL " );
  print_minmax(vmin,vmax);
  printf("\tdelta-z\t %d\t\r\n", z);
  printf("\r\n");
  printf(" Avg-x : %d - avg_stx : %d\n\r", avg_x, avg_stx);  
  printf(" Avg-y : %d - avg_sty : %d\n\r", avg_y, avg_sty);  
  printf(" Avg-z : %d - avg_stz : %d\n\r", avg_z, avg_stz);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	printf("Unknown error occurs, perhaps divided by 0 ?\r\n");
	HAL_Delay(2000);
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
