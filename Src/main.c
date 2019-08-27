/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SS_MS5607.h"
#include "SS_S25FL.h"
#include "SS_SCD30.h"
#include "SS_support.h"
#include "SS_it.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adc1_buf[2];
uint16_t adc3_buf[2];
extern uint32_t ADC1_BUF[2];
enum  UART_data{
	Ratunku = 0x01,
	Start_meas = 0x02,
	Erase_data = 0x06,
	logs_to_uart = 0x13,
	End_meas = 0x22,
	Erase_all,
	Start_log,
	End_log,


};
enum FLASH_ID{
	MS5607_pressureID = 1,
	MS5607_altitudeID,
	SCD30_co2ID,
	SCD30_humidityID,
	SCD30_temperatureID,
	ADC1_channel8ID,
	ADC1_channel13ID,
	ADC3_channel10ID,
	ADC3_channel11ID
};
typedef struct{
	uint16_t sizeCO2;
	uint8_t dataCO2[8];
	uint16_t sizeTemp;
	uint8_t dataTemp[8];
	uint16_t sizeHum;
	uint8_t dataHum[8];
}SCD30_VAR;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  //SCD30 variables
  SCD30_VAR SCD30_var;
  SCD30_var.sizeCO2 = 0;
  SCD30_var.sizeHum = 0;
  SCD30_var.sizeTemp = 0;
  float co2_ppm, temperature, relative_humidity;
  uint16_t data_ready;
  int16_t ret;
  uint8_t flash_uart;

  uint32_t page_number = 0;

  //MS5607 variables
  uint8_t refPressIter = 0;
  int32_t ref_pres = 0;
  int32_t average_altitude = 0;
  int16_t interval_in_seconds = 2;
  int32_t altitude_rocket = 0;
  int32_t pressure_rocket = 0;
  int32_t average_pressure = 0;
  uint16_t size = 0;
  uint8_t data[8];

  //ADC variables

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_TIM14_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  //TIM init
  HAL_TIM_Base_Start_IT(&htim2);

  //ADC meas
  HAL_ADC_Start_DMA(&hadc1,adc1_buf,2);
  HAL_ADC_Start_DMA(&hadc3,adc3_buf,2);


  //MS56 INIT
  SS_MS56_init(&ms5607,MS56_PRESS_4096,MS56_TEMP_4096);

  //S25FL Init
  SS_S25FL_reset_init();
  //SS_S25FL_erase_full_chip();
  SS_S25FL_saving_init();
  SS_S25FL_start_logging();

  //Ref press
  while(refPressIter<=10)
  {
	  SS_MS56_read_convert_non_polling(&ms5607);
	  SS_MS56_calculate_average_press(&ms5607, 10);
	  if(ms5607.stage == 0)
		  refPressIter++;
  }
  SS_MS56_set_ref_press(&ms5607);
  ref_pres = SS_MS56_get_ref_press(&ms5607);
  SS_buzzer_start_count(1000, 1600);


//  while(scd_probe() != HAL_OK) {
//
//	  HAL_UART_Transmit(&huart2,"NOK",sizeof("OK"),100);
//      HAL_Delay(100);
//  }
  HAL_UART_Transmit(&huart2,"OK",sizeof("OK"),100);

  scd_set_measurement_interval(interval_in_seconds);
  sensirion_sleep_usec(20000);
  scd_start_periodic_measurement(0);
  HAL_Delay(interval_in_seconds);
  uint32_t time = 0;
  uint32_t postTime = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  time= HAL_GetTick();
	  HAL_UART_Receive(&huart2,&flash_uart,1,100);
	  HAL_GPIO_TogglePin(LOOP_LED_GPIO_Port,LOOP_LED_Pin);
	  //Reset LEd responsible for erase memory
	  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,RESET);
	  HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);

//	  //<Measurements from SCD30>
//	  ret = scd_get_data_ready(&data_ready);
//	  if (ret == STATUS_OK)
//	  {
//		  if (data_ready)
//		  {
//			ret = scd_read_measurement(&co2_ppm, &temperature,&relative_humidity);
//			if (ret != STATUS_OK)
//			{
//				HAL_UART_Transmit(&huart2,"Error reading\n",sizeof("Error reading\n"),100);
//			}
//			else
//			{
//				SCD30_var.sizeCO2 = sprintf(SCD30_var.dataCO2,"%0.2f CO2_ppm\n",co2_ppm);
//				SCD30_var.sizeHum = sprintf(SCD30_var.dataHum,"%0.2f RH\n",relative_humidity);
//				SCD30_var.sizeTemp = sprintf(SCD30_var.dataTemp,"%0.2f Celsius\n",temperature);
//				HAL_UART_Transmit(&huart2,&SCD30_var.dataCO2,SCD30_var.sizeCO2,100);
//				//HAL_UART_Transmit(&huart2,&SCD30_var.dataHum,SCD30_var.sizeHum,100);
//				HAL_UART_Transmit(&huart2,&SCD30_var.dataTemp,SCD30_var.sizeTemp,100);
//
//				//<Save data from SCD30>
//				SS_S25FL_save_variable_u32(SCD30_co2ID,co2_ppm);
//				SS_S25FL_save_variable_u32(SCD30_humidityID,relative_humidity);
//				SS_S25FL_save_variable_u32(SCD30_temperatureID,temperature);
//			}
//		  }
//	  }
//	  else
//	  {
//		HAL_UART_Transmit(&huart2,"Error data flag\n",sizeof("Error data flag\n"),100);
//	  }
//	  //</Measurements from SCD30>
//
//	  //<=Measurements from MS5607>
//	  SS_MS56_DMA_read_convert_and_calculate();
//	  pressure_rocket = ms5607.press;
//	  SS_MS56_calculate_average_press(&ms5607,3);
//	  average_pressure = ms5607.average_press;
//	  //Print pressure
//	  size = sprintf(data,"%d\n\r",average_pressure);
//	  HAL_UART_Transmit(&huart2,data,size,100);
//
//	  altitude_rocket = SS_MS56_get_altitude(&ms5607);
//	  SS_MS56_calculate_average_altitiude(&ms5607,3);
//	  average_altitude = ms5607.average_altitude;
//	  //Print altitude
//	  size = sprintf(data,"%d\n\r",average_altitude);
//	  HAL_UART_Transmit(&huart2,data,size,100);
//	  //</=Measurements from MS5607>
//
//	  //<Save data to flash>
//	  SS_S25FL_save_variable_u32(MS5607_pressureID,pressure_rocket);
//	  SS_S25FL_save_variable_u32(MS5607_altitudeID,altitude_rocket);
//	  SS_S25FL_save_variable_u32(ADC1_channel8ID,adc1_buf[0]);
//	  SS_S25FL_save_variable_u32(ADC1_channel13ID,adc1_buf[1]);
//	  SS_S25FL_save_variable_u32(ADC3_channel10ID,adc3_buf[0]);
//	  SS_S25FL_save_variable_u32(ADC3_channel11ID,adc3_buf[1]);
	  //</Save data to flash>
	  switch(flash_uart){
	  case Start_meas:
		  //Reset LEd responsible for erase memory
		  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,RESET);
		  HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);

		  //<Measurements from SCD30>
		  ret = scd_get_data_ready(&data_ready);
		  if (ret == STATUS_OK)
		  {
			  if (data_ready)
			  {
				ret = scd_read_measurement(&co2_ppm, &temperature,&relative_humidity);
				if (ret != STATUS_OK)
				{
					HAL_UART_Transmit(&huart2,"Error reading\n",sizeof("Error reading\n"),100);
				}
				else
				{
					SCD30_var.sizeCO2 = sprintf(SCD30_var.dataCO2,"%0.2f CO2_ppm\n",co2_ppm);
					SCD30_var.sizeHum = sprintf(SCD30_var.dataHum,"%0.2f RH\n",relative_humidity);
					SCD30_var.sizeTemp = sprintf(SCD30_var.dataTemp,"%0.2f Celsius\n",temperature);
					HAL_UART_Transmit(&huart2,&SCD30_var.dataCO2,SCD30_var.sizeCO2,100);
					//HAL_UART_Transmit(&huart2,&SCD30_var.dataHum,SCD30_var.sizeHum,100);
					HAL_UART_Transmit(&huart2,&SCD30_var.dataTemp,SCD30_var.sizeTemp,100);

					//<Save data from SCD30>
					SS_S25FL_save_variable_u32(SCD30_co2ID,co2_ppm);
					SS_S25FL_save_variable_u32(SCD30_humidityID,relative_humidity);
					SS_S25FL_save_variable_u32(SCD30_temperatureID,temperature);
				}
			  }
		  }
		  else
		  {
			HAL_UART_Transmit(&huart2,"Error data flag\n",sizeof("Error data flag\n"),100);
		  }
		  //</Measurements from SCD30>

		  //<=Measurements from MS5607>
		  SS_MS56_DMA_read_convert_and_calculate();
		  pressure_rocket = ms5607.press;
		  SS_MS56_calculate_average_press(&ms5607,3);
		  average_pressure = ms5607.average_press;
		  //Print pressure
		  size = sprintf(data,"%d\n\r",average_pressure);
		  HAL_UART_Transmit(&huart2,data,size,100);

		  altitude_rocket = SS_MS56_get_altitude(&ms5607);
		  SS_MS56_calculate_average_altitiude(&ms5607,3);
		  average_altitude = ms5607.average_altitude;
		  //Print altitude
		  size = sprintf(data,"%d\n\r",average_altitude);
		  HAL_UART_Transmit(&huart2,data,size,100);
		  //</=Measurements from MS5607>

		  //<Save data to flash>
		  SS_S25FL_save_variable_u32(MS5607_pressureID,pressure_rocket);
		  SS_S25FL_save_variable_u32(MS5607_altitudeID,altitude_rocket);
		  SS_S25FL_save_variable_u32(ADC1_channel8ID,adc1_buf[0]);
		  SS_S25FL_save_variable_u32(ADC1_channel13ID,adc1_buf[1]);
		  SS_S25FL_save_variable_u32(ADC3_channel10ID,adc3_buf[0]);
		  SS_S25FL_save_variable_u32(ADC3_channel11ID,adc3_buf[1]);
		  //</Save data to flash>
		  break;
	  case Erase_data:
		  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,SET);
		  SS_S25FL_erase_only_written_pages();
		  SS_S25FL_reset_init();
		  SS_S25FL_saving_init();
		  SS_S25FL_start_logging();
		  flash_uart = 0;
		  break;
	  case logs_to_uart:
		  SS_buzzer_start_count(1500, 2500);
		  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,RESET);
		  SS_S25FL_stop_logging();
		  SS_S25FL_read_data_logs_to_uart(MS5607_pressureID);
//		  SS_S25FL_read_data_logs_to_uart(MS5607_altitudeID);
//		  SS_S25FL_read_data_logs_to_uart(SCD30_co2ID);
//		  SS_S25FL_read_data_logs_to_uart(SCD30_humidityID);
//		  SS_S25FL_read_data_logs_to_uart(SCD30_temperatureID);
//		  SS_S25FL_read_data_logs_to_uart(ADC1_channel8ID);
//		  SS_S25FL_read_data_logs_to_uart(ADC1_channel13ID);
//		  SS_S25FL_read_data_logs_to_uart(ADC3_channel10ID);
//		  SS_S25FL_read_data_logs_to_uart(ADC3_channel11ID);
		  SS_buzzer_start_count(1000,10000);
		  SS_S25FL_start_logging();
		  flash_uart = 0;
		  break;
	  case End_meas:
		  flash_uart = 0;
		  break;
	  case Erase_all:
		  SS_buzzer_start_count(1500, 2500);
		  SS_S25FL_reset_init();
		  SS_S25FL_erase_full_chip();
		  SS_S25FL_saving_init();
		  SS_S25FL_start_logging();
		  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,SET);
		  SS_buzzer_start_count(1500, 3500);
		  flash_uart = 0;
		  break;
	  case Start_log:
		  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,RESET);
		  SS_S25FL_start_logging();
		  flash_uart = 0;
		  break;
	  case End_log:
		  SS_S25FL_stop_logging();
		  flash_uart = 0;
		  break;
	  case Ratunku:
		  for(page_number = 501;page_number<=34044;page_number++)
		  {
		  		SS_S25FL_dump_page_to_uart(page_number);
		  }
		  break;

	  }
//	  postTime = HAL_GetTick() - time;
//	  size = sprintf(data,"%d\n\r",postTime);
//	  HAL_UART_Transmit(&huart2,data,size,100);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
