/*
 * SS_it.c
 *
 *  Created on: 08.02.2018
 *      Author: Tomasz
 */
#include "SS_it.h"
#include "SS_support.h"
#include "SS_MS5607.h"
#include "SS_S25FL.h"

extern uint8_t conversion_ongoing;
void set_beeps(uint8_t beep)
{
	beeps = beep;
}
void set_gap(uint8_t _gap)
{
	if(_gap > 0)
	gap = _gap;
	else gap=1;
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t cnt;
	if(htim == &BUZZER_TIM)
	{
		cnt++;
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, RESET);
		if(cnt % gap == 0)
		{
			cnt = 0;
			if(beeps > 0)
			{
				if(beeps < 255)
					beeps--;
				HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
			}
		}

	}
	if(htim->Instance == HAL_TIM_INSTANCE)
	{
		 SS_S25FL_inc_timestamp_handler();
	}
}


/**
  * @brief  This function handles after SPI transmission interrupt.
  * @note   It is hardware interrupt.
  * @retval None
  */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi == &HSPI_MS56)
	{
		if((HAL_GPIO_ReadPin(MS56_CS_GPIO_Port, MS56_CS_Pin) == GPIO_PIN_RESET) && ms5607.sequence_flag == 1)
		{
			ms5607.sequence_flag = 2;
			SS_MS56_CS_DISABLE();
		}
		else if((HAL_GPIO_ReadPin(MS56_CS_GPIO_Port, MS56_CS_Pin) == GPIO_PIN_RESET) && ms5607.sequence_flag == 3)
		{
			ms5607.sequence_flag = 0;
			if(ms5607.comp_type == press)
			{
				ms5607.comp_type = temp;
				SS_MS56_DMA_adc_read_RX_press();
			}
			else
			{
				ms5607.comp_type = press;
				SS_MS56_DMA_adc_read_RX_temp();
			}

		}

		/* Error report.  */
		if(HAL_SPI_GetState(&HSPI_MS56) == HAL_SPI_STATE_ERROR)
			ms5607.result = 1;
		else
			ms5607.result = 0;
	}

	if (hspi->Instance ==  HAL_SPI_INSTANCE) SS_S25FL__dma_transfer_cs_handler();

}

/**
  * @brief  This function handles after SPI receive interrupt.
  * @note   It is hardware interrupt.
  * @retval None
  */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi == &HSPI_MS56)
	{
		/* Check if its time to temp conversion or to end of conversion.  */
		if(ms5607.comp_type == temp)
		{
			SS_MS56_CS_DISABLE();
			ms5607.sequence_flag = 4;
		}
		else
		{
			SS_MS56_CS_DISABLE();
			conversion_ongoing = 0;

		}

		/* Error report.  */
		if(HAL_SPI_GetState(&HSPI_MS56) == HAL_SPI_STATE_ERROR)
			ms5607.result = 1;
		else
			ms5607.result = 0;
	}

	if (hspi->Instance ==  HAL_SPI_INSTANCE) SS_S25FL__dma_transfer_cs_handler();

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
}


