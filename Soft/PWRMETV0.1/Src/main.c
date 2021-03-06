/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "nrf24l01.h"
#include "eeprom_emul.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define BRD_ROLE 				0x10  				//0x10 - Power Meter
#define BRD_ADDRESS				0x01				// Address may save in ee

struct structPackage{			// Size should not exceed 32Bytes
	uint16_t Current;
	uint16_t Voltage;
	uint16_t EffPower;
} txPackage;

// Transmitter pipe address - Same of one of the pipe of receiver
uint8_t pipeOut[] = { 0xE7, 0xE6, 0xE5, 0xE4, 0xE3 };

NRF24L01_Pins_t NRF24L01_Pins;
uint8_t channel = 80;											// Channel(decimal)
uint8_t payload = sizeof(txPackage);
volatile uint16_t _tim1Ovf;
volatile uint32_t CF_Val,CF1_Val;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void NRF24_Init(void);
void EEProm_Init(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t dataPipe[5];
	uint8_t dataIn[32];
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  EEProm_Init();

  /* USER CODE BEGIN 2 */
  NRF24_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (NRF24L01_Available(dataPipe))
	  {
		  NRF24L01_GetData(dataIn);
		  if((dataIn[0] == BRD_ROLE) && dataIn[1]==BRD_ADDRESS)
		  {
			  if(dataIn[3]==0x05)							//Request Data
			  {
				  NRF24L01_Transmit(&txPackage);
			  }
		  }
	  }
	  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

void EEProm_Init(void)
{
	FLASH_Unlock();
	uint16_t result = EE_Init();

}
void NRF24_Init(void)
{
	NRF24L01_Pins.CE.GPIOx = RF_CE_GPIO_Port;
	NRF24L01_Pins.CE.GPIO_Pin = RF_CE_Pin;
	NRF24L01_Pins.CSN.GPIOx = GPIOA;
	NRF24L01_Pins.CSN.GPIO_Pin = GPIO_PIN_4;
	/* CSN(SS) high = disable SPI */
	HAL_GPIO_WritePin( GPIOA, GPIO_PIN_4, GPIO_PIN_SET );
	/* CE low = disable TX/RX */
	HAL_GPIO_WritePin( RF_CE_GPIO_Port, RF_CE_Pin, GPIO_PIN_RESET );
	NRF24L01_Init( channel, payload, &hspi1, NRF24L01_Pins );
	NRF24L01_SetRF( NRF24L01_DataRate_250k, NRF24L01_OutputPower_0dBm);
	/* Enable auto ack */
	NRF24L01_SetAutoAck(0xFF, 0);			// 0xFF Enable all or if you want enable just pipe 3, insert 3 																									// Have some troble with ack enable... many lost packages
	/* Enable CRC 2bytes */
	NRF24L01_SetCrcLength( NRF24L01_CRC_Disable );

	//APAGAR
	NRF24L01_TestCarrier();

	/* Open a reading pipe with an address*/
	NRF24L01_OpenReadingPipe(1, pipeOut);
	HAL_Delay(1);

	NRF24L01_StartListening();
	HAL_Delay(1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim1)
    {
       _tim1Ovf++; 					// count roll-overs.
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim1){
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
        {
        	CF_Val = TIM1->CCR3;
        } else
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
        {
        	CF1_Val = TIM1->CCR4;
        }

        //if(__HAL_TIM_GET_FLAG(&htim1,TIM_FLAG_UPDATE))
        //    _currTim1CC.word1++;

    }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
