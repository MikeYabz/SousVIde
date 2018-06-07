/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "adc.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "i2c-lcd.h"
#include "string.h"
#include "math.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


/************************************************************************************/
//	*		Thermistor Temperature Polling
/************************************************************************************/
float getTemp(void)
{
	#define Resistor 1487 //thermistor is the top part of a resistor divider, this value is the bottom fixed value
	
	uint16_t thermistorReading;
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc,5);
	thermistorReading = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);
	
	double thermistorResistance = (6142500/(double)thermistorReading)-Resistor;
	double y = 3969/(log(thermistorResistance/(10000*exp(-3969/298.15))))-273.15;
	return (float)y;
}


/*************************************END********************************************/



/************************************************************************************/
//	*		Calculate Hours, Minutes and Seconds Components from Total Seconds
/************************************************************************************/
void getTime(uint32_t totalSeconds,uint16_t* hours,uint16_t* minutes,uint16_t* seconds)
{
	*hours = totalSeconds / 3600;
	*minutes = (totalSeconds / 60) % 60;
	*seconds	= totalSeconds % 60;
}


/*************************************END********************************************/



/************************************************************************************/
//	*		Calculate Total Seconds from Hours, Minutes and Seconds Components
/************************************************************************************/
void setTime(uint32_t* totalSeconds,uint16_t hours,uint16_t minutes,uint16_t seconds)
{
	*totalSeconds = seconds+(((hours*60)+minutes)*60);
}
/*************************************END********************************************/



/************************************************************************************/
//	*		Update LCD
/************************************************************************************/
void updateLCD(uint8_t lengthTop,uint8_t lengthBottom,char* topStr,char* bottomStr)
{
	lcd_send_cmd (0x01);  // clear the display
	HAL_Delay (5);
	
	lcd_send_cmd (0x80);  // goto row 1
	lcd_transmit(lengthTop,topStr);
	
	lcd_send_cmd (0xc0); //go to row 2
	lcd_transmit(lengthBottom,bottomStr);	
}



/*************************************END********************************************/



/************************************************************************************/
//	*		Button Press
/************************************************************************************/
uint8_t getButtonPress (void)
{
	#define queueSize 100
	static uint8_t queue[queueSize] = {0};
	
				//GPIO Readings
	static uint8_t buttonRead;
	if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6) == 1 && HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7 == 1))
	{
		buttonRead = 1;
	}
	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6) == 1)
	{
		buttonRead = 2;
	}
	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7) == 1)
	{
		buttonRead = 3;
	}
	else
	{
		buttonRead = 0;
	}				
	
		//Queue Calculations
	for (uint8_t i=0;i<queueSize-1;i++)
	{
		queue[i] = queue[i+1];
	}
	queue[queueSize-1] = buttonRead;
	static uint8_t totalButtonReads[4];
	for(uint8_t i=0;i<4;i++)
	{
		totalButtonReads[i] = 0;		;
	}		
	for (uint8_t i=0;i<queueSize;i++)
	{
		if (queue[i] == 0) totalButtonReads[0]++;
		else if (queue[i] == 1) totalButtonReads[1]++;
		else if (queue[i] == 2) totalButtonReads[2]++;
		else if (queue[i] == 3) totalButtonReads[3]++;
	}
	static uint8_t filteredReading;		
	if (totalButtonReads[0] > totalButtonReads[1])
	{
		filteredReading  = 0;
	}
	else
	{
		filteredReading = 1;
	}	
	if (totalButtonReads[2] > totalButtonReads[filteredReading])
	{
		filteredReading = 2;
	}
	if (totalButtonReads[3] > totalButtonReads[filteredReading])
	{
		filteredReading = 3;
	}								
	
		//Schmitt Trigger Calculations
	static uint8_t buttonPress = 0;
	if ((buttonPress == 0 ) && (filteredReading != 0))
	{
		if (totalButtonReads[filteredReading] > queueSize*70/100)
		{
			buttonPress = filteredReading;
		}			
	}
	else if (buttonPress == filteredReading)
	{
		if (totalButtonReads[filteredReading] < queueSize*30/100)
		{
			buttonPress = 0;
		}			
	}
	return buttonPress;
}



/*************************************END********************************************/



/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_ADC_Init();

  /* USER CODE BEGIN 2 */





/************************************************************************************/
//	*		Init LCD and Times
/************************************************************************************/
		HAL_Delay(100);	//short startup delay
		lcd_init();	//initialize the LCD;
		
		uint16_t hours = 0;	//hour component of time, used for easy UI 
		uint16_t minutes = 0;	//minute component of time, used for easy UI 
		uint16_t seconds = 0;	//second component of time, used for easy UI 
		uint32_t totalSeconds = 0;	//total seconds, used internally for all calculations		

		char outputTop[40];
		uint8_t lengthTop;
		char outputBottom[40];
		uint8_t lengthBottom;		
		int leftTemp;	//value left of decimal, done so only 1 decimal place of the float is shown
		int rightTemp;	//value right of decimal, done so only 1 decimal place of the float is shown
		int leftSetTemp;	//value left of decimal, done so only 1 decimal place of the float is shown
		int rightSetTemp;	//value right of decimal, done so only 1 decimal place of the float is shown	
/*************************************END********************************************/		
		
		
/************************************************************************************/
//	*		Init Temp Variables
/************************************************************************************/
		float temp;	//temperature
		float filteredTemp;	//temperature after LPF
		float setTemp = 25;	//goal temperature, defaults 25
		filteredTemp = getTemp(); 	//initialize filtered temp as it uses hysterysis
/*************************************END********************************************/		
		
		
/************************************************************************************/
//	*		Init State Machine
/************************************************************************************/		
		enum states{	//input states are where values are being changed;
			startup = 0, 
			main = 1, 
			mainInput = 2, 
			Extra = 3, 
			ExtraInput = 4 
		};
		enum states state = startup;
		enum states pastState = startup;
/*************************************END********************************************/		
		
		


		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */		

		
		
		
		HAL_Delay(1);		
		
/************************************************************************************/
//	*		Time Update, Temperature Read and Button press
/************************************************************************************/	
			//Time
		static uint32_t secondTimestamp = 0;
		if (HAL_GetTick() - secondTimestamp > 1000)	//decrements totalSeconds every second 
		{
			secondTimestamp = HAL_GetTick();
			totalSeconds = totalSeconds - 1;
		}
		getTime(totalSeconds,&hours,&minutes,&seconds);	//updateLCD hours/minuets and seconds	
		
			//Temperature
		temp = getTemp();  //read Thermistor		
		filteredTemp = (temp + filteredTemp * 4)/5;	//LPF for more stability in readings	
		leftTemp = (int) filteredTemp;	//remove anything right of the decimal
		rightTemp = (int)((filteredTemp - (int) filteredTemp)*10);	//remove whatevers left of the decimal
		leftSetTemp = (int) setTemp;	//remove anything right of the decimal
		rightSetTemp = (int)((setTemp - (int) setTemp)*10);	//remove whatevers left of the decimal
			
			//Button Press
		getButtonPress();
/*************************************END********************************************/
		
		
		
/************************************************************************************/
//	*		Next State Logic
/************************************************************************************/	
		switch(pastState)
		{
			case startup:
				if (HAL_GetTick() > 2000)	//show for the first 2 seconds or so
				{
					state = main;
				}
				break;
			case 1:
				break;
			case 2:
				break;
			case 3:
				break;
			case 4:
				break;
			default:
				break;
		}
/*************************************END********************************************/
		
		
		
/************************************************************************************/
//	*		State Logic
/************************************************************************************/		
		switch(state)
		{
			case startup:
				lengthTop = sprintf(outputTop,"Hello!");
				lengthBottom = sprintf(outputTop,"M.Yabz-SousVide");
				updateLCD(lengthTop,lengthBottom,outputTop,outputBottom);
				break;
			case 1:
				lengthTop = sprintf(outputTop,"%ih %im %is",hours,minutes,seconds);//output time		
				lengthBottom = sprintf(outputBottom,"T:%i.%i Set:%i.%i",leftTemp,rightTemp,leftSetTemp,rightSetTemp);//output current temp and set temp	
				updateLCD(lengthTop,lengthBottom,outputTop,outputBottom);
				break;
			case 2:
				lengthTop = sprintf(outputTop,"%ih %im %is",hours,minutes,seconds);//output time		
				lengthBottom = sprintf(outputBottom,"T:%i.%i Set:%i.%i",leftTemp,rightTemp,leftSetTemp,rightSetTemp);//output current temp and set temp	
				updateLCD(lengthTop,lengthBottom,outputTop,outputBottom);
				break;
			case 3:
				break;
			case 4:
				break;
			default:
				break;
		}
		pastState = state;
/*************************************END********************************************/		
		
		
		
/************************************************************************************/
//	*		Heater On/Off Logic
/************************************************************************************/		
		
		//Only setup for one Heater now, PID will be implented here
		
		if (filteredTemp < setTemp)
		{
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
		}	
/*************************************END********************************************/		
		
		
		
		
		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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
