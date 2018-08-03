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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "limits.h"
#include "mydefs.h"
#include "comm_USART.h"
#include "IO_LIB.h"
#include "TIK_CON.h"
#include "Isokinetic.h"
#include "LoadSensor.h"
#include "PosSensor.h"
#include "ServoControl.H"
#include "DataConv.h"
#include "EQUIPMENT.H"
#include "commModbusRTU.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
BOOL	fInt_Scheduler;
BOOL	f2msecTick;
BOOL	f5msecTick;
BOOL	f10msecTick;
BOOL	fLEDToggleTick;

BOOL	t50usecToggle;
BOOL	fForceCalcTick;
BOOL	fAdcConv;

BYTE b5msCnt;
int		iLEDTogglCnt = TIM_500MSEC;
int		iCPUTick1msecCnt;
BYTE Uart3_txBuff[7];
BYTE Spi2_txBuff[10];
BYTE Spi2_rxBuff[10];

UINT lPulseCnt;
BYTE tempData=4;
BYTE bAlmRstCnt;
int iUsart1IrqCnt;
BYTE bSpiData[2]={0x55,0x55};
//uint16_t ADCValue; //ADC Readings
static BYTE hyperBuffIndex;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void VariableInit(void);
void makeTimeTick(void);
void SensorProc(void);
void outServoPulse( void );
void SystemIOProcess(void);
int Pulse_Out(ULONG lPulseNum);
void Isr_System_HyperTerm(BYTE tmpChar);
void Isr_Uart1(BYTE tmpChar);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//	int iPulseOut;
	if (htim->Instance == TIM2)	{		// 1msec
		fInt_Scheduler = TRUE;
//		CPU_LEDOff();
//		HAL_SPI_Transmit(&hspi1,bSpiData,1,0);
//		CPU_LEDOn();

//		wSpiData = 0x5555;
//		CPU_LEDToggle();
	}
	if (htim->Instance == TIM3)	{		// 50usec
		if (t50usecToggle == 1) {
			t50usecToggle = 0;
//			iPulseOut = Pulse_Out(tempData);
//			HAL_ADC_Start(&hadc1);
			getLoadData() ;
//			outServoPulse();
		}
		else {
			t50usecToggle = 1;
//			ADC_Value = HAL_ADC_GetValue(&hadc1);
		}
		outServoPulse();

	}

}

	
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	BYTE tmpChar;
	if (huart->Instance == USART1) {
		tmpChar = Uart1_rxBuff[Uart1_rxBuffIndex];
		Isr_Uart1(tmpChar);
		HAL_UART_Receive_IT(huart, Uart1_rxBuff+Uart1_rxBuffIndex, 1);
	}
	if (huart->Instance == USART2) {
		tmpChar = Uart2_rxBuff[Uart2_rxBuffIndex];
		if (CommPCMode == HYPER_TERMINAL )
			Isr_System_HyperTerm(tmpChar);
		else {
			if (Uart2_rxBuff[Uart2_rxBuffIndex] == ENQ) {
				Uart2_rxBuffIndex = 0;
			}
			else if (Uart2_rxBuff[Uart6_rxBuffIndex] != EOT) {
				Uart2_rxBuffIndex++;
			}
			else {
				if (Uart2_rxBuffIndex == 0)
				{
					if (hyperBuffIndex) {
						hyperBuffIndex = RESET;
						if (tmpChar == 't' )
							CommPCMode = HYPER_TERMINAL ;
					}
					else if (tmpChar == 'h' )
						hyperBuffIndex = SET;
				}
				else
				{	
					Uart2_rxBuff[Uart2_rxBuffIndex] = tmpChar;	// 수신 데이터를 버퍼에 저장
					Uart2_rxBuffIndex++;
				}
				fUart2Received = SET;
				Uart2_rxBuffSize = Uart2_rxBuffIndex;
				Uart2_rxBuffIndex = 0;
			}
		}
		HAL_UART_Receive_IT(huart, Uart2_rxBuff+Uart2_rxBuffIndex, 1);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		iUsart1IrqCnt++;
	}
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	fAdcConv = SET;
//	ADC_Value = HAL_ADC_GetValue(&hadc1);
//	HAL_ADC_Start_IT(&hadc1);

}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
BOOL fCheck;
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_IWDG_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_SPI3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */
//  HAL_NVIC_DisableIRQ(DMA1_Stream4_IRQn);	// Code Generation에서 자동으로 DMA 인터럽트를 Enable하므로 수동으로 다시 Disable
//  HAL_NVIC_DisableIRQ(DMA1_Stream3_IRQn);	// Code Generation에서 자동으로 DMA 인터럽트를 Enable하므로 수동으로 다시 Disable
//  HAL_NVIC_DisableIRQ(TIM2_IRQn);	// Code Generation에서 자동으로 DMA 인터럽트를 Enable하므로 수동으로 다시 Disable
//  HAL_NVIC_DisableIRQ(TIM3_IRQn);	// Code Generation에서 자동으로 DMA 인터럽트를 Enable하므로 수동으로 다시 Disable
//	int1msecTickOffset = HAL_GetTick();
//		  	HAL_ADC_Start_DMA(&hadc, (uint32_t*) ADCReadings, 3);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_UART_Receive_IT(&huart1, Uart1_rxBuff, 1);
	HAL_UART_Receive_IT(&huart2, Uart2_rxBuff, 1);
	HAL_NVIC_EnableIRQ(ADC_IRQn);
	HAL_ADC_Start_IT(&hadc1);
	

	VariableInit();
//	lServoPulseAddCnt = 800;	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //	SensorProc();
//		CPU_LEDToggle();
	if (fInt_Scheduler == TRUE) {
		fInt_Scheduler = FALSE;

		Scheduler();
//		getPosData();
		makeTimeTick();
		LoadDataCollect() ;
		SystemIOProcess();
		
  	}
	if (fCheck == SET) {
		fCheck = RESET;
		HAL_SPI_Abort(&hspi1);
	}

	if (fAdcConv == SET) {
		fAdcConv = RESET;
		ADC_Value = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Start_IT(&hadc1);
	}

	ConsoleProcess();
	
	if (f5msecTick) {
		f5msecTick = RESET;
		ServoOperationControl();
//		SensorProc();
		b5msCnt++;
		switch ( b5msCnt)
		{
			case 1 : 
				SensorAmpControl(Uart3_txBuff);
				Uart3_TransmitStart(Uart3_txBuff, 7);
				break;
				
			case 2 :
				SensorCal() ;
				break;
			
			case 3:
				PosSensorProc(Spi2_rxBuff);	// Position Sensor Data Read
				HAL_SPI_TransmitReceive_IT(&hspi2, Spi2_txBuff, Spi2_rxBuff, 10);	// SPI Pos sensor drive
				break;
			case 4:
				debugOut();
				b5msCnt=0;
				break;	
		}
		

	}

	if (fForceCalcTick)
	{
		fForceCalcTick =RESET;
		conv_angle_Force_Torque ();
	}

	if (f10msecTick) {
		f10msecTick = RESET;
		if (Cnt10msec < MAX_LOGGING_NUM) {
			UpdateLoggingData(Cnt10msec);
			Cnt10msec++;
		}
		
		if (CommPCMode == PC_PROGRAM ) {
			if (bPCCommFailCnt < PC_COMM_FAIL_LIMIT	) {
				bPCCommFailCnt++;
			}
			else 
				FPCCommFail = SET;
		}
		else
			FPCCommFail = RESET;
		

	}	
	if (fLEDToggleTick) {
		fLEDToggleTick = RESET;
		CPU_LEDToggle();
	}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	HAL_IWDG_Refresh(&hiwdg);


  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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

/** NVIC Configuration
*/
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* SPI2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SPI2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(SPI2_IRQn);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;
  ADC_InjectionConfTypeDef sConfigInjected;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = TIM_CLOCK;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIM_1000USEC-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = TIM_CLOCK;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = TIM_50USEC-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPD_POS_Pin|SERVO_PULSE_DIR_Pin|LED_CPU_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SERVO_STOP_Pin|SERVO_SVON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPD_POS_Pin SERVO_PULSE_DIR_Pin */
  GPIO_InitStruct.Pin = SPD_POS_Pin|SERVO_PULSE_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ESTOP_Pin SERVO_ALM_Pin LO_LIMIT_Pin HI_LIMIT_Pin */
  GPIO_InitStruct.Pin = ESTOP_Pin|SERVO_ALM_Pin|LO_LIMIT_Pin|HI_LIMIT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SERVO_STOP_Pin SERVO_SVON_Pin */
  GPIO_InitStruct.Pin = SERVO_STOP_Pin|SERVO_SVON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_CPU_Pin */
  GPIO_InitStruct.Pin = LED_CPU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_CPU_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void	VariableInit()
{
	int i;
	
	bPCCommFailCnt = 0;
	FPCCommFail = RESET;
	
	CommPCMode = PC_PROGRAM;
	CommPCMode = HYPER_TERMINAL;
	initEquipVar();	
	isotonicLoad = 2000;

	Spi2_txBuff[0] = 0xAA;
	for(i=1;i<10;i++)
	{
		Spi2_txBuff[i] = 0xFF;
	}
	InitSensorRegisterData();
	FlInitSensorCal = SET;

	ServoSpdModeOff();


}
// 1msec 
void makeTimeTick(void)
{
	static int i1msecCnt;
	int idx;
	
	if ((i1msecCnt%2) == 0)  {
		f2msecTick = SET;
	}
	if (bAlmRstCnt) {
		bAlmRstCnt--;
	}	

	idx = i1msecCnt%5;
	if ((i1msecCnt%5) == 0)  {
		f5msecTick = SET;
	}
	else if (idx == 1)	{
		fForceCalcTick=SET;
	}	
	else if (idx == 3 )
	{
		fForceCalcTick=SET;
//		Uart1_TransmitStart(6);
//		if (DebugOutIdx ==4)  rDBGU_THR = '=';
	}
	
	if ((i1msecCnt%10) == 0)  {
		f10msecTick = SET;
	}
	if ((i1msecCnt%iLEDTogglCnt) == 0)  {
		fLEDToggleTick = SET;
	}
	i1msecCnt++;
	i1msecCnt %= 10000;
	iCPUTick1msecCnt++;
	
	if (Uart1TimeoutCnt < 1000){
		Uart1TimeoutCnt++;
	}
	if (bCntEOF < 0xFF) {
		bCntEOF++;
	}
	// Inverter 통신단절 체크
	if(ModCommDisconTimer < MOD_COMM_DISCONN_CHK_TIME )
	{
		fModCommFail = RESET;
		ModCommDisconTimer++;
	}
	else 
	{	
		fModCommFail = SET;				// 통신단절됨
	}	
	
	if (ServoStateTimeoutCnt < USHRT_MAX) {
		ServoStateTimeoutCnt++;
	}
}

/***********************************************************
	Input : 
		U16 wPulseNum : 발생시켜야 할 총 펄스 수
	Return :
		U16 :총 발생시킨 펄스 수
*************************************************************/

const UWORD wPulsePattern[9] =
	{
		0x0000,
		0x8000,
		0xA000,
		0xA800,
		0xAA00,
		0xAA80,
		0xAAA0,
		0xAAA8,
		0xAAAA
	};
BYTE bPulsePattern[18] =
	{
		0x00,	// 0
		0x00,
		0x02,	// 1
		0x00,
		0x02,	// 2
		0x02,
		0x02,	// 3
		0x50,
		0x22,	// 4
		0x22,
		0x50,	// 5
		0x52,
		0x52,	// 6
		0x52,
		0x55,	// 7
		0x52,
		0x55,	// 8
		0x55
	};		
int Pulse_Out(ULONG lPulseNum)
{
	int i;

	if (lPulseNum > 8 ) {
		i = 8;
	}
	else {
		i = (int)lPulseNum;
	}
	HAL_SPI_Transmit(&hspi1,bPulsePattern+i*2,1,0);
	return i;
}

//#define  PULSE_TRAIN_NO		50	// 5msec주기 펄스 지령 update, 100usec단위 펄스 출력
#define  PULSE_TRAIN_NO		100	// 5msec주기 펄스 지령 update, 50usec단위 펄스 출력, 5msec마다 Pulse Out수행 회수

// lServoPulseAddCnt : 외부에서 추가적으로 발생한 위치 지령(펄스 수)
// lPulseCnt : 최종 남은 출력 펄스 수
// PlsTrainCnt : lServoPulseAddCnt를 모두 소모하기 위한 펄스 트레인 수
// RgrPulseCnt : 5msec당 출력해야 하는 펄스 수
// RemainPulse : 마지막 펄스 5msec주기에서 출력해야 하는 나머지 펄스 수
// lServoPulseAddCnt 최대값  800 : 
//		5msec주기로 지령이 나가고 100usec단위로 최대 8개 펄스가 나가기 때문에 8*PULSE_TRAIN_NO
void outServoPulse( void )
{
	static short RgrPulseCnt, RgrPulseCompenPeriod,RemainPulse;
	static short PlsTrainCnt, PlsRemainIntvl;
	int iPulseOut;
	char SendingPulse;
//	lServoPulseAddCnt = 8;	
	if (lServoPulseAddCnt != 0) {	// 추가 펄스 카운트가 발생하면
		if (abs(lServoPulseAddCnt - lServoPulseAddCntPrev) > SERVO_MAX_PULSE)
		{
			lPulseCnt = abs(lServoPulseAddCntPrev) + SERVO_MAX_PULSE;
		}
		else 
		{
			lPulseCnt = abs(lServoPulseAddCnt);
		}
		PlsRemainIntvl = 0;			
		PlsTrainCnt = 0;
		RgrPulseCnt = lPulseCnt/PULSE_TRAIN_NO;
		RemainPulse = lPulseCnt%PULSE_TRAIN_NO;
		RgrPulseCompenPeriod = 0;
		if (RemainPulse)
		{
			RgrPulseCompenPeriod = PULSE_TRAIN_NO/RemainPulse;
		}
		if (lServoPulseAddCnt > 0)
		{
			fMotorDir = SERVO_FWD;	
		}
		else if (lServoPulseAddCnt < 0)
		{
			fMotorDir = SERVO_BWD;	
		}
		setMotorDir (fMotorDir);
		lServoPulseAddCntPrev = lServoPulseAddCnt;
		lServoPulseAddCnt = 0;		
	}
	
	if ((PlsRemainIntvl == 0) && (RemainPulse > 0))
	{
		SendingPulse = RgrPulseCnt + 1;
		RemainPulse--;
	}
	else
	{
		SendingPulse = RgrPulseCnt;
	}

	PlsRemainIntvl++;
	if (PlsRemainIntvl >= RgrPulseCompenPeriod) 
	{
		PlsRemainIntvl = 0;
	}
	
	if (PlsTrainCnt < PULSE_TRAIN_NO)
	{		
		iPulseOut = Pulse_Out(SendingPulse);
		
		lTotalPositionCountOut += fMotorDir*iPulseOut;
		lPulseCnt -= iPulseOut ;
		PlsTrainCnt++;
	}
//	else //연속운전
//	{
//		lServoPulseAddCnt = lServoPulseAddCntPrev;
//	}
}

#define LOAD_SENSOR_NUM	4
#define POS_SENSOR_NUM	6

BYTE bSensorBoardStatus;
//WORD wLoadSensorData[LOAD_SENSOR_NUM];
//WORD wPosSensorData[POS_SENSOR_NUM];
//UINT wPosSensorData[POS_SENSOR_NUM];
BYTE bSensorCmd;




/*
void PosSensorProc()
{
	static int  out[6];
	static int  prev[6];
	int iTemp,iTemp2;
	int tIndex;
	int iMidGap;
	int i;
	
	if (fPosSensorReceived == SET) {
		fPosSensorReceived = RESET;

		tIndex = bPosSensorChannelIndex ;
		if (((bRcvData[0] ^ bRcvData[2]) == 0xFF) && ((bRcvData[1] ^ bRcvData[3]) == 0xFF)) {
			iTemp = bRcvData[0]*0x100+bRcvData[1];
			iTemp = iTemp>>2;
			
			PosSensorDataORG[tIndex] = iTemp;
			iMidGap = 8192 - posSensorOrgOffset[tIndex];
			if (iMidGap> 0) {
				iTemp += ( (iTemp-iMidGap) <= 0) ? (16384-iMidGap) : 0-iMidGap;
			}
			else {
				iTemp += ( (iTemp-iMidGap) >= 16384) ? (0-iMidGap-16384) : 0-iMidGap;
			}	
			

			if (FlFilterInit)
			{
				out[tIndex] = iTemp*4096;
				bPosSensorData[tIndex]= prev[tIndex] = iTemp;
			}
			else if (abs(iTemp-prev[tIndex]) < POS_DEV_LIMIT )
			{
				LPF (&out[tIndex], iTemp, &prev[tIndex], -600);
				bPosSensorData[tIndex] = out[tIndex] /4096 ;      //  4096 = 2^LPF_SHIFT
			}
			else
			{
				bPosSensorData[tIndex]= prev[tIndex];
			}

		}	
		

		if (tIndex == 0)
		{
			PosSensorData[0] = bPosSensorData[0];
		}
		else if (tIndex == 1)
		{
			
			if (bPosSensorData[1] <= SensorCalDataR_LP[0][0] )	{
				PosSensorData[1] = SensorCalDataR_LP[1][0];
			}
			else if (bPosSensorData[1] >= SensorCalDataR_LP[0][CAL_DATA_NO-1] )  {
				PosSensorData[1] = SensorCalDataR_LP[1][CAL_DATA_NO-1];
			}
			else {				
				for ( i = 1; i<CAL_DATA_NO; i++)
				{
					if ((bPosSensorData[1] < SensorCalDataR_LP[0][i] ) && (bPosSensorData[1] >= SensorCalDataR_LP[0][i-1] ))
					{
						iTemp = SensorCalDataR_LP[1][i-1] + (int)((float)((float)(bPosSensorData[1]-SensorCalDataR_LP[0][i-1])
						        *(float)(SensorCalDataR_LP[1][i]-SensorCalDataR_LP[1][i-1])
						        /(float)(SensorCalDataR_LP[0][i]-SensorCalDataR_LP[0][i-1])));
						PosSensorData[1] = iTemp;
					}
				}
			}
			PosSensorData[1] += SensorCalFineOffset[0] ;	
		}
		else if (tIndex == 2)
		{
			if (bPosSensorData[2] <= SensorCalDataR_LE[0][0] )	{
				PosSensorData[2] = SensorCalDataR_LE[1][0];
			}
			else if (bPosSensorData[2] >= SensorCalDataR_LE[0][CAL_DATA_NO-1] )  {
				PosSensorData[2] = SensorCalDataR_LE[1][CAL_DATA_NO-1];
			}
			else {				
				for ( i = 1; i<CAL_DATA_NO; i++)
				{
					if ((bPosSensorData[2] < SensorCalDataR_LE[0][i] ) && (bPosSensorData[2] >= SensorCalDataR_LE[0][i-1] ))
					{
						iTemp = SensorCalDataR_LE[1][i-1] + (int)((float)((float)(bPosSensorData[2]-SensorCalDataR_LE[0][i-1])
						        *(float)(SensorCalDataR_LE[1][i]-SensorCalDataR_LE[1][i-1])
						        /(float)(SensorCalDataR_LE[0][i]-SensorCalDataR_LE[0][i-1])));
						PosSensorData[2] = iTemp;
					}
				}
			}
			PosSensorData[2] += SensorCalFineOffset[1] ;	
			
		}		
		else if (tIndex == 3)
		{
			PosSensorData[3] = bPosSensorData[3];
		}
		else if (tIndex == 4)
		{
			if (bPosSensorData[4] <= SensorCalDataL_LP[0][0] )	{
				PosSensorData[4] = SensorCalDataL_LP[1][0];
			}
			else if (bPosSensorData[4] >= SensorCalDataL_LP[0][CAL_DATA_NO-1] )  {
				PosSensorData[4] = SensorCalDataL_LP[1][CAL_DATA_NO-1];
			}
			else {
				for ( i = 1; i<CAL_DATA_NO; i++)
				{
					if ((bPosSensorData[4] < SensorCalDataL_LP[0][i] ) && (bPosSensorData[4] >= SensorCalDataL_LP[0][i-1] ))
					{
						iTemp = SensorCalDataL_LP[1][i-1] + (int)((float)((float)(bPosSensorData[4]-SensorCalDataL_LP[0][i-1])
						        *(float)(SensorCalDataL_LP[1][i]-SensorCalDataL_LP[1][i-1])
						        /(float)(SensorCalDataL_LP[0][i]-SensorCalDataL_LP[0][i-1])));
						PosSensorData[4] = iTemp;
					}
				}
			}	
			PosSensorData[4] += SensorCalFineOffset[2] ;	
			
		}
		else if (tIndex == 5)
		{
			if (bPosSensorData[5] <= SensorCalDataL_LE[0][0] )	{
				PosSensorData[5] = SensorCalDataL_LE[1][0];
			}
			else if (bPosSensorData[5] >= SensorCalDataL_LE[0][CAL_DATA_NO-1] )  {
				PosSensorData[5] = SensorCalDataL_LE[1][CAL_DATA_NO-1];
			}
			else {
				for ( i = 1; i<CAL_DATA_NO; i++)
				{
					if ((bPosSensorData[5] < SensorCalDataL_LE[0][i] ) && (bPosSensorData[5] >= SensorCalDataL_LE[0][i-1] ))
					{
						iTemp = SensorCalDataL_LE[1][i-1] + (int)((float)((float)(bPosSensorData[5]-SensorCalDataL_LE[0][i-1])
						        *(float)(SensorCalDataL_LE[1][i]-SensorCalDataL_LE[1][i-1])
						        /(float)(SensorCalDataL_LE[0][i]-SensorCalDataL_LE[0][i-1])));
						PosSensorData[5] = iTemp;
					}
				}
			}
			PosSensorData[5] += SensorCalFineOffset[3] ;	
			
		}

//		PosSensorData[tIndex] = bPosSensorData[tIndex];
//		PosSensorEnable(tIndex);
	}
}
*/

#define ESTOP_FILTER_NUM 	5

void SystemIOProcess(void)
{
	static int iEstopPortStateCnt;
	
//	if (bAlmRstCnt) {
//		ServoSVAlarmRstOn();
//	}
//	else {
//		ServoSVAlarmRstOff();
//	}
//	if (IS_SERVO_ALM) {
//		SysStatus.var.StatusR = SYS_ERROR;
//	}
	
	if (getEstopState()) {
		if (iEstopPortStateCnt < ESTOP_FILTER_NUM){
			iEstopPortStateCnt++;
		}
		else {
			SysStatus.var.Estop = 1;
		}
	}
	else {
		if (iEstopPortStateCnt){
			iEstopPortStateCnt--;
		}
		else {
			SysStatus.var.Estop = 0;
		}
	}
}

void	Uart1_TransmitStart(BYTE *bTxBuff, BYTE bFrameSize)
{
	HAL_UART_IRQHandler(&huart1); 
	HAL_UART_Transmit_DMA(&huart1,bTxBuff,bFrameSize);
}

void	Uart2_TransmitStart(BYTE *bTxBuff, BYTE bFrameSize)
{
	HAL_UART_IRQHandler(&huart2); 
	HAL_UART_Transmit_DMA(&huart2,bTxBuff,bFrameSize);
}

void	Uart3_TransmitStart(BYTE *bTxBuff, BYTE bFrameSize)
{
	HAL_UART_IRQHandler(&huart3); 
	HAL_UART_Transmit_DMA(&huart3,bTxBuff,bFrameSize);
}

/*
void	Uart6_TransmitStart(BYTE *bTxBuff, BYTE bFrameSize)
{
	HAL_UART_IRQHandler(&huart6); 
	HAL_UART_Transmit_DMA(&huart6,bTxBuff,bFrameSize);
}
*/

BOOL IsUart2Received()
{
	BOOL fRet;
	
	if (fUart2Received)
	{
		fUart2Received = 0;
		fRet = SET;
	}
	else
		fRet = 0;
		
	return fRet;
}
BOOL IsUart6Received()
{
	BOOL fRet;
	
	if (fUart6Received)
	{
		fUart6Received = 0;
		fRet = SET;
	}
	else
		fRet = 0;
		
	return fRet;
}

void Isr_Uart1(BYTE tmpChar)
{
	switch (commRcvState) {
		case COMM_IDLING:						// First byte is address
			if (tmpChar == MOD_PROT_STATION_ID) {
				Uart1_rxBuffIndex = 0;					// first message come
				Uart1_rxBuff[Uart1_rxBuffIndex++] = tmpChar;	// 수신 데이터를 버퍼에 저장
				commRcvState = COMM_RECEIVING;
			}
			else
				commRcvState = COMM_IGNORING;   // this is not my address
											// waiting til end-of-frame
			bCntEOF=0;						// 4 character timer start.
			break;

		case COMM_RECEIVING:
			if (Uart1_rxBuffIndex < COMM_BUFF_SIZE) {
				Uart1_rxBuff[Uart1_rxBuffIndex++] = tmpChar;	// 수신 데이터를 버퍼에 저장
				bCntEOF=0;
			}
			else {
				commRcvState = COMM_IGNORING;
			}
			break;

		case COMM_IGNORING:		// Only wait EOF is come and back to the IDLING State
			bCntEOF=0;
			break;

		default:
			break;
	}
}
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
