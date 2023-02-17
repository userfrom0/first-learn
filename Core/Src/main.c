/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LDC1314.h"
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
extern __IO uint32_t ADC_ConvertedValue;//这条指令定义了一个名为 ADC_ConvertedValue 的变量，
																				//其类型为 uint32_t，并且该变量是一个 __IO 类型，
																				//即具有输入输出的特殊属性，用于表示一个可能被外部设备或中断程序修改的变量。
float ADC_Vout;																				
//uint32_t ADC_Value[2] = {0};				
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t data = {0};
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
  MX_I2C2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	while(1)
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_SET);//PC12输出高电平接通开关Q5、Q6;
		
		DEVICE_ID();//读取期器件ID		
		LDC1314_Init();
		
		READ_STATUS();//读取期器件状态寄存器
		
		do
		{
			data = I2C_Read(LDC1314_Addr, LDC13xx16xx_CMD_DATA_MSB_CH0);//CH0通道连续采样
			
			/*-------数据手册table32--------------CH0通道连续采样:MUX_CONFIG+CONFIG.ACTIVE_CHAN（0X1A）------------------------------------*/
			//printf("CH0=%d \r\n ",data);
		
		}while(data > 0XFFF);
		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_SET);//PC11输出高电平，SD令LDC1314处于处于睡眠状态，降低功耗
		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_RESET);//PC12输出低电平断开开关Q5、Q6;
			
		
		
		//等待接收指令后排尿如何实现？？
		/*———————————————————————需要加入串口or蓝牙————————————————————————————————————*/
		/*
		while(1)
		{
			ch=getchar();
			
			switch(ch)
			{
				case '1':
				GPIOE_PIN2_SET;		    //PE2输出高电平接通开关Q2；
				GPIOC_PIN0_RESET;	    //PC0输出低电平接通开关Q1；
			    break;
			}
		}
		*/
		
		
		/*------------------------ADC采集：单通道连续采样+中断服务函数+存储值送入SRAM，Debug读取------------------------------------*/
		
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_SET);//PE2输出高电平接通开关Q2；
			    
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);	    //PC0输出低电平接通开关Q1；
		

		MX_ADC1_Init();
		
		HAL_ADCEx_Calibration_Start(&hadc1); //校准ADC
		
		HAL_ADC_Start_IT(&hadc1);						//启动+中断
		
		
		do
		{
			
			//ADC_ConvertedValue = HAL_ADC_GetValue(&hadc1);//Get ADC regular group conversion result.在中断的回调函数中获取
			
			//HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_Value,2);  //使能采集函数保存到ADC_value数组内
			ADC_Vout =(float) ADC_ConvertedValue*(float)3.3/4096;  //转换为模拟值,Vout = Vcc/2 + K*Iin,K=264mV/A
			
		}while(ADC_Vout < 0XFF);//温度升高电阻增大，电流减小，初始电流值大约为xx
		
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_RESET);//PE2输出低电平断开开关Q2；
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);	//PC0输出高电平断开开关Q1；
		       
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
