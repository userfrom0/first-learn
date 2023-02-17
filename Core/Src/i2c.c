/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
#include "i2c.h"

/* USER CODE BEGIN 0 */
#include "LDC1314.h"
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c2;

/* I2C2 init function */
void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C2 clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
  /* USER CODE BEGIN I2C2_MspInit 1 */
	  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;//解决方案
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE END I2C2_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);

  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */


//MCU向LDC写入数据，初始化配置
void I2C_Send(uint8_t M_addr, uint8_t Reg_addr, uint16_t send) //I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_REF_COUNT_CH0, 0x04D6);
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t d_send[2] = {0};			//数据缓冲区
	
	d_send[0] = (send&0xff00)>>8;		//数据只能8位8位发送
	d_send[1] = send&0x00ff;				//

	status = HAL_I2C_Mem_Write(&hi2c2, M_addr, Reg_addr, I2C_MEMADD_SIZE_8BIT, &d_send[0], 2,3000);
	if(status != HAL_OK)
	{
//		/* 反初始化I2C通信总线 */
//		HAL_I2C_DeInit(&hi2c1);
//		
//		/* 重新初始化I2C通信总线*/
//		MX_I2C1_Init();
//		printf("EEPROM I2C通信超时！！！ 重新启动I2C...\n");
		while(1);
	}
}

//MCU从LDC读取测量值
uint16_t I2C_Read(uint8_t M_addr, uint8_t Reg_addr)   ////MCU从LDC读取测量值;I2C_Read(LDC1314_Addr, LDC13xx16xx_CMD_DATA_MSB_CH0);
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t d_read[2] = {0};    //数据缓冲区
	uint16_t read = 0;					//声明16字节的read，存放传感器结果

	status = HAL_I2C_Mem_Read(&hi2c2, M_addr, Reg_addr, I2C_MEMADD_SIZE_8BIT, &d_read[0], 2,3000);////提供设备地址，设备寄存器地址，数据缓存区
	if(status != HAL_OK)
	{
//		/* 反初始化I2C通信总线 */
//		HAL_I2C_DeInit(&hi2c1);
//		
//		/* 重新初始化I2C通信总线*/
//		MX_I2C1_Init();
//		printf("EEPROM I2C通信超时！！！ 重新启动I2C...\n");
		while(1);
	}
	read = (d_read[0]<<8)|d_read[1];   ////data16位，低12位代表计算结果，高4位是错误标志
	return read;
}




/* USER CODE END 1 */
