#include "LDC1314.h"
#include "i2c.h"
uint16_t DEVICE_ID(void)//获取设备ID-0X3054
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t read[3] = {0};//数据缓冲区 声明了一个名为"read"的数组，数组元素的数据类型是"uint8_t"，数组大小为3，并初始化为0。
	uint16_t ID = 0;//

	//把LDC1314的设备ID读取到数据缓冲区read[3]中，从read[0]开始存放，按照8字节存放，read[0]存放高8位，read[1]存放低8位
	status = HAL_I2C_Mem_Read(&hi2c2, LDC1314_Addr, LDC13xx16xx_CMD_DEVID, I2C_MEMADD_SIZE_8BIT, &read[0], 2,3000);
	
	//检查I2C状态位
	if(status != HAL_OK)
	{
		/* 反初始化I2C通信总线 */
		HAL_I2C_DeInit(&hi2c2);
		
		/* 重新初始化I2C通信总线*/
		MX_I2C2_Init();
		//printf("EEPROM I2C通信超时！！！ 重新启动I2C...\n");
	}
	ID = (read[0]<<8)|read[1];//？？？？？？？？？？？？？？？
	return ID;
}

uint16_t READ_STATUS(void)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t read[3] = {0};
	uint16_t Reg_STATUS = 0;

	status = HAL_I2C_Mem_Read(&hi2c2, LDC1314_Addr, LDC13xx16xx_CMD_STATUS, I2C_MEMADD_SIZE_8BIT, &read[0], 2,3000);
	if(status != HAL_OK)
	{
		/* 反初始化I2C通信总线 */
		HAL_I2C_DeInit(&hi2c2);
		
		/* 重新初始化I2C通信总线*/
		MX_I2C2_Init();
		//printf("EEPROM I2C通信超时！！！ 重新启动I2C...\n");
	}
	else
	{
		//printf("EEPROM I2C通信OK！！！ \n");
	}
	Reg_STATUS = (read[0]<<8)|read[1];
	
	return Reg_STATUS;
}

void LDC1314_Init(void)
{
	/*
	// software reset
	I2C_Send(LDC1314_Addr,LDC13xx16xx_CMD_RESET_DEVICE,0x8000);
	HAL_Delay(10);
	
	I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_REF_COUNT_CH0, 0x04D6);//根据fREF=24MHz,计算出RCOUNT0 = 1486.5，向上取整为1488
	//通道0参考计数转换时间
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_REF_COUNT_CH1, 0x04D6);
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_REF_COUNT_CH2, 0x04D6);
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_REF_COUNT_CH3, 0x04D6);
	
	I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_SETTLE_COUNT_CH0, 0x000A);//设定给LC回路振幅在通道0开启前的稳定时间
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_SETTLE_COUNT_CH1, 0x000A);//默认值 settle time为8us
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_SETTLE_COUNT_CH2, 0x000A);
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_SETTLE_COUNT_CH3, 0x000A);
	
	I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_CLOCK_DIVIDERS_CH0, 0x1002);//！！！需要改！！！输入频率不分频，参考频率设定为时钟频率的二分频
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_CLOCK_DIVIDERS_CH1, 0x1002);
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_CLOCK_DIVIDERS_CH2, 0x1002);
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_CLOCK_DIVIDERS_CH3, 0x1002);
	
	I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_ERROR_CONFIG, 0x0000);
	
	I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_MUX_CONFIG, 0xC20C);//——————————————改！！！！！！开启全通道自动扫描，输入滤波器最低频率设定在3.3MHZ
	I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_DRIVE_CURRENT_CH0,0x9000); //定义通道0的传感器时钟在稳定和转换时的驱动电流
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_DRIVE_CURRENT_CH1,0x9000);
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_DRIVE_CURRENT_CH2,0x9000);
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_DRIVE_CURRENT_CH3,0x9000);
	
	
	I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_CONFIG,0x1011);	 //开启自动振幅校正和自动校准,使能全电流
																									//在传感器激活过程中，选择内部时钟源，唤醒设备,开始转换
																									//此寄存器必须最后写入，因为设备激活时不允许配置
	*/
	
/*---------------------------先用自动校准模式调试获得电流范围----------------------------------------------------------*/
	/*
1). 将目标设置为与传感器的最大计划工作距离。
2). 通过将 CONFIG.SLEEP_MODE_EN 设置为 b0，使器件进入睡眠模式。
3). 为通道编程所需的 SETTLECOUNT 和 RCOUNT 值。
4). 通过将 RP_OVERRIDE_EN 设置为 b0 来启用自动校准。
5). 通过将 CONFIG.SLEEP_MODE_EN 设置为 b1 使设备退出睡眠模式。
6). 允许设备执行至少一次测量，目标稳定（固定）在最大工作范围内。
7). 从 INIT_DRIVEx 字段（位 10:6）中的相应 DRIVE_CURRENTx 寄存器（地址 0x1e、0x1f、0x20 或 0x21）读取通道电流驱动值。保存这个值。
8). 在正常工作模式启动期间，将 INIT_DRIVEx 位域中保存的值写入 IDRIVEx 位域（位 15:11）。
9). 在正常工作模式下，RP_OVERRIDE_EN 应设置为 b1 用于固定电流驱动
*/
	I2C_Send(LDC1314_Addr,LDC13xx16xx_CMD_RESET_DEVICE,0x8000);//软件复位
	HAL_Delay(10);
	
	I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_REF_COUNT_CH0, 0x05DC);//根据fREF = 20MHz,RCOUNT0 = 1487,向上取整至1500，即0X05DC.
	
	
	I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_SETTLE_COUNT_CH0, 0x000A);//设定给LC回路振幅在通道0开启前的稳定时间
	
	
	I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_CLOCK_DIVIDERS_CH0, 0x1002);//输入频率不分频，参考频率设定为时钟频率的二分频
	
	
	I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_ERROR_CONFIG, 0x0000);//启用中断
	
	I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_MUX_CONFIG, 0x020D);//开启CH0通道自动扫描，输入滤波器最低频率设定在10MHZ
	
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_DRIVE_CURRENT_CH0,0xF000); //定义通道0的传感器时钟在稳定和转换时的驱动电流，设置的电流位1355uA，对应的Rp范围是0.7k~1.05k
	
	
	
	I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_CONFIG,0x0601);	 //选择活动通道 = ch 0，禁用自动幅度校正
															//先启用自动校准，（参考先调试，后确定电流范围的方式）
	//中断？？												//在传感器激活期间启用全电流驱动，选择外部时钟源，不启用中断时设置为0x1681
															//唤醒设备以开始转换。该寄存器写入必须最后发生，
															//因为当 LDC 处于活动模式时不允许进行设备配置。	
	
	/*-----------然后复位，Debug看 IDRIVE0 的电流值，写入IDRIVE0，重新开启，禁用自动校准----------------*/
	//从 INIT_DRIVE0字段（位 10:6）中的相应 DRIVE_CURRENT0 寄存器（地址 0x1e）读取通道电流驱动值。保存这个值
	//8). 在正常工作模式启动期间，将 INIT_DRIVEx 位域中保存的值写入 IDRIVEx 位域（位 15:11）。
	
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_DRIVE_CURRENT_CH0,0xF000);//电流值从寄存器中读取，再写入
    
	//9). 在正常工作模式下，RP_OVERRIDE_EN 应设置为 b1 用于固定电流驱动
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_CONFIG,0x1601);
	
	
	
}
