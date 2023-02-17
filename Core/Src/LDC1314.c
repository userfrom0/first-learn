#include "LDC1314.h"
#include "i2c.h"
uint16_t DEVICE_ID(void)//��ȡ�豸ID-0X3054
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t read[3] = {0};//���ݻ����� ������һ����Ϊ"read"�����飬����Ԫ�ص�����������"uint8_t"�������СΪ3������ʼ��Ϊ0��
	uint16_t ID = 0;//

	//��LDC1314���豸ID��ȡ�����ݻ�����read[3]�У���read[0]��ʼ��ţ�����8�ֽڴ�ţ�read[0]��Ÿ�8λ��read[1]��ŵ�8λ
	status = HAL_I2C_Mem_Read(&hi2c2, LDC1314_Addr, LDC13xx16xx_CMD_DEVID, I2C_MEMADD_SIZE_8BIT, &read[0], 2,3000);
	
	//���I2C״̬λ
	if(status != HAL_OK)
	{
		/* ����ʼ��I2Cͨ������ */
		HAL_I2C_DeInit(&hi2c2);
		
		/* ���³�ʼ��I2Cͨ������*/
		MX_I2C2_Init();
		//printf("EEPROM I2Cͨ�ų�ʱ������ ��������I2C...\n");
	}
	ID = (read[0]<<8)|read[1];//������������������������������
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
		/* ����ʼ��I2Cͨ������ */
		HAL_I2C_DeInit(&hi2c2);
		
		/* ���³�ʼ��I2Cͨ������*/
		MX_I2C2_Init();
		//printf("EEPROM I2Cͨ�ų�ʱ������ ��������I2C...\n");
	}
	else
	{
		//printf("EEPROM I2Cͨ��OK������ \n");
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
	
	I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_REF_COUNT_CH0, 0x04D6);//����fREF=24MHz,�����RCOUNT0 = 1486.5������ȡ��Ϊ1488
	//ͨ��0�ο�����ת��ʱ��
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_REF_COUNT_CH1, 0x04D6);
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_REF_COUNT_CH2, 0x04D6);
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_REF_COUNT_CH3, 0x04D6);
	
	I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_SETTLE_COUNT_CH0, 0x000A);//�趨��LC��·�����ͨ��0����ǰ���ȶ�ʱ��
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_SETTLE_COUNT_CH1, 0x000A);//Ĭ��ֵ settle timeΪ8us
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_SETTLE_COUNT_CH2, 0x000A);
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_SETTLE_COUNT_CH3, 0x000A);
	
	I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_CLOCK_DIVIDERS_CH0, 0x1002);//��������Ҫ�ģ���������Ƶ�ʲ���Ƶ���ο�Ƶ���趨Ϊʱ��Ƶ�ʵĶ���Ƶ
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_CLOCK_DIVIDERS_CH1, 0x1002);
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_CLOCK_DIVIDERS_CH2, 0x1002);
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_CLOCK_DIVIDERS_CH3, 0x1002);
	
	I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_ERROR_CONFIG, 0x0000);
	
	I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_MUX_CONFIG, 0xC20C);//�����������������������������ģ���������������ȫͨ���Զ�ɨ�裬�����˲������Ƶ���趨��3.3MHZ
	I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_DRIVE_CURRENT_CH0,0x9000); //����ͨ��0�Ĵ�����ʱ�����ȶ���ת��ʱ����������
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_DRIVE_CURRENT_CH1,0x9000);
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_DRIVE_CURRENT_CH2,0x9000);
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_DRIVE_CURRENT_CH3,0x9000);
	
	
	I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_CONFIG,0x1011);	 //�����Զ����У�����Զ�У׼,ʹ��ȫ����
																									//�ڴ�������������У�ѡ���ڲ�ʱ��Դ�������豸,��ʼת��
																									//�˼Ĵ����������д�룬��Ϊ�豸����ʱ����������
	*/
	
/*---------------------------�����Զ�У׼ģʽ���Ի�õ�����Χ----------------------------------------------------------*/
	/*
1). ��Ŀ������Ϊ�봫���������ƻ��������롣
2). ͨ���� CONFIG.SLEEP_MODE_EN ����Ϊ b0��ʹ��������˯��ģʽ��
3). Ϊͨ���������� SETTLECOUNT �� RCOUNT ֵ��
4). ͨ���� RP_OVERRIDE_EN ����Ϊ b0 �������Զ�У׼��
5). ͨ���� CONFIG.SLEEP_MODE_EN ����Ϊ b1 ʹ�豸�˳�˯��ģʽ��
6). �����豸ִ������һ�β�����Ŀ���ȶ����̶������������Χ�ڡ�
7). �� INIT_DRIVEx �ֶΣ�λ 10:6���е���Ӧ DRIVE_CURRENTx �Ĵ�������ַ 0x1e��0x1f��0x20 �� 0x21����ȡͨ����������ֵ���������ֵ��
8). ����������ģʽ�����ڼ䣬�� INIT_DRIVEx λ���б����ֵд�� IDRIVEx λ��λ 15:11����
9). ����������ģʽ�£�RP_OVERRIDE_EN Ӧ����Ϊ b1 ���ڹ̶���������
*/
	I2C_Send(LDC1314_Addr,LDC13xx16xx_CMD_RESET_DEVICE,0x8000);//�����λ
	HAL_Delay(10);
	
	I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_REF_COUNT_CH0, 0x05DC);//����fREF = 20MHz,RCOUNT0 = 1487,����ȡ����1500����0X05DC.
	
	
	I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_SETTLE_COUNT_CH0, 0x000A);//�趨��LC��·�����ͨ��0����ǰ���ȶ�ʱ��
	
	
	I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_CLOCK_DIVIDERS_CH0, 0x1002);//����Ƶ�ʲ���Ƶ���ο�Ƶ���趨Ϊʱ��Ƶ�ʵĶ���Ƶ
	
	
	I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_ERROR_CONFIG, 0x0000);//�����ж�
	
	I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_MUX_CONFIG, 0x020D);//����CH0ͨ���Զ�ɨ�裬�����˲������Ƶ���趨��10MHZ
	
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_DRIVE_CURRENT_CH0,0xF000); //����ͨ��0�Ĵ�����ʱ�����ȶ���ת��ʱ���������������õĵ���λ1355uA����Ӧ��Rp��Χ��0.7k~1.05k
	
	
	
	I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_CONFIG,0x0601);	 //ѡ��ͨ�� = ch 0�������Զ�����У��
															//�������Զ�У׼�����ο��ȵ��ԣ���ȷ��������Χ�ķ�ʽ��
	//�жϣ���												//�ڴ����������ڼ�����ȫ����������ѡ���ⲿʱ��Դ���������ж�ʱ����Ϊ0x1681
															//�����豸�Կ�ʼת�����üĴ���д������������
															//��Ϊ�� LDC ���ڻģʽʱ����������豸���á�	
	
	/*-----------Ȼ��λ��Debug�� IDRIVE0 �ĵ���ֵ��д��IDRIVE0�����¿����������Զ�У׼----------------*/
	//�� INIT_DRIVE0�ֶΣ�λ 10:6���е���Ӧ DRIVE_CURRENT0 �Ĵ�������ַ 0x1e����ȡͨ����������ֵ���������ֵ
	//8). ����������ģʽ�����ڼ䣬�� INIT_DRIVEx λ���б����ֵд�� IDRIVEx λ��λ 15:11����
	
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_DRIVE_CURRENT_CH0,0xF000);//����ֵ�ӼĴ����ж�ȡ����д��
    
	//9). ����������ģʽ�£�RP_OVERRIDE_EN Ӧ����Ϊ b1 ���ڹ̶���������
	//I2C_Send(LDC1314_Addr, LDC13xx16xx_CMD_CONFIG,0x1601);
	
	
	
}
