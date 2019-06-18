//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include <timer.h>
#include <uartdrv.h>
//#include "Temperature.h"
#include "Typedefs.h"
#include "em_gpio.h"
#include "Uart.h"
#include "main.h"
//#include "Error.h"
//#include "hal-config.h"
#include "msg.h"
//#include "Pack_Current.h"
#include "crc.h"
#include <string.h>
//#include "System.h"


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define CRC16_INIT 0x0000
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
Uart_Send_Msg_Type 	Uart_Send_Msg;
struct msg Uart_Receive_Msg;

//Uart模块初始化
void Uart_Init()
{
    memset(&Uart_Send_Msg, 0, sizeof(Uart_Send_Msg_Type));// 将数据存储区初始化为0
    memset(&Uart_Receive_Msg, 0, sizeof(struct msg));// 将数据存储区初始化为0
    Uart_Receive_Msg.headA = 0x55;
    Uart_Receive_Msg.headB = 0xaa;
    Uart_Send_Msg.headA= 0x55;
    Uart_Send_Msg.headB = 0xAA;
    Uart_Send_Msg.len = sizeof(Data_Type);
    Uart_Send_Msg.buf.buf[0] = 0;
    Uart_Send_Msg.buf.buf[1] = 2;
    Uart_Send_Msg.seq = 0;
	Uart_Send_Msg.dev = 1;
	Uart_Send_Msg.type = 0;

	uartSetup();
	//RS422_POWER_ON
	//Uart_Command_Recevied = Discharge_Disenable;
}

//Uart数据发送
void Uart_Msg_Send_Task()
{
	Sort_Data();
//	RS422_POWER_ON

//	uartPutData((uint8 *)&Uart_Receive_Msg, sizeof(Uart_Receive_Msg));
	uartPutData((uint8 *)&Uart_Send_Msg, sizeof(Uart_Send_Msg_Type));
//	Delay_ms(100);
//	RS422_POWER_OFF
//	UART2SendData((int8 *)Test_data, sizeof(Test_data));
}

void Sort_Data()
{
	uint32 crc;
	Uart_Send_Msg.seq += 1;
	crc = CalCRC16((uint8*)&Uart_Send_Msg.len, (uint8*)&Uart_Send_Msg.buf - (uint8*)&Uart_Send_Msg.len + Uart_Send_Msg.len);
	Uart_Send_Msg.crcA = crc & 0xFF;
	Uart_Send_Msg.crcB = (crc >> 8) & 0xFF;

}

void GainInit()
{
	//PC15 PC14 PC13 =000    gain=1  SENSOR ID=1
	GPIO_PinModeSet(gpioPortC,13, gpioModeWiredAndPullUp, 0);		//PC13 LSB
	GPIO_PinModeSet(gpioPortC,14, gpioModeWiredAndPullUp, 0);
	GPIO_PinModeSet(gpioPortC,15, gpioModeWiredAndPullUp, 0);		//PC15 MSB

	GPIO_PinModeSet(gpioPortD ,6, gpioModeWiredAndPullUp, 0);		//PD6	LSB
	GPIO_PinModeSet(gpioPortD ,7, gpioModeWiredAndPullUp, 0);		//PD7	MSB


	//PD4 PB13 PB11 = 111  gain=1  SENSOR ID=2
	GPIO_PinModeSet(gpioPortB,11, gpioModeWiredAndPullUp, 1);		//PB11 LSB
	GPIO_PinModeSet(gpioPortB,13, gpioModeWiredAndPullUp, 1);
	GPIO_PinModeSet(gpioPortD,4, gpioModeWiredAndPullUp,  1);		//PD4  MSB

	GPIO_PinModeSet(gpioPortB ,7 , gpioModeWiredAndPullUp, 0);
	GPIO_PinModeSet(gpioPortB ,8 , gpioModeWiredAndPullUp, 0);

	//PC1 PC0 PA2=000 gain=1   SENSOR ID=4
	GPIO_PinModeSet(gpioPortA,2, gpioModeWiredAndPullUp, 0);
	GPIO_PinModeSet(gpioPortC,0, gpioModeWiredAndPullUp, 0);
	GPIO_PinModeSet(gpioPortC,1, gpioModeWiredAndPullUp, 0);

	GPIO_PinModeSet(gpioPortA ,0 , gpioModeWiredAndPullUp, 0);
	GPIO_PinModeSet(gpioPortA ,1 , gpioModeWiredAndPullUp, 0);
}

void OneAxis(struct msg *pmsg)
{
	uint8 temp1 =pmsg->buf[1]-1;
	uint8 temp2 =pmsg->buf[2]-1;

	switch(pmsg->buf[0])//sensor ID
	{
	case 1:
		GPIO_PinModeSet(gpioPortC,13, gpioModeWiredAndPullUp, temp1 & 0x01);		//PC13 LSB
		GPIO_PinModeSet(gpioPortC,14, gpioModeWiredAndPullUp, (temp1 & 0x02)>>1);
		GPIO_PinModeSet(gpioPortC,15, gpioModeWiredAndPullUp, (temp1 & 0x04)>>2);	//PC15 MSB

		GPIO_PinModeSet(gpioPortD ,6, gpioModeWiredAndPullUp, temp2 & 0x01);		//PD6	LSB
		GPIO_PinModeSet(gpioPortD ,7, gpioModeWiredAndPullUp, (temp2 & 0x02)>>1);	//PD7	MSB
		break;
	case 2:
		GPIO_PinModeSet(gpioPortB,11, gpioModeWiredAndPullUp, temp1 & 0x01);		//PB11 LSB
		GPIO_PinModeSet(gpioPortB,13, gpioModeWiredAndPullUp, (temp1 & 0x02)>>1);
		GPIO_PinModeSet(gpioPortD,4, gpioModeWiredAndPullUp, (temp1 & 0x04)>>2);	//PD4  MSB

		GPIO_PinModeSet(gpioPortB ,7 , gpioModeWiredAndPullUp, temp2 & 0x01);
		GPIO_PinModeSet(gpioPortB ,8 , gpioModeWiredAndPullUp, (temp2 & 0x02)>>1);
		break;
	case 4:
		GPIO_PinModeSet(gpioPortA,2, gpioModeWiredAndPullUp, temp1 & 0x01);
		GPIO_PinModeSet(gpioPortC,0, gpioModeWiredAndPullUp, (temp1 & 0x02)>>1);
		GPIO_PinModeSet(gpioPortC,1, gpioModeWiredAndPullUp, (temp1 & 0x04)>>2);

		GPIO_PinModeSet(gpioPortA ,0 , gpioModeWiredAndPullUp, temp2 & 0x01);
		GPIO_PinModeSet(gpioPortA ,1 , gpioModeWiredAndPullUp, (temp2 & 0x02)>>1);
		break;
	default:
		break;
	}

}

void ThreeAxis(struct msg *pmsg)
{
	uint8 temp =pmsg->buf[1]-1;
	GPIO_PinModeSet(gpioPortA,0, gpioModeWiredAndPullUp, temp & 0x01);
	GPIO_PinModeSet(gpioPortA,1, gpioModeWiredAndPullUp, (temp & 0x02)>>1);
	GPIO_PinModeSet(gpioPortA,2, gpioModeWiredAndPullUp, (temp & 0x04)>>2);

	temp =pmsg->buf[2]-1;
	GPIO_PinModeSet(gpioPortB,7, gpioModeWiredAndPullUp, temp & 0x01);
	GPIO_PinModeSet(gpioPortB,8, gpioModeWiredAndPullUp, (temp & 0x02)>>1);
}

//void ThreeAxis2(uint8 gain)
//{
//	uint8 temp =gain-1;
//	GPIO_PinModeSet(gpioPortB,7, gpioModeWiredAndPullUp, temp & 0x01);
//	GPIO_PinModeSet(gpioPortB,8, gpioModeWiredAndPullUp, (temp & 0x02)>>1);
//}

uint8 Uart_Msg_Receive_Task()
{
	//if (get_msg(&Uart_Receive_Msg, uartReadChar))
	if (get_msg(&Uart_Receive_Msg, uartReadChar) == 1)
	{
		//Uart_Msg_Send_Task();
		//Delay_ms(1);
		//GPIO_PinModeSet(gpioPortE,12 ,gpioModeWiredAndPullUp, 1);
		if(Uart_Receive_Msg.type ==0)//立即关闭或者开启
		{
			if(Uart_Receive_Msg.buf[0] == 0 )
			{
				GPIO_PinModeSet(PORT_CapSelect, PIN_Sensor_1AxisAccelerometer_CapSelect1, gpioModeWiredAndPullUp, 0);
			}
			else if(Uart_Receive_Msg.buf[0] == 1 )
			{
				GPIO_PinModeSet(PORT_CapSelect, PIN_Sensor_1AxisAccelerometer_CapSelect1, gpioModeWiredAndPullUp, 1);

			}
		}
	}
	return 0;
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//整理Uart数据帧
/*void Battery_Data_Coll(void)
{
	uint32 crc;
	Uart_Send_Msg.seq += 1;
	memcpy( (uint8 *)Uart_Send_Msg.Battery_Data.voltage_cell , (uint8 *)voltage_cell, sizeof(voltage_cell));
	memcpy( (uint8 *)Uart_Send_Msg.Battery_Data.voltage_sum , (uint8 *)voltage_sum, sizeof(voltage_sum));
	memcpy( (uint8 *)&Uart_Send_Msg.Battery_Data.voltage_timestamp , (uint8 *)&voltage_timestamp, sizeof(int32));
	memcpy( (uint8 *)Uart_Send_Msg.Battery_Data.temperatures , (uint8 *)temperatures, sizeof(temperatures));
	memcpy( (uint8 *)Uart_Send_Msg.Battery_Data.temperature_adc_values , (uint8 *)temperature_adc_values, sizeof(temperature_adc_values));
	memcpy( (uint8 *)&Uart_Send_Msg.Battery_Data.Charge_Current_ADC_Value , (uint8 *)&pack_current_adc_value.charge_current_adc_value, sizeof(int16));
	memcpy( (uint8 *)&Uart_Send_Msg.Battery_Data.Discharge_Current_ADC_Value , (uint8 *)&pack_current_adc_value.discharge_current_adc_value, sizeof(int16));
	memcpy( (uint8 *)&Uart_Send_Msg.Battery_Data.Discharge_Current , (uint8 *)&pack_current.discharge_current, sizeof(int16));
	memcpy( (uint8 *)&Uart_Send_Msg.Battery_Data.Charge_Current , (uint8 *)&pack_current.charge_current, sizeof(int16));
	memcpy( (uint8 *)&Uart_Send_Msg.Battery_Data.pack_current_timestamp , (uint8 *)&pack_current_timestamp, sizeof(int32));
	memcpy( (uint8 *)&Uart_Send_Msg.Battery_Data.error_code_current , (uint8 *)&error_code_current, sizeof(int8));
	memcpy( (uint8 *)Uart_Send_Msg.Battery_Data.error_data , (uint8 *)error_data, sizeof(error_data));
	memcpy( (uint8 *)Uart_Send_Msg.Battery_Data.error_count , (uint8 *)error_count, sizeof(error_count));

	crc = CalCRC16((uint8*)&Uart_Send_Msg.len, (uint8*)&Uart_Send_Msg.Battery_Data - (uint8*)&Uart_Send_Msg.len + Uart_Send_Msg.len);
	Uart_Send_Msg.crcA = crc & 0xFF;
	Uart_Send_Msg.crcB = (crc >> 8) & 0xFF;
}*/
/*
//计算CRC
uint16 CalCRC16(void *pData, uint32 dwNumOfBytes)
{
  uint16 wCRC = CRC16_INIT;  // CRC校验码
  uint8 *pbDataBuf = (uint8*)pData;
  while ( 0 != dwNumOfBytes--)  wCRC = ((wCRC << 8) ^ (crc16_table[((wCRC >> 8) ^ (*pbDataBuf++)) & 0xff]));
  return  wCRC;
}
*/
/*if((Uart_Receive_Msg.buf[0] == 1 )&&(Uart_Receive_Msg.buf[1] == 0 ))

					{
						GPIO_PinModeSet(PORT_GainSelect,PIN_Pressure_GainSelect0, gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_GainSelect,PIN_Pressure_GainSelect1, gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_GainSelect,PIN_Pressure_GainSelect2, gpioModeWiredAndPullUp, 0);
					}
					else if((Uart_Receive_Msg.buf[0] == 1 )&&(Uart_Receive_Msg.buf[1] == 1 ))
					{
						GPIO_PinModeSet(PORT_GainSelect,PIN_Pressure_GainSelect0, gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_GainSelect,PIN_Pressure_GainSelect1, gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_GainSelect,PIN_Pressure_GainSelect2, gpioModeWiredAndPullUp, 0);
					}
					else if((Uart_Receive_Msg.buf[0] == 1 )&&(Uart_Receive_Msg.buf[1] == 2 ))
					{
						GPIO_PinModeSet(PORT_GainSelect,PIN_Pressure_GainSelect0, gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_GainSelect,PIN_Pressure_GainSelect1, gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_GainSelect,PIN_Pressure_GainSelect2, gpioModeWiredAndPullUp, 0);
					}
					else if((Uart_Receive_Msg.buf[0] == 1 )&&(Uart_Receive_Msg.buf[1] == 3 ))
					{
						GPIO_PinModeSet(PORT_GainSelect,PIN_Pressure_GainSelect0, gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_GainSelect,PIN_Pressure_GainSelect1, gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_GainSelect,PIN_Pressure_GainSelect2, gpioModeWiredAndPullUp, 0);
					}
					else if((Uart_Receive_Msg.buf[0] == 1 )&&(Uart_Receive_Msg.buf[1] == 4 ))
					{
						GPIO_PinModeSet(PORT_GainSelect,PIN_Pressure_GainSelect0, gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_GainSelect,PIN_Pressure_GainSelect1, gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_GainSelect,PIN_Pressure_GainSelect2, gpioModeWiredAndPullUp, 1);
					}
					else if((Uart_Receive_Msg.buf[0] == 1 )&&(Uart_Receive_Msg.buf[1] == 5 ))
					{
						GPIO_PinModeSet(PORT_GainSelect,PIN_Pressure_GainSelect0, gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_GainSelect,PIN_Pressure_GainSelect1, gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_GainSelect,PIN_Pressure_GainSelect2, gpioModeWiredAndPullUp, 1);
					}
					else if((Uart_Receive_Msg.buf[0] == 1 )&&(Uart_Receive_Msg.buf[1] == 6 ))
					{
						GPIO_PinModeSet(PORT_GainSelect,PIN_Pressure_GainSelect0, gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_GainSelect,PIN_Pressure_GainSelect1, gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_GainSelect,PIN_Pressure_GainSelect2, gpioModeWiredAndPullUp, 1);
					}
					else if((Uart_Receive_Msg.buf[0] == 1 )&&(Uart_Receive_Msg.buf[1] == 7 ))
					{
						GPIO_PinModeSet(PORT_GainSelect,PIN_Pressure_GainSelect0, gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_GainSelect,PIN_Pressure_GainSelect1, gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_GainSelect,PIN_Pressure_GainSelect2, gpioModeWiredAndPullUp, 1);
					}

					else if((Uart_Receive_Msg.buf[0] == 2 )&&(Uart_Receive_Msg.buf[1] == 0 ))

					{
						GPIO_PinModeSet(PORT_CurrentSelect,		  PIN_Sensor_Temperature_GainSelect0, gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_CurrentSelect,		  PIN_Sensor_Temperature_GainSelect1, gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_Temperature_Pressure,PIN_Sensor_Temperature_GainSelect2, gpioModeWiredAndPullUp, 0);
					}
					else if((Uart_Receive_Msg.buf[0] == 2 )&&(Uart_Receive_Msg.buf[1] == 1 ))
					{
						GPIO_PinModeSet(PORT_CurrentSelect,		  PIN_Sensor_Temperature_GainSelect0, gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_CurrentSelect,		  PIN_Sensor_Temperature_GainSelect1, gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_Temperature_Pressure,PIN_Sensor_Temperature_GainSelect2, gpioModeWiredAndPullUp, 0);
					}
					else if((Uart_Receive_Msg.buf[0] == 2 )&&(Uart_Receive_Msg.buf[1] == 2 ))
					{
						GPIO_PinModeSet(PORT_CurrentSelect,		  PIN_Sensor_Temperature_GainSelect0, gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_CurrentSelect,		  PIN_Sensor_Temperature_GainSelect1, gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_Temperature_Pressure,PIN_Sensor_Temperature_GainSelect2, gpioModeWiredAndPullUp, 0);
					}
					else if((Uart_Receive_Msg.buf[0] == 2 )&&(Uart_Receive_Msg.buf[1] == 3 ))
					{
						GPIO_PinModeSet(PORT_CurrentSelect,		  PIN_Sensor_Temperature_GainSelect0, gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_CurrentSelect,		  PIN_Sensor_Temperature_GainSelect1, gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_Temperature_Pressure,PIN_Sensor_Temperature_GainSelect2, gpioModeWiredAndPullUp, 0);
					}
					else if((Uart_Receive_Msg.buf[0] == 2 )&&(Uart_Receive_Msg.buf[1] == 4 ))
					{
						GPIO_PinModeSet(PORT_CurrentSelect,		  PIN_Sensor_Temperature_GainSelect0, gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_CurrentSelect,		  PIN_Sensor_Temperature_GainSelect1, gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_Temperature_Pressure,PIN_Sensor_Temperature_GainSelect2, gpioModeWiredAndPullUp, 1);
					}
					else if((Uart_Receive_Msg.buf[0] == 2 )&&(Uart_Receive_Msg.buf[1] == 5 ))
					{
						GPIO_PinModeSet(PORT_CurrentSelect,		  PIN_Sensor_Temperature_GainSelect0, gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_CurrentSelect,		  PIN_Sensor_Temperature_GainSelect1, gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_Temperature_Pressure,PIN_Sensor_Temperature_GainSelect2, gpioModeWiredAndPullUp, 1);
					}
					else if((Uart_Receive_Msg.buf[0] == 2 )&&(Uart_Receive_Msg.buf[1] == 6 ))
					{
						GPIO_PinModeSet(PORT_CurrentSelect,		  PIN_Sensor_Temperature_GainSelect0, gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_CurrentSelect,		  PIN_Sensor_Temperature_GainSelect1, gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_Temperature_Pressure,PIN_Sensor_Temperature_GainSelect2, gpioModeWiredAndPullUp, 1);
					}
					else if((Uart_Receive_Msg.buf[0] == 2 )&&(Uart_Receive_Msg.buf[1] == 7 ))
					{
						GPIO_PinModeSet(PORT_CurrentSelect,		  PIN_Sensor_Temperature_GainSelect0, gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_CurrentSelect,		  PIN_Sensor_Temperature_GainSelect1, gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_Temperature_Pressure,PIN_Sensor_Temperature_GainSelect2, gpioModeWiredAndPullUp, 1);
					}
					else if((Uart_Receive_Msg.buf[0] == 4 )&&(Uart_Receive_Msg.buf[1] == 0 ))

					{
						GPIO_PinModeSet(PORT_CapSelect,PIN_Sensor_1AxisAccelerometer_CapSelect1, gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_CapSelect,PIN_Sensor_1AxisAccelerometer_CapSelect2, gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_CapSelect,PIN_Sensor_1AxisAccelerometer_GainSelect0, gpioModeWiredAndPullUp, 0);
					}
					else if((Uart_Receive_Msg.buf[0] == 4 )&&(Uart_Receive_Msg.buf[1] == 1 ))
					{
						GPIO_PinModeSet(PORT_CapSelect,PIN_Sensor_1AxisAccelerometer_CapSelect1, gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_CapSelect,PIN_Sensor_1AxisAccelerometer_CapSelect2, gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_CapSelect,PIN_Sensor_1AxisAccelerometer_GainSelect0, gpioModeWiredAndPullUp, 0);
					}
					else if((Uart_Receive_Msg.buf[0] == 4 )&&(Uart_Receive_Msg.buf[1] == 2 ))
					{
						GPIO_PinModeSet(PORT_CapSelect,PIN_Sensor_1AxisAccelerometer_CapSelect1, gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_CapSelect,PIN_Sensor_1AxisAccelerometer_CapSelect2, gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_CapSelect,PIN_Sensor_1AxisAccelerometer_GainSelect0, gpioModeWiredAndPullUp, 0);
					}
					else if((Uart_Receive_Msg.buf[0] == 4 )&&(Uart_Receive_Msg.buf[1] == 3 ))
					{
						GPIO_PinModeSet(PORT_CapSelect,PIN_Sensor_1AxisAccelerometer_CapSelect1, gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_CapSelect,PIN_Sensor_1AxisAccelerometer_CapSelect2, gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_CapSelect,PIN_Sensor_1AxisAccelerometer_GainSelect0, gpioModeWiredAndPullUp, 0);
					}
					else if((Uart_Receive_Msg.buf[0] == 4 )&&(Uart_Receive_Msg.buf[1] == 4 ))
					{
						GPIO_PinModeSet(PORT_CapSelect,PIN_Sensor_1AxisAccelerometer_CapSelect1, gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_CapSelect,PIN_Sensor_1AxisAccelerometer_CapSelect2, gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_CapSelect,PIN_Sensor_1AxisAccelerometer_GainSelect0, gpioModeWiredAndPullUp, 1);
					}
					else if((Uart_Receive_Msg.buf[0] == 4 )&&(Uart_Receive_Msg.buf[1] == 5 ))
					{
						GPIO_PinModeSet(PORT_CapSelect,PIN_Sensor_1AxisAccelerometer_CapSelect1, gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_CapSelect,PIN_Sensor_1AxisAccelerometer_CapSelect2, gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_CapSelect,PIN_Sensor_1AxisAccelerometer_GainSelect0, gpioModeWiredAndPullUp, 1);
					}
					else if((Uart_Receive_Msg.buf[0] == 4 )&&(Uart_Receive_Msg.buf[1] == 6 ))
					{
						GPIO_PinModeSet(PORT_CapSelect,PIN_Sensor_1AxisAccelerometer_CapSelect1, gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_CapSelect,PIN_Sensor_1AxisAccelerometer_CapSelect2, gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_CapSelect,PIN_Sensor_1AxisAccelerometer_GainSelect0, gpioModeWiredAndPullUp, 1);
					}
					else if((Uart_Receive_Msg.buf[0] == 4 )&&(Uart_Receive_Msg.buf[1] == 7 ))
					{
						GPIO_PinModeSet(PORT_CapSelect,PIN_Sensor_1AxisAccelerometer_CapSelect1, gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_CapSelect,PIN_Sensor_1AxisAccelerometer_CapSelect2, gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_CapSelect,PIN_Sensor_1AxisAccelerometer_GainSelect0, gpioModeWiredAndPullUp, 1);
					}

					else if((Uart_Receive_Msg.buf[0] == 1 )&&(Uart_Receive_Msg.buf[2] == 0 ))
					{
						GPIO_PinModeSet(PORT_Temperature_Pressure ,PIN_Pressure_CapSelect1, gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_Temperature_Pressure ,PIN_Pressure_CapSelect2, gpioModeWiredAndPullUp, 0);
					}
					else if((Uart_Receive_Msg.buf[0] == 1 )&&(Uart_Receive_Msg.buf[2] == 1 ))
					{
						GPIO_PinModeSet(PORT_Temperature_Pressure ,PIN_Pressure_CapSelect1, gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_Temperature_Pressure ,PIN_Pressure_CapSelect2, gpioModeWiredAndPullUp, 0);
					}
					else if((Uart_Receive_Msg.buf[0] == 1 )&&(Uart_Receive_Msg.buf[2] == 2 ))
					{
						GPIO_PinModeSet(PORT_Temperature_Pressure ,PIN_Pressure_CapSelect1, gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_Temperature_Pressure ,PIN_Pressure_CapSelect2, gpioModeWiredAndPullUp, 1);
					}
					else if((Uart_Receive_Msg.buf[0] == 1 )&&(Uart_Receive_Msg.buf[2] == 3 ))
					{
						GPIO_PinModeSet(PORT_Temperature_Pressure ,PIN_Pressure_CapSelect1, gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_Temperature_Pressure ,PIN_Pressure_CapSelect2, gpioModeWiredAndPullUp, 1);
					}
					else if((Uart_Receive_Msg.buf[0] == 2 )&&(Uart_Receive_Msg.buf[2] == 0 ))
					{
						GPIO_PinModeSet(PORT_CurrentSelect ,PIN_Sensor_Temperature_GainSelect0 , gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_CurrentSelect ,PIN_Sensor_Temperature_GainSelect1 , gpioModeWiredAndPullUp, 0);
					}
					else if((Uart_Receive_Msg.buf[0] == 2 )&&(Uart_Receive_Msg.buf[2] == 1 ))
					{
						GPIO_PinModeSet(PORT_CurrentSelect ,PIN_Sensor_Temperature_GainSelect0 , gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_CurrentSelect ,PIN_Sensor_Temperature_GainSelect1 , gpioModeWiredAndPullUp, 0);
					}
					else if((Uart_Receive_Msg.buf[0] == 2 )&&(Uart_Receive_Msg.buf[2] == 2 ))
					{
						GPIO_PinModeSet(PORT_CurrentSelect ,PIN_Sensor_Temperature_GainSelect0 , gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_CurrentSelect ,PIN_Sensor_Temperature_GainSelect1 , gpioModeWiredAndPullUp, 1);
					}
					else if((Uart_Receive_Msg.buf[0] == 2 )&&(Uart_Receive_Msg.buf[2] == 3 ))
					{
						GPIO_PinModeSet(PORT_CurrentSelect ,PIN_Sensor_Temperature_GainSelect0 , gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_CurrentSelect ,PIN_Sensor_Temperature_GainSelect1 , gpioModeWiredAndPullUp, 1);
					}

					else if((Uart_Receive_Msg.buf[0] == 4 )&&(Uart_Receive_Msg.buf[2] == 0 ))
					{
						GPIO_PinModeSet(PORT_GainSelect ,PIN_Sensor_1AxisAccelerometer_GainSelect1 , gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_GainSelect ,PIN_Sensor_1AxisAccelerometer_GainSelect1 , gpioModeWiredAndPullUp, 0);
					}
					else if((Uart_Receive_Msg.buf[0] == 4 )&&(Uart_Receive_Msg.buf[2] == 1 ))
					{
						GPIO_PinModeSet(PORT_GainSelect ,PIN_Sensor_1AxisAccelerometer_GainSelect1 , gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_GainSelect ,PIN_Sensor_1AxisAccelerometer_GainSelect1 , gpioModeWiredAndPullUp, 0);
					}
					else if((Uart_Receive_Msg.buf[0] == 4 )&&(Uart_Receive_Msg.buf[2] == 2 ))
					{
						GPIO_PinModeSet(PORT_GainSelect ,PIN_Sensor_1AxisAccelerometer_GainSelect1 , gpioModeWiredAndPullUp, 0);
						GPIO_PinModeSet(PORT_GainSelect ,PIN_Sensor_1AxisAccelerometer_GainSelect1 , gpioModeWiredAndPullUp, 1);
					}
					else if((Uart_Receive_Msg.buf[0] == 4 )&&(Uart_Receive_Msg.buf[2] == 3 ))
					{
						GPIO_PinModeSet(PORT_GainSelect ,PIN_Sensor_1AxisAccelerometer_GainSelect1 , gpioModeWiredAndPullUp, 1);
						GPIO_PinModeSet(PORT_GainSelect ,PIN_Sensor_1AxisAccelerometer_GainSelect1 , gpioModeWiredAndPullUp, 1);
					}*/



/*if((Uart_Receive_Msg.buf[1] == 0 ))
				{
					GPIO_PinModeSet(gpioPortA,0, gpioModeWiredAndPullUp, 0);
					GPIO_PinModeSet(gpioPortA,1, gpioModeWiredAndPullUp, 0);
					GPIO_PinModeSet(gpioPortA,2, gpioModeWiredAndPullUp, 0);
				}
				else if((Uart_Receive_Msg.buf[1] == 1 ))
				{
					GPIO_PinModeSet(gpioPortA,0, gpioModeWiredAndPullUp, 1);
					GPIO_PinModeSet(gpioPortA,1, gpioModeWiredAndPullUp, 0);
					GPIO_PinModeSet(gpioPortA,2, gpioModeWiredAndPullUp, 0);
				}
				else if((Uart_Receive_Msg.buf[1] == 2 ))
				{
					GPIO_PinModeSet(gpioPortA,0, gpioModeWiredAndPullUp, 0);
					GPIO_PinModeSet(gpioPortA,1, gpioModeWiredAndPullUp, 1);
					GPIO_PinModeSet(gpioPortA,2, gpioModeWiredAndPullUp, 0);
				}
				else if((Uart_Receive_Msg.buf[1] == 3 ))
				{
					GPIO_PinModeSet(gpioPortA,0, gpioModeWiredAndPullUp, 1);
					GPIO_PinModeSet(gpioPortA,1, gpioModeWiredAndPullUp, 1);
					GPIO_PinModeSet(gpioPortA,2, gpioModeWiredAndPullUp, 0);
				}
				else if((Uart_Receive_Msg.buf[1] == 4 ))
				{
					GPIO_PinModeSet(gpioPortA,0, gpioModeWiredAndPullUp, 0);
					GPIO_PinModeSet(gpioPortA,1, gpioModeWiredAndPullUp, 0);
					GPIO_PinModeSet(gpioPortA,2, gpioModeWiredAndPullUp, 1);
				}
				else if((Uart_Receive_Msg.buf[1] == 5 ))
				{
					GPIO_PinModeSet(gpioPortA,0, gpioModeWiredAndPullUp, 1);
					GPIO_PinModeSet(gpioPortA,1, gpioModeWiredAndPullUp, 0);
					GPIO_PinModeSet(gpioPortA,2, gpioModeWiredAndPullUp, 1);
				}
				else if((Uart_Receive_Msg.buf[1] == 6 ))
				{
					GPIO_PinModeSet(gpioPortA,0, gpioModeWiredAndPullUp, 0);
					GPIO_PinModeSet(gpioPortA,1, gpioModeWiredAndPullUp, 1);
					GPIO_PinModeSet(gpioPortA,2, gpioModeWiredAndPullUp, 1);
				}
				else if(Uart_Receive_Msg.buf[1] == 7 )
				{
					GPIO_PinModeSet(gpioPortA,0, gpioModeWiredAndPullUp, 1);
					GPIO_PinModeSet(gpioPortA,1, gpioModeWiredAndPullUp, 1);
					GPIO_PinModeSet(gpioPortA,2, gpioModeWiredAndPullUp, 1);
				}
				else if((Uart_Receive_Msg.buf[0] == 3 )&&(Uart_Receive_Msg.buf[2] == 0 ))
				{
					GPIO_PinModeSet(gpioPortB,7, gpioModeWiredAndPullUp, 0);
					GPIO_PinModeSet(gpioPortB,8, gpioModeWiredAndPullUp, 0);
				}
				else if((Uart_Receive_Msg.buf[0] == 3 )&&(Uart_Receive_Msg.buf[2] == 1 ))
				{
					GPIO_PinModeSet(gpioPortB,7, gpioModeWiredAndPullUp, 1);
					GPIO_PinModeSet(gpioPortB,8, gpioModeWiredAndPullUp, 0);
				}
				else if((Uart_Receive_Msg.buf[0] == 3 )&&(Uart_Receive_Msg.buf[2] == 2 ))
				{
					GPIO_PinModeSet(gpioPortB,7, gpioModeWiredAndPullUp, 0);
					GPIO_PinModeSet(gpioPortB,8, gpioModeWiredAndPullUp, 1);
				}
				else if((Uart_Receive_Msg.buf[0] == 3 )&&(Uart_Receive_Msg.buf[2] == 3 ))
				{
					GPIO_PinModeSet(gpioPortB,7, gpioModeWiredAndPullUp, 1);
					GPIO_PinModeSet(gpioPortB,8, gpioModeWiredAndPullUp, 1);
				}*/
