#ifndef __UART_H__
#define __UART_H__

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define UART_TASK_RATE  100  // 单位为ms，同样也是电压测量任务运行周期
#define UART_BAUD_RATE  115200
#include "main.h"
//#include "Error.h"
#include "Typedefs.h"

//数据结构体
typedef struct{

uint8 buf[8]; 					// 数据

} __attribute__ ((packed))  Data_Type;
//Uart发送数据帧结构体
typedef struct{
	uint8 headA;
	uint8 headB;
	uint8 len;
	uint8 seq;
	uint8 dev;
	uint8 type;
	Data_Type buf;
	uint8 crcA;
	uint8 crcB;
} __attribute__ ((packed)) Uart_Send_Msg_Type;
/*
typedef enum{
	Discharge_Disenable,
	Discharge_Enable,
	Charge_Disenable,
	Charge_Enable
} __attribute__ ((packed)) Uart_Command_Received_Typedef;*/
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Globle Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Uart_Init();//Uart 模块初始化
void Uart_Msg_Send_Task();
uint8 Uart_Msg_Receive_Task();
void Sort_Data();
void GainInit();
//extern Uart_Command_Received_Typedef Uart_Command_Recevied,Uart_Command_Recevied_Prev;
//extern int8 Discharge_Disenable_Times,Discharge_Enable_Times;
//void presure_gain(u8 senseID,u8 data);
#endif
