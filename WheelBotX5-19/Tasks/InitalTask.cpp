#include "DL_H750.h"
#include "app_threadx.h"
#include "InitalTask.h"
#include "ReferDriver.h"
#include "stdarg.h"
#include "tim.h"
#include "ControlTask.h"
uint16_t serve_moto=500;
TX_THREAD LedThread;
TX_THREAD DebugThread;
TX_SEMAPHORE LedSem;

uint8_t LedThreadStack[512]={0};
uint8_t DebugThreadStack[1024]={0};

TX_BYTE_POOL UARTPool;
SRAM_SET_D2 uint8_t UART_PoolBuf[8192]={0}; //D2 串口内存池空间

TX_BYTE_POOL MotorPool;
SRAM_SET_D2 uint8_t Motor_PoolBuf[8192]={0};//D2 Motor RS485 data pool

TX_BYTE_POOL KEFPool;
SRAM_SET_DTCM uint8_t KEF_PoolBuf[4096]={0};//EKF INS Pool

/*
一切需要用DMA搬运的数据，都不应该放在DTCM域中，否则将引发单片机运行异常但编译不报错
解决方式如上，使用定义的"SRAM_SET_D2"
*/


extern void UART1ThreadFun(ULONG initial_input);
extern void UART2ThreadFun(ULONG initial_input);
extern void RemoterThreadFun(ULONG initial_input);
extern void INSThreadFun(ULONG initial_input);
extern void TemThreadFun (ULONG initial_input);
extern void CANThreadFun(ULONG initial_input);
extern void MotorThreadFun(ULONG initial_input);
extern void MotorWThreadFun(ULONG initial_input);
extern void RoboCTRThreadFun(ULONG initial_input);
extern void CloseLoopThreadFun(ULONG initial_input);
extern void UART5ThreadFun(ULONG initial_input);
extern void DrawUIThreadFun(ULONG initial_input);
extern void MessageThreadFun(ULONG initial_input);



extern TX_BYTE_POOL UARTPool;

extern TX_THREAD UART1Thread;
extern TX_SEMAPHORE UART1RXSem;
extern TX_SEMAPHORE UART1TXSem;

extern TX_THREAD UART2Thread;
extern TX_QUEUE  RefQue;
extern TX_SEMAPHORE  RefSem;

extern TX_THREAD RemoterThread;
extern TX_QUEUE  RemoterRXQue;

extern TX_THREAD INSThread;
extern TX_THREAD TemThread;

extern TX_THREAD RoboCTRThread;
extern TX_SEMAPHORE	CANUpSem;

extern TX_THREAD CloseLoopThread;

extern TX_THREAD 	MotorThread;
extern TX_SEMAPHORE MotorLegSem;
extern TX_THREAD	MotorWThread;

extern TX_THREAD UART5Thread;
extern TX_THREAD DrawUIThread;

extern TX_THREAD MessgaeThread;

extern uint8_t UART1ThreadStack[512];

extern uint8_t UART2ThreadStack[256];



extern uint8_t RemoterThreadStack[512];
extern uint8_t RemoterQueueStack[32];

extern uint8_t INSThreadStack[2048];
extern uint8_t TemThreadStack[512];

extern uint8_t MotorThreadStack[2048];
extern uint8_t MotorWThreadStack[2048];

extern uint8_t RoboCTRThreadStack[2048];

extern uint8_t CloseLoopThreadStack[1024];

extern uint8_t UART5ThreadStack[1024];
extern uint8_t DrawUIThreadStack[1024];
extern uint8_t RefQueueStack[32];

extern uint8_t MessageThreadStack[1024];


float BatVal = 0.0f;
extern float L_leg_dot[2];
extern float R_leg_dot[2];
extern float Leg_Ldot[2];
extern float Leg_Rdot[2];
extern float L_T[2];
extern float R_T[2];
extern cINS *INS;
static void LedThreadFun(ULONG initial_input)
{
	ULONG ticker = 0;

	LL_TIM_EnableAllOutputs(TIM8);
	LL_TIM_CC_EnableChannel(TIM8,LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM8,LL_TIM_CHANNEL_CH2);
	LL_TIM_EnableCounter(TIM8);
	LL_TIM_EnableAllOutputs(TIM4);
	LL_TIM_CC_EnableChannel(TIM4,LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_EnableChannel(TIM4,LL_TIM_CHANNEL_CH4);
	LL_TIM_EnableCounter(TIM4);
	
	/*电压检测配置使能*/
	LL_ADC_StartCalibration(ADC1,LL_ADC_CALIB_OFFSET_LINEARITY,LL_ADC_SINGLE_ENDED);
	while(LL_ADC_IsCalibrationOnGoing(ADC1))
	{tx_thread_sleep(100);}
	LL_OPAMP_Enable(OPAMP1);
	tx_thread_sleep(100);
	LL_ADC_Enable(ADC1);
	tx_thread_sleep(100);

	uint16_t RGColor=0;
	int16_t ADC_Val = 0;
	
	for(;;)
	{
		LL_ADC_REG_StartConversion(ADC1);
		
		/*No battery should not scream*/
		if((BatVal<20.0f)&&(BatVal>9.0f))
		{
			LL_TIM_OC_SetCompareCH1(TIM8,499);
			LL_TIM_OC_SetCompareCH2(TIM8,100);
			tx_thread_sleep(50);
			LL_TIM_OC_SetCompareCH1(TIM8,0);
			LL_TIM_OC_SetCompareCH2(TIM8,999);
			tx_thread_sleep(50);
		}
		else
		{
			/*按照红色计算*/
			if(BatVal>25.5f){RGColor=999;}
			else
			{
				RGColor = 400*(BatVal-20.0f);
			}
		
			/*绿色*/
			LL_TIM_OC_SetCompareCH3(TIM4,999-RGColor);
			LL_TIM_OC_SetCompareCH2(TIM8,RGColor);
			tx_thread_sleep(900);
			LL_TIM_OC_SetCompareCH3(TIM4,999);
			LL_TIM_OC_SetCompareCH2(TIM8,999);
			tx_thread_sleep(100);
		}
		
		ADC_Val = LL_ADC_REG_ReadConversionData16(ADC1);
		BatVal = 0.2f*BatVal + 0.0004431152f*(float)LL_ADC_REG_ReadConversionData16(ADC1);
	}
}

uint8_t DebugPrintf(uint8_t *buf, const char *str, ...)
{
		/*计算字符串长度,并将字符串输出到数据区*/
		va_list ap;
		va_start(ap, str);
		uint8_t len = vsnprintf( (char*)buf, 256, str, ap);
		va_end(ap);
		return len;
}


cUART UARTDebug;
extern cRobotControl *RobotControl;
uint8_t *pbufx = 0;
static void DebugThreadFun(ULONG initial_input)
{
	/*对UART初始化*/	
	UARTDebug.UART_Init(UART7,DMA1,LL_DMA_STREAM_3);
	
	/*Transmit buf*/
	
	tx_byte_allocate(&MotorPool, (VOID **)&pbufx, 256, TX_NO_WAIT);
	
	uint8_t Txlen = 0;
	ULONG ticker = 0;
	for(;;)
	{
	   __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, serve_moto);
     __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, serve_moto);
	   __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, serve_moto);
     __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, serve_moto);
		//测试腿误差,左1右0
		//Txlen = DebugPrintf(pbufx,"f=%f,g=%f,V=%f,L=%d,",RobotControl->ChasisControl.Get_GravityForward(),RobotControl->ChasisControl.Get_CheckOG_Value(),RobotControl->ChasisControl.ObserveVal.X[3]*10,RobotControl->ChasisControl.GetLegLenFlag()*10);
		Txlen = DebugPrintf(pbufx,"Llen_dot1=%f,Llen_dot2=%f,",RobotControl->ChasisControl.Lleg_ObserveVal.ChasisFn,RobotControl->ChasisControl.Rleg_ObserveVal.ChasisFn,Leg_Rdot[0]);
//    Txlen = DebugPrintf(pbufx,"Llen_dot0=%f,Llen_dot1=%f,Llen_dot2=%f,",RobotControl->ChasisControl.Rleg_ObserveVal.X[0],RobotControl->ChasisControl.Rleg_ObserveVal.X[1],RobotControl->ChasisControl.Rleg_ObserveVal.ChasisThetaAccel);
		SCB_CleanInvalidateDCache_by_Addr((uint32_t*)pbufx,Txlen);
		UARTDebug.Transmit_DMA(pbufx,Txlen);
		tx_thread_sleep(50);
	}
}


void Task_Init(void)
{
	LL_GPIO_SetOutputPin(EN_5V_GPIO_Port,EN_5V_Pin);
	
/**********内存池***********/
	tx_byte_pool_create(
		&UARTPool,
		(CHAR*)"UART_Pool",
		UART_PoolBuf,
		sizeof(UART_PoolBuf));
	
	tx_byte_pool_create(
		&MotorPool,
		(CHAR*)"Motor_Pool",
		Motor_PoolBuf,
		sizeof(Motor_PoolBuf));

	tx_byte_pool_create(
		&KEFPool,
		(CHAR*)"KEF_Pool",
		KEF_PoolBuf,
		sizeof(KEF_PoolBuf));

/**********信号量***********/	
	tx_semaphore_create(
		&CANUpSem,
		(CHAR*)"CANUpSem",
		0
		);

	tx_semaphore_create(
		&MotorLegSem,
		(CHAR*)"MotorLegSem",
		0
		);
		
	tx_semaphore_create(
		&RefSem,
		(CHAR*)"RefSem",
		0
		);
/**********消息队列***********/
	/*DBUS*/
	tx_queue_create(
		&RemoterRXQue,
		(CHAR*)"REMOTERQUE",
		4, 
		RemoterQueueStack,
		sizeof(RemoterQueueStack));
		
	tx_queue_create(
		&RefQue,
		(CHAR*)"REFQUE",
		sizeof(ref_msg_t), 
		RefQueueStack,
		sizeof(RefQueueStack));
		
/**********进程***********/

		
	tx_thread_create(
		&RemoterThread, 
		(CHAR*)"REMOTER",
		RemoterThreadFun, 
		0x0000,
		RemoterThreadStack,
		sizeof(RemoterThreadStack),
		4,
		4,
		TX_NO_TIME_SLICE,
		TX_AUTO_START);
			
	tx_thread_create(
		&INSThread, 
		(CHAR*)"INS",
		INSThreadFun, 
		0x0000,
		INSThreadStack,
		sizeof(INSThreadStack),
		3,
		3,
		TX_NO_TIME_SLICE,
		TX_AUTO_START);

	tx_thread_create(
		&TemThread, 
		(CHAR*)"INSTEM",
		TemThreadFun, 
		0x0000,
		TemThreadStack,
		sizeof(TemThreadStack),
		10,
		10,
		TX_NO_TIME_SLICE,
		TX_AUTO_START);
	
	tx_thread_create(
		&MotorThread, 
		(CHAR*)"Motor",
		MotorThreadFun, 
		0x0000,
		MotorThreadStack,
		sizeof(MotorThreadStack),
		5,
		5,
		TX_NO_TIME_SLICE,
		TX_AUTO_START);

	tx_thread_create(
		&MotorWThread, 
		(CHAR*)"MotorW",
		MotorWThreadFun, 
		0x0000,
		MotorWThreadStack,
		sizeof(MotorWThreadStack),
		5,
		5,
		TX_NO_TIME_SLICE,
		TX_AUTO_START);
		
	tx_thread_create(
		&RoboCTRThread, 
		(CHAR*)"CONTROL",
		RoboCTRThreadFun, 
		0x0000,
		RoboCTRThreadStack,
		sizeof(RoboCTRThreadStack),
		6,
		6,
		TX_NO_TIME_SLICE,
		TX_AUTO_START);

	tx_thread_create(
		&CloseLoopThread, 
		(CHAR*)"CLOSELOOP",
		CloseLoopThreadFun, 
		0x0000,
		CloseLoopThreadStack,
		sizeof(CloseLoopThreadStack),
		6,
		6,
		TX_NO_TIME_SLICE,
		TX_AUTO_START);
		
	tx_thread_create(
		&UART5Thread, 
		(CHAR*)"UART5",
		UART5ThreadFun, 
		0x0000,
		UART5ThreadStack,
		sizeof(UART5ThreadStack),
		7,
		7,
		TX_NO_TIME_SLICE,
		TX_AUTO_START);
		
	tx_thread_create(
		&DrawUIThread,
		(CHAR*)"DrawUI",
		DrawUIThreadFun,
		0x0000,
		DrawUIThreadStack,
		sizeof(DrawUIThreadStack),
		8,
		8,
		TX_NO_TIME_SLICE,
		TX_AUTO_START);	
				
	tx_thread_create(
		&LedThread, 
		(CHAR*)"Led",
		LedThreadFun, 
		0x0000,
		LedThreadStack,
		sizeof(LedThreadStack),
		15,
		15,
		TX_NO_TIME_SLICE,
		TX_AUTO_START);	
					
	tx_thread_create(
		&DebugThread, 
		(CHAR*)"DEBUG",
		DebugThreadFun, 
		0x0000,
		DebugThreadStack,
		sizeof(DebugThreadStack),
		10,
		10,
		TX_NO_TIME_SLICE,
		TX_AUTO_START);
				
}

void DMA1_Stream2_IRQHandler(void)
{
	
}
void DMA1_Stream3_IRQHandler(void)
{
	UARTDebug.IRQ_Tx();
}
void UART7_IRQHandler(void)
{
	UARTDebug.IRQ_Rx();
}


