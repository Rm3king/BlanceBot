#include "MotorTask.h"
#include "RemoterTask.h"
#include "ControlTask.h"

void CANFilterConfig(void);
extern cRemoter *Msg_Remoter;
extern cRef *Msg_Refer;
extern TX_BYTE_POOL MotorPool;
extern cRobotControl* RobotControl;
cMotorUnit MotorUnitt;
cMotorUnit *MotorUnit = &MotorUnitt;

TX_SEMAPHORE MotorLegSem;
TX_SEMAPHORE CANUpSem;

TX_THREAD MotorThread;
uint8_t MotorThreadStack[2048]={0};

ULONG tim1,tim2;

void MotorThreadFun(ULONG initial_input)
{	
	CANFilterConfig();
	/*Gimbal 6020 correct value*/
	MotorUnit->Y6020.SetEcdOffset(Y6020CORRECT);
	
	/*Send to gimbal*/
	FDCAN_TxHeaderTypeDef TxHeader1 = {0};
	TxHeader1.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader1.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader1.Identifier = 0x111;                                     
	TxHeader1.IdType = FDCAN_STANDARD_ID;
	TxHeader1.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader1.TxFrameType = FDCAN_DATA_FRAME;
	
	FDCAN_TxHeaderTypeDef TxHeader3 = {0};
	TxHeader3.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader3.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader3.Identifier = 0x113;                                     
	TxHeader3.IdType = FDCAN_STANDARD_ID;
	TxHeader3.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader3.TxFrameType = FDCAN_DATA_FRAME;
	
	FDCAN_TxHeaderTypeDef TxHeader4 = {0};
	TxHeader4.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader4.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader4.Identifier = 0x114;                                     
	TxHeader4.IdType = FDCAN_STANDARD_ID;
	TxHeader4.DataLength = FDCAN_DLC_BYTES_7;
	TxHeader4.TxFrameType = FDCAN_DATA_FRAME;
	
	/*Send to SC*/
	FDCAN_TxHeaderTypeDef TxHeader2 = {0};
	TxHeader2.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader2.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader2.Identifier = 0x401;                                     
	TxHeader2.IdType = FDCAN_STANDARD_ID;
	TxHeader2.DataLength = FDCAN_DLC_BYTES_2;
	TxHeader2.TxFrameType = FDCAN_DATA_FRAME;
	
	/*8006*/
	MotorUnit->LEGMotor[0].SetID(&hfdcan2,2);
	MotorUnit->LEGMotor[0].SetMotorMode(1,0);//左前
	MotorUnit->LEGMotor[1].SetID(&hfdcan2,3);
	MotorUnit->LEGMotor[1].SetMotorMode(1,1);//左后
	MotorUnit->LEGMotor[2].SetID(&hfdcan2,4);
	MotorUnit->LEGMotor[2].SetMotorMode(0,0);//右前
	MotorUnit->LEGMotor[3].SetID(&hfdcan2,5);
	MotorUnit->LEGMotor[3].SetMotorMode(0,1);//右后
	
	tx_thread_sleep(150);
	//#define CAL_LEG
	#ifdef CAL_LEG
	/*设置零点ROM*/
	tx_thread_sleep(1000);
	MotorUnit->LEGMotor[0].SetZero();tx_thread_sleep(500);
	MotorUnit->LEGMotor[1].SetZero();tx_thread_sleep(500);
	MotorUnit->LEGMotor[2].SetZero();tx_thread_sleep(500);
	MotorUnit->LEGMotor[3].SetZero();tx_thread_sleep(500);
	#endif

	/*保护性置0*/
	MotorUnit->LEGMotor[0].MITUpdate(0,0,0,0,0);
	MotorUnit->LEGMotor[1].MITUpdate(0,0,0,0,0);
	MotorUnit->LEGMotor[2].MITUpdate(0,0,0,0,0);
	MotorUnit->LEGMotor[3].MITUpdate(0,0,0,0,0);
	
	uint8_t RobotModeLast = RobotControl->CheckRobotMode();
	
	ULONG timer = 0;
	uint8_t psr = 0;
	for(;;)
	{
		timer = tx_time_get();
		tim1 = timer;
		
		/*云台控制发送 250Hz*/
		/*Send CAN message*/	
		if(++psr==4)
		{
			HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&TxHeader1,(uint8_t*)(&RobotControl->rc_data_1));
			psr=0;
		}

		else if(psr==3)
		{
			/*低于40J开始限制电容充电*/
			uint8_t pdata[2];
			uint16_t powerlimit = Msg_Refer->robot_referee_status->ext_game_robot_status.chassis_power_limit-5;
			if(Msg_Refer->robot_referee_status->ext_power_heat_data.chassis_power_buffer<40)
			{powerlimit = powerlimit * Msg_Refer->robot_referee_status->ext_power_heat_data.chassis_power_buffer/40;}
				
			pdata[0] = (powerlimit-5)&0xff;
			pdata[1] = (powerlimit-5)>>8;
			HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&TxHeader2,pdata);
		}
		else if(psr==2)
		{
			HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&TxHeader3,(uint8_t*)(&RobotControl->rc_data_2));	
		}
		else if(psr==1)
		{
			HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&TxHeader4,(uint8_t*)(&RobotControl->rc_data_3));	
		}
		
		/*Motor disable or enable*/
		if(RobotModeLast!=RobotControl->CheckRobotMode())
		{
			if(RobotControl->CheckRobotMode()==ROBOTMODE_NORMAL)
			{
				RobotControl->ChasisControl.MotorUnits->LEGMotor[0].EnableMotor();tx_semaphore_get(&MotorLegSem,2);
				RobotControl->ChasisControl.MotorUnits->LEGMotor[1].EnableMotor();tx_semaphore_get(&MotorLegSem,2);
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&TxHeader1,(uint8_t*)(&RobotControl->ComDown));
				RobotControl->ChasisControl.MotorUnits->LEGMotor[2].EnableMotor();tx_semaphore_get(&MotorLegSem,2);
				RobotControl->ChasisControl.MotorUnits->LEGMotor[3].EnableMotor();tx_semaphore_get(&MotorLegSem,2);
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&TxHeader1,(uint8_t*)(&RobotControl->ComDown));
			}
			else if(RobotModeLast==ROBOTMODE_NORMAL)
			{
				RobotControl->ChasisControl.MotorUnits->LEGMotor[0].DisableMotor();tx_semaphore_get(&MotorLegSem,2);
				RobotControl->ChasisControl.MotorUnits->LEGMotor[1].DisableMotor();tx_semaphore_get(&MotorLegSem,2);
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&TxHeader1,(uint8_t*)(&RobotControl->ComDown));
				RobotControl->ChasisControl.MotorUnits->LEGMotor[2].DisableMotor();tx_semaphore_get(&MotorLegSem,2);
				RobotControl->ChasisControl.MotorUnits->LEGMotor[3].DisableMotor();tx_semaphore_get(&MotorLegSem,2);
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&TxHeader1,(uint8_t*)(&RobotControl->ComDown));
			}
			timer = tx_time_get();		
		}
		RobotModeLast = RobotControl->CheckRobotMode();
		
		//		#warning "DEBUGHERE"	
		MotorUnit->LEGMotor[0].MITTransmit();if(tx_semaphore_get(&MotorLegSem,2)==TX_NO_INSTANCE){MotorUnit->LEGMotor[0].loseCom++;}	
		MotorUnit->LEGMotor[1].MITTransmit();if(tx_semaphore_get(&MotorLegSem,2)==TX_NO_INSTANCE){MotorUnit->LEGMotor[1].loseCom++;}			
		tx_thread_sleep(1);
		MotorUnit->LEGMotor[2].MITTransmit();if(tx_semaphore_get(&MotorLegSem,2)==TX_NO_INSTANCE){MotorUnit->LEGMotor[2].loseCom++;}	
		MotorUnit->LEGMotor[3].MITTransmit();if(tx_semaphore_get(&MotorLegSem,2)==TX_NO_INSTANCE){MotorUnit->LEGMotor[3].loseCom++;}	
		
		
		/*就在这搞, 这路电机寄了全都寄吧*/
		MotorUnit->UpdateLink();
		tx_thread_sleep_until(&timer,2);
		tim2 = tx_time_get() - tim1;
	}
}

TX_THREAD MotorWThread;
uint8_t MotorWThreadStack[2048]={0};
void MotorWThreadFun(ULONG initial_input)
{
	cUART UARTL;
	cUART UARTR;
	/*这是一个工具人*/
	uint8_t *pBuf=0;
	
	/*对UART初始化*/	
	tx_byte_allocate(&MotorPool, (VOID **)&pBuf, 256, TX_NO_WAIT);
	UARTL.UART_BufferConfig(pBuf,256);
	UARTL.UART_Init(USART2,DMA1,LL_DMA_STREAM_1,0);
	
	
	/*对UART初始化*/	
	tx_byte_allocate(&MotorPool, (VOID **)&pBuf, 256, TX_NO_WAIT);
	UARTR.UART_BufferConfig(pBuf,256);
	UARTR.UART_Init(USART3,DMA2,LL_DMA_STREAM_2,0);
		
	/*对RS485初始化*/
	MotorUnit->BUS[0].SetUART(&UARTL,USART2_DE_GPIO_Port,USART2_DE_Pin);
	/*对RS485初始化 */
	MotorUnit->BUS[1].SetUART(&UARTR,USART3_DE_GPIO_Port,USART3_DE_Pin);
	
	/*Super Cap power control pack*/
	FDCAN_TxHeaderTypeDef TxHeader1 = {0};
	TxHeader1.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader1.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader1.Identifier = 0x401;                                     
	TxHeader1.IdType = FDCAN_STANDARD_ID;
	TxHeader1.DataLength = FDCAN_DLC_BYTES_2;
	TxHeader1.TxFrameType = FDCAN_DATA_FRAME;
	
	/*对电机初始化*/
	/*9025L*/
	tx_byte_allocate(&MotorPool, (VOID **)&pBuf, sizeof(Motor_Measurement_t), TX_NO_WAIT);
	MotorUnit->KF9025[0].SetMsr((Motor_Measurement_t*)pBuf);
	tx_byte_allocate(&MotorPool, (VOID **)&pBuf, sizeof(Motor_Measurement_t), TX_NO_WAIT);
	MotorUnit->KF9025[0].SetParams((Motor_Measurement_t*)pBuf);
	MotorUnit->KF9025[0].Params->Motor_id = 0x01;
	MotorUnit->KF9025[0].SetIqConst(TORQUECONSTANT_9025_16T);
		
	/*9025R*/
	tx_byte_allocate(&MotorPool, (VOID **)&pBuf, sizeof(Motor_Measurement_t), TX_NO_WAIT);
	MotorUnit->KF9025[1].SetMsr((Motor_Measurement_t*)pBuf);
	tx_byte_allocate(&MotorPool, (VOID **)&pBuf, sizeof(Motor_Measurement_t), TX_NO_WAIT);
	MotorUnit->KF9025[1].SetParams((Motor_Measurement_t*)pBuf);
	MotorUnit->KF9025[1].Params->Motor_id = 0x01;
	MotorUnit->KF9025[1].SetIqConst(TORQUECONSTANT_9025_16T);
	
	/*电机发送缓冲区*/
	uint8_t pDataTX[32];
	
	/*开启电机接收*/
	MotorUnit->BUS[0].pUART->Recieve_DMA(&MotorUnit->BUS[0].pprecbuf,MOTOR_DATA_LEN);
	MotorUnit->BUS[1].pUART->Recieve_DMA(&MotorUnit->BUS[1].pprecbuf,MOTOR_DATA_LEN);
	
	/*保护性置0*/
	MotorUnit->KF9025[0].Params->iqcontrol = 0;
	MotorUnit->KF9025[1].Params->iqcontrol = 0;
	
	ULONG timer = 0;
	
	for(;;)
	{
		timer = tx_time_get();
		
		/*KF9025 TX*/
		MotorUnit->KF9025[0].SetIqControl(pDataTX);
		MotorUnit->KF9025[0].loseCom++;
		MotorUnit->BUS[0].BusTransmit(pDataTX,9);
		MotorUnit->KF9025[1].SetIqControl(pDataTX);
		MotorUnit->KF9025[1].loseCom++;
		MotorUnit->BUS[1].BusTransmit(pDataTX,9);
		
		tx_thread_sleep_until(&timer,2);
		MotorUnit->UpdateOdomentor();
	}

}


void USART3_IRQHandler(void)
{
	uint8_t ID;
	if(LL_USART_IsActiveFlag_IDLE(USART3))
	{
		MotorUnit->BUS[1].pUART->IRQ_Rx();
		/*清Cache*/
		SCB_CleanInvalidateDCache_by_Addr((uint32_t*)MotorUnit->BUS[1].pprecbuf, MOTOR_DATA_LEN);
		if(MotorUnit->BUS[1].pprecbuf[1]==0xA1)
		{
			if((data_check(MotorUnit->BUS[1].pprecbuf+5,7))==0)
			{
				
				switch(MotorUnit->BUS[1].pprecbuf[2])
				{
					case 0x01:
						MotorUnit->KF9025[1].ReceiveIqMsr(MotorUnit->BUS[1].pprecbuf+5);
						MotorUnit->KF9025[1].UpdateData();
						MotorUnit->KF9025[1].loseCom--;
					break;
				}
			}
		} 

		MotorUnit->BUS[1].pUART->Recieve_DMA(&MotorUnit->BUS[1].pprecbuf,MOTOR_DATA_LEN);
	}
}

void USART2_IRQHandler(void)
{
	uint8_t ID;
	if(LL_USART_IsActiveFlag_IDLE(USART2))
	{
		MotorUnit->BUS[0].pUART->IRQ_Rx();
		/*清Cache*/
		SCB_CleanInvalidateDCache_by_Addr((uint32_t*)MotorUnit->BUS[0].pprecbuf, MOTOR_DATA_LEN);
	
		if(MotorUnit->BUS[0].pprecbuf[1]==0xA1)
		{
			if((data_check(MotorUnit->BUS[0].pprecbuf+5,7))==0)
			{
				
				switch(MotorUnit->BUS[0].pprecbuf[2])
				{
					case 0x01:
						MotorUnit->KF9025[0].ReceiveIqMsr(MotorUnit->BUS[0].pprecbuf+5);
						MotorUnit->KF9025[0].UpdateData();
						MotorUnit->KF9025[0].loseCom--;
					break;
				}
			}
		}
		
		MotorUnit->BUS[0].pUART->Recieve_DMA(&MotorUnit->BUS[0].pprecbuf,MOTOR_DATA_LEN);
	}
}

/*草TMD，老子就写到一块，疯了就疯了*/
void cMotorUnit::UpdateLink(void)
{
//	#warning "DEBUGHERE"//前后
	this->LinkSolver[0].InputLink(this->LEGMotor[0].GetRadian(), this->LEGMotor[1].GetRadian());
	//this->LinkSolver[0].InputLink(0.087f, 3.054f);
	this->LinkSolver[1].InputLink(this->LEGMotor[2].GetRadian(), this->LEGMotor[3].GetRadian());
	this->LinkSolver[0].Resolve();
	this->LinkSolver[1].Resolve();
}

//#warning " reply by jyy"
/*可以 但是要处理好电机重新上电后多圈位置变化*/
/*多了个控制指令控制速度可能达不到 不如直接用变化值算增量吧*/
void cMotorUnit::UpdateOdomentor(void)
{
	float tmp1=0;
	tmp1=0.5f*(this->KF9025[0].GetSpeed()-this->KF9025[1].GetSpeed());
	//tmp1=-this->KF9025[1].GetSpeed();
	this->velocity=this->FilterV.BTW2Cal(tmp1);
	
	float tmp[2]={0};
	tmp[0]=this->KF9025[0].GetRadian()-this->dlast[0];
	tmp[1]=this->KF9025[1].GetRadian()-this->dlast[1];
	
	this->dlast[0] = this->KF9025[0].GetRadian();
	this->dlast[1] = this->KF9025[1].GetRadian();
	
	tmp[0] = (tmp[0]>4) ?  tmp[0]-2*PI : tmp[0] ;
	tmp[0] = (tmp[0]<-4)?  tmp[0]+2*PI : tmp[0] ;
	
	tmp[1] = (tmp[1]>4) ?  tmp[1]-2*PI : tmp[1] ;
	tmp[1] = (tmp[1]<-4)?  tmp[1]+2*PI : tmp[1] ;

	//#warning "DEBUGHERE"
	this->displacement += (WHEELCOEFF*this->FilterD.BTW2Cal(0.5f*(tmp[0]-tmp[1])) );
	//this->displacement += WHEELCOEFF*this->FilterD.BTW2Cal(tmp[0]);
}


/*
	在下面的两个FDCAN接收回调函数里面，选一个处理RM电机，选一个处理云台发送回来的数据
*/


uint8_t RxData1[8];
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	FDCAN_RxHeaderTypeDef RxHeader;
	HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&RxHeader,RxData1);
	/*Data is in RxData1*/
	if(RxHeader.Identifier==0x205)//Y6020
	{
		MotorUnit->Y6020.UpdateMotorRec(RxData1);
		RobotControl->ChasisControl.SetErrWithGim(0.000244140625*MotorUnit->Y6020.GetEcdCorrect()-1.0f);
	}
	else if(RxHeader.Identifier==0x301)//SuperCapacity
	{RobotControl->ChasisControl.SCPowerRemianInput(RxData1);}
	else if(RxHeader.Identifier==0x112)//Gimbal
	{
		memcpy(&(RobotControl->ComUP),RxData1,COMUPSIZE);
		tx_semaphore_put(&CANUpSem);
	}
}

uint8_t RxData2[8];
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	FDCAN_RxHeaderTypeDef RxHeader;
	HAL_FDCAN_GetRxMessage(&hfdcan2,FDCAN_RX_FIFO1,&RxHeader,RxData2);
	
	/*Data is in RxData2*/
	if(RxHeader.Identifier==0x100)
	{
		MotorUnit->LEGMotor[(uint8_t)(RxData2[0]&0x0F)-2].MessageDecode(RxData2);
		MotorUnit->LEGMotor[(uint8_t)(RxData2[0]&0x0F)-2].UpdateMotor();
		tx_semaphore_put(&MotorLegSem);
	}
}

void CANFilterConfig(void)
{
	FDCAN_FilterTypeDef Filter;
	Filter.IdType = FDCAN_STANDARD_ID;	
	Filter.FilterIndex = 0;
	Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	Filter.FilterType = FDCAN_FILTER_MASK;
	Filter.FilterID1 = 0x0000;
	Filter.FilterID2 = 0x0000;

	
	HAL_FDCAN_ConfigFilter(&hfdcan2,&Filter);
	HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_RX_FIFO1_NEW_MESSAGE,0);
	
	
	Filter.FilterType = FDCAN_FILTER_DUAL;
	Filter.FilterID1 = 0x0205;/*Y6020*/
	Filter.FilterID2 = 0x0301;/*SuperCapacity*/
	Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	HAL_FDCAN_ConfigFilter(&hfdcan1,&Filter);
	
	Filter.FilterID1 = 0x0112;/*Gimbal*/
	Filter.FilterID2 = 0x0000;/*NONE*/
	Filter.FilterIndex = 1;
	HAL_FDCAN_ConfigFilter(&hfdcan1,&Filter);
	
	HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);

	
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

	HAL_FDCAN_Start(&hfdcan1);
	HAL_FDCAN_Start(&hfdcan2);
}