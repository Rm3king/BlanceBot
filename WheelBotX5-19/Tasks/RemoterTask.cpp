#include "RemoterTask.h"
#include "arm_math.h"
#include "ControlTask.h"

extern TX_BYTE_POOL UARTPool;
//kmj
extern cRobotControl *RobotControl;

TX_THREAD RemoterThread;
TX_QUEUE  RemoterRXQue;
uint8_t RemoterThreadStack[512]={0};//DTCM UART3ջ�ռ�
SRAM_SET_D2	uint8_t RemoterQueueStack[32]={0};  //UART3��Ϣ����ջ�ռ�

/*ң����*/
cRemoter *Msg_Remoter;
RC_Ctl_t rc_data_buf;//DTCM ң������������

void RemoterThreadFun(ULONG initial_input)
{
	Msg_Remoter = new cRemoter;
	uint8_t *precbuf = 0;
	
	/*ָ�����ջ�����*/
	tx_byte_allocate(&UARTPool, (VOID **)&precbuf, BUS_LEN*5, TX_NO_WAIT);
	Msg_Remoter->UART_BufferConfig(precbuf,BUS_LEN*5);
	
	/*ָ��RC��������*/
	Msg_Remoter->ConfigRemoter(&rc_data_buf);
	
	/*��ʼ���շ�����*/
	Msg_Remoter->UART_Init(UART4,DMA1,LL_DMA_STREAM_0,0);
	
	/*��������*/
	Msg_Remoter->Recieve_DMA(&Msg_Remoter->pbuf,BUS_LEN);
	
	/*������ջ��ϵ��쳣����*/
	tx_thread_sleep(100);
	tx_queue_flush(&RemoterRXQue);
	
	/*����Ϣ�����н��յĵ�ַ*/
	uint8_t *pbuf;
	
	/*�ݴ�*/
	int16_t CHBuf[7]={0};
	
	int16_t rmtmid	= (BUS_MAX + BUS_MIN)/2;
	float rmthalfrev = 2.0f/(float)(BUS_MAX - BUS_MIN); 
//	int16_t swcal0 = (BUS_MAX - BUS_MIN)/2;
//	int16_t swcal1 = swcal0 - BUS_MIN;
	for(;;)
	{
		/*��ȡң���������ź���*/
		if(tx_queue_receive(&RemoterRXQue, (VOID*)&pbuf, 300)==0x0A)
		{
			//��ʱ,ң��������
			Msg_Remoter->IsRCOffline = 1;
			tx_queue_receive(&RemoterRXQue, (VOID*)&pbuf, TX_WAIT_FOREVER);
			//��ʱ,ң������������
			Msg_Remoter->IsRCOffline = 0;
		}
		
		/*����CACHE*/
		SCB_CleanInvalidateDCache_by_Addr((uint32_t*)pbuf,BUS_LEN);
		
//		/*���帳ֵ*/
//		CHBuf[0] = 		(((int16_t)pbuf[1] 		  | ((int16_t)pbuf[2] << 8)) & 0x07FF)								- rmtmid; 
//		CHBuf[1] = 		((((int16_t)pbuf[2]>> 3)  | ((int16_t)pbuf[3] << 5)) & 0x07FF)								- rmtmid;
//		CHBuf[2] = 		((((int16_t)pbuf[3] >> 6) | ((int16_t)pbuf[4] << 2) | ((int16_t)pbuf[5] << 10)) & 0x07FF)	- rmtmid;
//		CHBuf[3] = 		((((int16_t)pbuf[5] >> 1) | ((int16_t)pbuf[6] << 7)) & 0x07FF)								- rmtmid;
//		CHBuf[4] = 		((((int16_t)pbuf[6] >> 4) | ((int16_t)pbuf[7] << 4)) & 0x07FF)								;
//		CHBuf[5] =		((((int16_t)pbuf[7] >> 7) | ((int16_t)pbuf[8] << 1) | ((int16_t)pbuf[9] << 9)) & 0x07FF)	;
//		CHBuf[6] = 		((((int16_t)pbuf[9] >> 2) | ((int16_t)pbuf[10]<< 6)) & 0x07FF)								- rmtmid;
//		
//		if(abs(CHBuf[0])<BUS_DEATHZOON)
//		{CHBuf[0] = 0;}
//		if(abs(CHBuf[1])<BUS_DEATHZOON)
//		{CHBuf[1] = 0;}
//		if(abs(CHBuf[2])<BUS_DEATHZOON)
//		{CHBuf[2] = 0;}
//		if(abs(CHBuf[3])<BUS_DEATHZOON)
//		{CHBuf[3] = 0;}
//		Msg_Remoter->IsRCOffline = pbuf[23];
//		Msg_Remoter->RC_Data->rmt.CH0 = 		rmthalfrev*(float)(CHBuf[0]);
//		Msg_Remoter->RC_Data->rmt.CH3 = 		rmthalfrev*(float)(CHBuf[1]);
//		Msg_Remoter->RC_Data->rmt.CH1 = 		rmthalfrev*(float)(CHBuf[2]);
//		Msg_Remoter->RC_Data->rmt.CH2 = 		rmthalfrev*(float)(CHBuf[3]);
//		Msg_Remoter->RC_Data->rmt.SW1 = 		(CHBuf[4]+swcal1)/swcal0;
//		Msg_Remoter->RC_Data->rmt.SW2 =			(CHBuf[5]+swcal1)/swcal0;
//		Msg_Remoter->RC_Data->rmt.WHEEL = 		rmthalfrev*(float)(CHBuf[6]);

		
		/*
			����ĺ��ˣ�CubeMX���˴��ڵ�����(��ң����У��λ��һ��)��ͷ�ļ��ĳ���DBUSģʽ
			�����ǲ��� ����-1��-1 ������ʱ������ ����������123
		*/
		/*���帳ֵ*/
		CHBuf[0] = 		(((int16_t)pbuf[0] 		  | ((int16_t)pbuf[1] << 8)) & 0x07FF)								- rmtmid; 
		CHBuf[1] = 		((((int16_t)pbuf[1]>> 3)  | ((int16_t)pbuf[2] << 5)) & 0x07FF)								- rmtmid;
		CHBuf[2] = 		((((int16_t)pbuf[2] >> 6) | ((int16_t)pbuf[3] << 2) | ((int16_t)pbuf[4] << 10)) & 0x07FF)	- rmtmid;
		CHBuf[3] = 		((((int16_t)pbuf[4] >> 1) | ((int16_t)pbuf[5] << 7)) & 0x07FF)								- rmtmid;
		
		CHBuf[4] = 		((pbuf[5] >> 4) & 0x000C) >> 2;
		CHBuf[5] =		((pbuf[5] >> 4) & 0x0003);
		
		CHBuf[6] = 		((((int16_t)pbuf[16] >> 2) | ((int16_t)pbuf[17]<< 6)) & 0x07FF)		-256;						
		
		/*ң������ֵ*/
		
		Msg_Remoter->RC_Data->rmt.CH0 = 		((float)(CHBuf[0]))*rmthalfrev;
		Msg_Remoter->RC_Data->rmt.CH1 = 		((float)(CHBuf[1]))*rmthalfrev;
		Msg_Remoter->RC_Data->rmt.CH2 = 		((float)(CHBuf[2]))*rmthalfrev;
		Msg_Remoter->RC_Data->rmt.CH3 = 		((float)(CHBuf[3]))*rmthalfrev;
		
		if(CHBuf[4]==3){CHBuf[4]=2;}
		else if(CHBuf[4]==2){CHBuf[4]=3;}
		if(CHBuf[5]==3){CHBuf[5]=2;}
		else if(CHBuf[5]==2){CHBuf[5]=3;}
		Msg_Remoter->RC_Data->rmt.SW1 = 		CHBuf[4];
		Msg_Remoter->RC_Data->rmt.SW2 =			CHBuf[5];
		
		Msg_Remoter->RC_Data->rmt.WHEEL = 		(float)(CHBuf[6]) /165;
		
		
		Msg_Remoter->RC_Data->mouse.x = ((int16_t)pbuf[6]) | ((int16_t)pbuf[7] << 8);
		Msg_Remoter->RC_Data->mouse.y = ((int16_t)pbuf[8]) | ((int16_t)pbuf[9] << 8);
		Msg_Remoter->RC_Data->mouse.z = ((int16_t)pbuf[10]) | ((int16_t)pbuf[11] << 8); 
		Msg_Remoter->RC_Data->mouse.press_l = pbuf[12];
		Msg_Remoter->RC_Data->mouse.press_r = pbuf[13];
		
		memcpy(&Msg_Remoter->RC_Data->key,&pbuf[14],2);
		
		/*kmj ң��������*/
		RobotControl->rc_data_1.ch0 =  CHBuf[0];
		RobotControl->rc_data_1.ch1 =  CHBuf[1];
		RobotControl->rc_data_1.ch2 =  CHBuf[2];
		RobotControl->rc_data_1.ch3 =  CHBuf[3];
		
		RobotControl->rc_data_2.sw_left = CHBuf[4];
		RobotControl->rc_data_2.sw_right = CHBuf[5];
		RobotControl->rc_data_2.vx = ((int16_t)pbuf[6]) | ((int16_t)pbuf[7] << 8);	
		RobotControl->rc_data_2.vy = ((int16_t)pbuf[8]) | ((int16_t)pbuf[9] << 8);		
		RobotControl->rc_data_2.vz = ((int16_t)pbuf[10]) | ((int16_t)pbuf[11] << 8);
		
		RobotControl->rc_data_3.press_l = pbuf[12];
		RobotControl->rc_data_3.press_r = pbuf[13];
		RobotControl->rc_data_3.keyboard.key_code = ((int16_t)pbuf[14]) | ((int16_t)pbuf[15] << 8);
		RobotControl->rc_data_3.wheel = CHBuf[6];
	}
}
void UART4_IRQHandler(void)
{
	if(LL_USART_IsActiveFlag_IDLE(UART4))
	{
		Msg_Remoter->IRQ_Rx();
		tx_queue_send(&RemoterRXQue, (VOID*)&Msg_Remoter->pbuf, TX_NO_WAIT);
		Msg_Remoter->Recieve_DMA(&Msg_Remoter->pbuf,BUS_LEN);
	}

}
void DMA1_Stream0_IRQHandler(void)
{

}
