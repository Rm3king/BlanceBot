#include "MessageTask.h"


extern TX_BYTE_POOL UARTPool;

TX_THREAD MessgaeThread;

uint8_t MessageThreadStack[1024]={0};//DTCM UART7ջ�ռ�

/*���򴮿ڷ��Ϳ���*/
cMSG  *Msg_UART7;

/*��������*/


/*��������*/
uint8_t *rbuf = 0;
static void DispTaskInfo(void);

void MessageThreadFun(ULONG initial_input)
{
	Msg_UART7 = new cMSG;
	uint8_t *pbuf = 0;
	/*ָ�����ջ�����*/
	tx_byte_allocate(&UARTPool, (VOID **)&pbuf, 128, TX_NO_WAIT);
	Msg_UART7->UART_BufferConfig(pbuf,128);

	/*��ʼ���շ�����*/
	Msg_UART7->UART_Init(UART7,DMA1,LL_DMA_STREAM_2,0,DMA1,LL_DMA_STREAM_3);
	tx_thread_sleep(1000);
	
	/*���Խ���*/
	Msg_UART7->Recieve_DMA(&rbuf,10);
	
	/*Printf��������*/
	uint8_t *txbuf = 0;
	tx_byte_allocate(&UARTPool, (VOID **)&txbuf, 256, TX_NO_WAIT);
	Msg_UART7->SetPrintf(txbuf,256);
	txbuf[0]=0xAA;
	
	ULONG timer = 0;
	float EuAng[3]={0};
	for(;;)
	{	
		timer = tx_time_get();
		//Msg_UART7->Printf("RPML=%d,RPMR=%d\n",M3508L->GetRpm(),M3508R->GetRpm());
		tx_thread_sleep(10);
	}
}

void UART7_IRQHandler(void)
{
	if(LL_USART_IsActiveFlag_IDLE(UART7))
	{
		Msg_UART7->IRQ_Rx();
		Msg_UART7->Transmit(rbuf,Msg_UART7->GetRL(),0xFF);
		Msg_UART7->Recieve_DMA(&rbuf,10);
	}
}
void DMA1_Stream2_IRQHandler(void)
{

}
void DMA1_Stream3_IRQHandler(void)
{
	if(LL_DMA_IsActiveFlag_TC4(DMA1))
	{
		Msg_UART7->IRQ_Tx();
	}
}


/*
*********************************************************************************************************
*    �� �� ��: DispTaskInfo
*    ����˵��: ��ThreadX������Ϣͨ�����ڴ�ӡ����
*    ��    �Σ���
*    �� �� ֵ: ��
*********************************************************************************************************
*/
extern TX_THREAD LedThread;
static void DispTaskInfo(void)
{
    TX_THREAD      *p_tcb;            /* ����һ��������ƿ�ָ�� */

    p_tcb = &LedThread;
    
    /* ��ӡ���� */
//    Msg_UART4->Printf("\n===============================================================\r\n");
//    Msg_UART4->Printf("OS CPU Usage = %5.2f%%\r\n", OSCPUUsage);
    Msg_UART7->Printf("\n===============================================================\r\n");
    Msg_UART7->Printf("   ID     Prio     StackSize   CurStack    MaxStack   Taskname\r\n");
	uint8_t ID = 0;
    /* ����������ƿ��б�(TCB list)����ӡ���е���������ȼ������� */
    while (p_tcb != (TX_THREAD *)0) 
    {
        
         Msg_UART7->Printf("   %2d     %2d        %5d      %5d       %5d      %s\r\n", 
                    ID++,
					p_tcb->tx_thread_priority,
                    p_tcb->tx_thread_stack_size,
                    (int)p_tcb->tx_thread_stack_end - (int)p_tcb->tx_thread_stack_ptr,
                    (int)p_tcb->tx_thread_stack_end - (int)p_tcb->tx_thread_stack_highest_ptr,
                    p_tcb->tx_thread_name);


        p_tcb = p_tcb->tx_thread_created_next;

        if(p_tcb == &LedThread) break;
    }
}



