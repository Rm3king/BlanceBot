#include "ReferTask.h"
#include "ReferDriver.h"
#include "ControlTask.h"
#include "LinkSolver.h"
#include "ControlTask.h"

cRef msgref_t;
cRef *Msg_Refer=&msgref_t;

extern cRobotControl *RobotControl;
extern TX_BYTE_POOL UARTPool;

extern cMotorUnit * MotorUnit;
extern cINS *INS;
TX_THREAD UART5Thread;
TX_THREAD DrawUIThread;
TX_QUEUE  RefQue;
TX_SEMAPHORE  RefSem;

uint8_t UART5ThreadStack[1024]={0};//DTCM UART2栈空间
uint8_t RefQueueStack[32]={0};  //DTCM UART2消息队列栈空间
uint8_t DrawUIThreadStack[1024]={0};//DTCM UI绘图栈空间
SRAM_SET_D2 robot_referee_status_t refdata;

uint8_t isUIRefresh_Flag = 0;

void DrawUIThreadFun(ULONG initial_input)
{
	

	uint8_t *txbuf = 0;
	tx_thread_sleep(30000);
	
	/*静态图形组*/
	uint8_t* puibuf0 = 0;	
	tx_byte_allocate(&UARTPool, (VOID **)&puibuf0, 128, TX_NO_WAIT);
	/*动态图形组*/
	uint8_t* puibuf1 = 0;	
	tx_byte_allocate(&UARTPool, (VOID **)&puibuf1, 128, TX_NO_WAIT);
	/*字符组*/
	uint8_t* puibuf2 = 0;	
	tx_byte_allocate(&UARTPool, (VOID **)&puibuf2, 60, TX_NO_WAIT);
	/*功能组*/
	uint8_t* puibuf3 = 0;	
	tx_byte_allocate(&UARTPool, (VOID **)&puibuf3, 128, TX_NO_WAIT);
	/*静态车道线组*/
	uint8_t* puibuf4 = 0;	
	tx_byte_allocate(&UARTPool, (VOID **)&puibuf4, 128, TX_NO_WAIT);
	/*静态枪线*/
	uint8_t* puibuf5 = 0;
	tx_byte_allocate(&UARTPool, (VOID **)&puibuf5, 128, TX_NO_WAIT);
	/*Out Loop, used to reset UI*/
	for(;;)
	{
		
//	if(isUIRefresh_Flag == 0 && RobotControl->isUIRefresh == 1)
//	{
//		 isUIRefresh_Flag = 1;
//		
	tx_thread_sleep(500);
	Msg_Refer->DrawUI.SetRobotID(Msg_Refer->robot_referee_status->ext_game_robot_status.robot_id);
	
	/*
		静态图形组
		指南针-圆
		电容-框
		低血量-框1
		低血量-框2s
		机身
	*/
	group_config_t group0={.GroupName=UI_GROUP_0,.GraphNum=UI_GROUP_GNUM_7,.GroupLayer=UI_LAYER_0,.pbuf=puibuf0};
	Msg_Refer->DrawUI.GraphGroupReg(&group0);

	/*
		动态图形组
		指南针-线
		电容-条
		低血量-条
		关节左
		关节右
	*/
	group_config_t group1={.GroupName=UI_GROUP_1,.GraphNum=UI_GROUP_GNUM_7,.GroupLayer=UI_LAYER_1,.pbuf=puibuf1};
	Msg_Refer->DrawUI.GraphGroupReg(&group1);
	
	/*
		静态车道线
	*/
	group_config_t group2={.GroupName=UI_GROUP_2,.GraphNum=UI_GROUP_GNUM_7,.GroupLayer=UI_LAYER_2,.pbuf=puibuf4};
	Msg_Refer->DrawUI.GraphGroupReg(&group2);
	
	/*
		静态枪线
	*/
	group_config_t group3={.GroupName=UI_GROUP_3,.GraphNum=UI_GROUP_GNUM_5,.GroupLayer=UI_LAYER_3,.pbuf=puibuf5};
	Msg_Refer->DrawUI.GraphGroupReg(&group3);	
		
//}
	
//		if(isUIRefresh_Flag == 1)
//		{
		tx_thread_sleep(5000);

		/* 清除UI图层 */
		Msg_Refer->DrawUI.LayerDelete(puibuf3,UI_LAYER_ALL);
		SCB_CleanInvalidateDCache_by_Addr((uint32_t*)puibuf3,128);
		Msg_Refer->Transmit_DMA(puibuf3,128);
		tx_thread_sleep(100);

		/*机器人信息1*/
		memset(puibuf2,0,128);
		Msg_Refer->DrawUI.CharacterUpdate(puibuf2,UI_LAYER_2,(char*)"UI1",UI_GRAPHE_OPERATE_ADD,UI_COLOR_GREEN,2,650,560,"STATUE");
		SCB_CleanInvalidateDCache_by_Addr((uint32_t*)puibuf2,64);
		Msg_Refer->Transmit_DMA(puibuf2,60);
		tx_thread_sleep(50);
		memset(puibuf2,0,60);
		
		/*机器人信息2*/
		memset(puibuf2,0,128);
		Msg_Refer->DrawUI.CharacterUpdate(puibuf2,UI_LAYER_2,(char*)"UI2",UI_GRAPHE_OPERATE_ADD,UI_COLOR_GREEN,2,1500,560,"MOTOR");
		SCB_CleanInvalidateDCache_by_Addr((uint32_t*)puibuf2,64);
		Msg_Refer->Transmit_DMA(puibuf2,60);
		tx_thread_sleep(50);
		memset(puibuf2,0,60);
		
		/*ID*/
		Msg_Refer->DrawUI.CharacterUpdate(puibuf2,UI_LAYER_2,(char*)"UI3",UI_GRAPHE_OPERATE_ADD,UI_COLOR_GREEN,5,650,720,"ID");
		SCB_CleanInvalidateDCache_by_Addr((uint32_t*)puibuf2,64);
		Msg_Refer->Transmit_DMA(puibuf2,60);
		tx_thread_sleep(50);
		memset(puibuf2,0,60);
    
		/* autoAimFlag */
		memset(puibuf2,0,128);
		Msg_Refer->DrawUI.CharacterUpdate(puibuf2,UI_LAYER_2,(char*)"UI4",UI_GRAPHE_OPERATE_ADD,UI_COLOR_GREEN,2,650,880,"AutoAim");
		SCB_CleanInvalidateDCache_by_Addr((uint32_t*)puibuf2,64);
		Msg_Refer->Transmit_DMA(puibuf2,60);
		tx_thread_sleep(50);
		memset(puibuf2,0,60);

		/*静态图形组设置*/
		/*指南针头*/
		Msg_Refer->DrawUI.GraphGroupUpdateLine(UI_GROUP_0,UI_GROUP_GID_0,(char*)"S01",UI_GRAPHE_OPERATE_ADD,UI_COLOR_RED_BLUE,5,960,270,960,290);
		/*指南针圆*/
		Msg_Refer->DrawUI.GraphGroupUpdateCircle(UI_GROUP_0,UI_GROUP_GID_1,(char*)"S02",UI_GRAPHE_OPERATE_ADD,UI_COLOR_WHITE,5,960,240,50);
		/*电容*/
		Msg_Refer->DrawUI.GraphGroupUpdateRectangle(UI_GROUP_0,UI_GROUP_GID_2,(char*)"S03",UI_GRAPHE_OPERATE_ADD,UI_COLOR_WHITE,2,640,99,1280,121);
		/*关节*/
		Msg_Refer->DrawUI.GraphGroupUpdateLine(UI_GROUP_0,UI_GROUP_GID_3,(char*)"S04",UI_GRAPHE_OPERATE_ADD,UI_COLOR_WHITE,5,1200,490,1200,580);
		/*血量*/
		Msg_Refer->DrawUI.GraphGroupUpdateRectangle(UI_GROUP_0,UI_GROUP_GID_4,(char*)"S05",UI_GRAPHE_OPERATE_ADD,UI_COLOR_RED_BLUE,5,700-50,700-50,700+50,700+50);
		/*机体小线*/
		Msg_Refer->DrawUI.GraphGroupUpdateRectangle(UI_GROUP_0,UI_GROUP_GID_5,(char*)"S06",UI_GRAPHE_OPERATE_ADD,UI_COLOR_WHITE,2,1300,800,1300,540);


		Msg_Refer->DrawUI.GrapgGroupPack(UI_GROUP_0,txbuf);
		SCB_CleanInvalidateDCache_by_Addr((uint32_t*)puibuf0,128);
		Msg_Refer->Transmit_DMA(puibuf0,128);
		tx_thread_sleep(100);
		
		/*静态枪线设置*/
		Msg_Refer->DrawUI.GraphGroupUpdateLine(UI_GROUP_3,UI_GROUP_GID_0,(char*)"M01",UI_GRAPHE_OPERATE_ADD,UI_COLOR_WHITE,3,910,490,1010,490);
		Msg_Refer->DrawUI.GraphGroupUpdateLine(UI_GROUP_3,UI_GROUP_GID_1,(char*)"M02",UI_GRAPHE_OPERATE_ADD,UI_COLOR_WHITE,3,930,460,990,460);
		Msg_Refer->DrawUI.GraphGroupUpdateLine(UI_GROUP_3,UI_GROUP_GID_2,(char*)"M03",UI_GRAPHE_OPERATE_ADD,UI_COLOR_WHITE,3,930,430,990,430);
		Msg_Refer->DrawUI.GraphGroupUpdateLine(UI_GROUP_3,UI_GROUP_GID_3,(char*)"M04",UI_GRAPHE_OPERATE_ADD,UI_COLOR_YELLOW,5,960,400,960,530);
		Msg_Refer->DrawUI.GrapgGroupPack(UI_GROUP_3,txbuf);
		SCB_CleanInvalidateDCache_by_Addr((uint32_t*)puibuf5,128);
		Msg_Refer->Transmit_DMA(puibuf5,128);
		tx_thread_sleep(100);
		/*静态车道线设置*/
		/*左撇*/
		Msg_Refer->DrawUI.GraphGroupUpdateLine(UI_GROUP_2,UI_GROUP_GID_0,(char*)"T01",UI_GRAPHE_OPERATE_ADD,UI_COLOR_RED_BLUE,5,0,0,500,300);
		/*左横*/
		Msg_Refer->DrawUI.GraphGroupUpdateLine(UI_GROUP_2,UI_GROUP_GID_1,(char*)"T02",UI_GRAPHE_OPERATE_ADD,UI_COLOR_RED_BLUE,5,500,300,800,300);
		/*右撇*/
		Msg_Refer->DrawUI.GraphGroupUpdateLine(UI_GROUP_2,UI_GROUP_GID_2,(char*)"T03",UI_GRAPHE_OPERATE_ADD,UI_COLOR_RED_BLUE,5,1920,0,1420,300);
		/*右横*/
		Msg_Refer->DrawUI.GraphGroupUpdateLine(UI_GROUP_2,UI_GROUP_GID_3,(char*)"T04",UI_GRAPHE_OPERATE_ADD,UI_COLOR_RED_BLUE,5,1420,300,1120,300);
		/*机身圆*/
		//Msg_Refer->DrawUI.GraphGroupUpdateRectangle(UI_GROUP_2,UI_GROUP_GID_4,(char*)"T05",UI_GRAPHE_OPERATE_ADD,UI_COLOR_RED_BLUE,2,1250,775,1350,825);
		Msg_Refer->DrawUI.GraphGroupUpdateCircle(UI_GROUP_2,UI_GROUP_GID_4,(char*)"T05",UI_GRAPHE_OPERATE_ADD,UI_COLOR_WHITE,5,1300,800,50);
		
		/*机身姿态线 */
//		Msg_Refer->DrawUI.GraphGroupUpdateLine(UI_GROUP_2,UI_GROUP_GID_5,(char*)"T06",UI_GRAPHE_OPERATE_ADD,UI_COLOR_RED_BLUE,5,1250,800,1350,800);
		Msg_Refer->DrawUI.GrapgGroupPack(UI_GROUP_2,txbuf);
		SCB_CleanInvalidateDCache_by_Addr((uint32_t*)puibuf4,128);
		Msg_Refer->Transmit_DMA(puibuf4,128);
		tx_thread_sleep(100);
		
		/*动态图形组设置*/
		/*指南针*/
		Msg_Refer->DrawUI.GraphGroupUpdateLine(UI_GROUP_1,UI_GROUP_GID_0,(char*)"D01",UI_GRAPHE_OPERATE_ADD,UI_COLOR_RED_BLUE,5,960,240,960,290);
		/*电容*/
		Msg_Refer->DrawUI.GraphGroupUpdateLine(UI_GROUP_1,UI_GROUP_GID_1,(char*)"D02",UI_GRAPHE_OPERATE_ADD,UI_COLOR_RED_BLUE,20,641,110,1279,110);
		/*关节*/
		Msg_Refer->DrawUI.GraphGroupUpdateLine(UI_GROUP_1,UI_GROUP_GID_2,(char*)"D03",UI_GRAPHE_OPERATE_ADD,UI_COLOR_RED_BLUE,5,1190,540,1210,540);
		/*低血量*/
		Msg_Refer->DrawUI.GraphGroupUpdateLine(UI_GROUP_1,UI_GROUP_GID_3,(char*)"D04",UI_GRAPHE_OPERATE_ADD,UI_COLOR_ORANGE,5,700+60,700-50,700+60,700+50);
		/*左关节*/
		Msg_Refer->DrawUI.GraphGroupUpdateLine(UI_GROUP_1,UI_GROUP_GID_4,(char*)"D05",UI_GRAPHE_OPERATE_ADD,UI_COLOR_BLUEGREEN,5,1200,900,1200,700);
		/*右关节*/
		Msg_Refer->DrawUI.GraphGroupUpdateLine(UI_GROUP_1,UI_GROUP_GID_5,(char*)"D06",UI_GRAPHE_OPERATE_ADD,UI_COLOR_ORANGE,5,1200,900,1200,700);
		/*机身姿态线      */
		Msg_Refer->DrawUI.GraphGroupUpdateLine(UI_GROUP_1,UI_GROUP_GID_6,(char*)"D07",UI_GRAPHE_OPERATE_ADD,UI_COLOR_RED_BLUE,5,1250,800,1350,800);
		Msg_Refer->DrawUI.GrapgGroupPack(UI_GROUP_1,txbuf);
		SCB_CleanInvalidateDCache_by_Addr((uint32_t*)puibuf1,128);
		Msg_Refer->Transmit_DMA(puibuf1,128);
		tx_thread_sleep(50);

		
		/*Inside, used for ui freshing*/
		for(;;)
		{
			
			/*动态图形组刷新*/
			/*指南针*/
			float rad=RobotControl->ChasisControl.GetErrWithGimRaw()*PI;
			uint16_t Dx=50*arm_sin_f32(rad);uint16_t Dy=50*arm_cos_f32(rad);
			Msg_Refer->DrawUI.GraphGroupUpdateLine(UI_GROUP_1,UI_GROUP_GID_0,(char*)"D01",UI_GRAPHE_OPERATE_CHANGE,UI_COLOR_WHITE,5,960-Dx,240-Dy,960+0.8f*Dx,240+0.8f*Dy);
			/*电容*/
			Msg_Refer->DrawUI.GraphGroupUpdateLine(UI_GROUP_1,UI_GROUP_GID_1,(char*)"D02",UI_GRAPHE_OPERATE_CHANGE,UI_COLOR_RED_BLUE,20,641,110,641+638*RobotControl->ChasisControl.SCPowerGet(),110);
			/*关节*/
			uint16_t len = 490+RobotControl->ChasisControl.GetLegLenFlag()*30.0;
			Msg_Refer->DrawUI.GraphGroupUpdateLine(UI_GROUP_1,UI_GROUP_GID_2,(char*)"D03",UI_GRAPHE_OPERATE_CHANGE,UI_COLOR_RED_BLUE,5,1190,len,1210,len);
			/*低血量*/
			uint16_t HP[5]={0};uint8_t LID = 0;uint16_t minHP=0;eUI_COLOR HPcolor;
			if(Msg_Refer->robot_referee_status->ext_game_robot_status.robot_id<100)//We are Red
			{
				HP[0]=Msg_Refer->robot_referee_status->ext_game_robot_HP.blue_1_robot_HP;
				HP[1]=Msg_Refer->robot_referee_status->ext_game_robot_HP.blue_2_robot_HP;
				HP[2]=Msg_Refer->robot_referee_status->ext_game_robot_HP.blue_3_robot_HP;
				HP[3]=Msg_Refer->robot_referee_status->ext_game_robot_HP.blue_4_robot_HP;
				HP[4]=Msg_Refer->robot_referee_status->ext_game_robot_HP.blue_5_robot_HP;
				HPcolor = UI_COLOR_BLUEGREEN;
			}
			else
			{
				HP[0]=Msg_Refer->robot_referee_status->ext_game_robot_HP.red_1_robot_HP;
				HP[1]=Msg_Refer->robot_referee_status->ext_game_robot_HP.red_2_robot_HP;
				HP[2]=Msg_Refer->robot_referee_status->ext_game_robot_HP.red_3_robot_HP;
				HP[3]=Msg_Refer->robot_referee_status->ext_game_robot_HP.red_4_robot_HP;
				HP[4]=Msg_Refer->robot_referee_status->ext_game_robot_HP.red_5_robot_HP;
				HPcolor = UI_COLOR_VIOLETRED;
			}
			minHP = 500;
			for(uint8_t i=1;i<5;i++)
			{
				if(HP[i]>0)
				{
					if(HP[i]<minHP)
					{
						minHP=HP[i];
						LID = i+1;
					}
				}
			}
			Msg_Refer->DrawUI.GraphGroupUpdateLine(UI_GROUP_1,UI_GROUP_GID_3,(char*)"D04",UI_GRAPHE_OPERATE_CHANGE,HPcolor,5,700+60,700-50,700+60,650+minHP/2);
			/*左关节*/
			float lendx,lendy;
			lendx = -700*RobotControl->ChasisControl.MotorUnits->LinkSolver[0].GetPendulumLen()*arm_cos_f32(RobotControl->ChasisControl.MotorUnits->LinkSolver[0].GetPendulumRadian());
			lendy = 700*RobotControl->ChasisControl.MotorUnits->LinkSolver[0].GetPendulumLen()*arm_sin_f32(RobotControl->ChasisControl.MotorUnits->LinkSolver[0].GetPendulumRadian());
			Msg_Refer->DrawUI.GraphGroupUpdateLine(UI_GROUP_1,UI_GROUP_GID_4,(char*)"D05",UI_GRAPHE_OPERATE_CHANGE,UI_COLOR_BLUEGREEN,5,1300,800,1300-lendx,800-lendy);
			/*右关节*/
			lendx = -700*RobotControl->ChasisControl.MotorUnits->LinkSolver[1].GetPendulumLen()*arm_cos_f32( RobotControl->ChasisControl.MotorUnits->LinkSolver[1].GetPendulumRadian());
			lendy = 700*RobotControl->ChasisControl.MotorUnits->LinkSolver[1].GetPendulumLen()*arm_sin_f32( RobotControl->ChasisControl.MotorUnits->LinkSolver[1].GetPendulumRadian());
			Msg_Refer->DrawUI.GraphGroupUpdateLine(UI_GROUP_1,UI_GROUP_GID_5,(char*)"D06",UI_GRAPHE_OPERATE_CHANGE,UI_COLOR_ORANGE,5,1300,800,1300-lendx,800-lendy);
			/* 机体姿态 */
			float pitch_rad = QCS.Pitch(INS->Q); 
			float pitch_dx,pitch_dy;
			pitch_dx = 50*arm_cos_f32(pitch_rad);
			pitch_dy = 50*arm_sin_f32(pitch_rad);
			Msg_Refer->DrawUI.GraphGroupUpdateLine(UI_GROUP_1,UI_GROUP_GID_6,(char*)"D07",UI_GRAPHE_OPERATE_CHANGE,UI_COLOR_RED_BLUE,5,1300-pitch_dx,800-pitch_dy,1300+pitch_dx,800+pitch_dy);
			Msg_Refer->DrawUI.GrapgGroupPack(UI_GROUP_1,txbuf);
			SCB_CleanInvalidateDCache_by_Addr((uint32_t*)puibuf1,128);
			Msg_Refer->Transmit_DMA(puibuf1,128);
			tx_thread_sleep(40);

			static uint8_t Num=0;
			if(++Num>=4)
			{
				Num = 0;
				/*机器人状态1*/
				char Mode[5]={0};
				switch(RobotControl->CheckRobotMode())
				{
					case 0:
						Mode[0]='I';Mode[1]='D';Mode[2]='L';Mode[3]='E';
						break;
					case 1:
						Mode[0]='E';Mode[1]='S';Mode[2]='C';Mode[3]='A';
						break;
					
					case 2:
						Mode[0]='N';Mode[1]='O';Mode[2]='R';Mode[3]='M';
						break;
				}	

				char Follow[3]={0};
				if(RobotControl->ChasisControl.GetChasisMode()==ROBOTPART_CHASIS_SIDEMODE)
				{Follow[0]='S';Follow[1]='I';}
				else if(RobotControl->ChasisControl.GetCFGENFlag())
				{Follow[0]='F';Follow[1]='O';}
				else{Follow[0]='D';Follow[1]='E';}
			
				float Leglen=(RobotControl->ChasisControl.MotorUnits->LinkSolver[0].GetPendulumLen()+RobotControl->ChasisControl.MotorUnits->LinkSolver[1].GetPendulumLen())*500.0f;
				Msg_Refer->DrawUI.CharacterUpdate(puibuf2,UI_LAYER_2,(char*)"UI1",UI_GRAPHE_OPERATE_CHANGE,UI_COLOR_GREEN,2,650,570,"%s %s \nLen:%3.1f \nSC:%2.2f",Mode,Follow,Leglen,RobotControl->ChasisControl.SCPowerGet()*100.0f);
				/*Clear Cache*/
				SCB_CleanInvalidateDCache_by_Addr((uint32_t*)puibuf2,60);
				Msg_Refer->Transmit_DMA(puibuf2,60);
				tx_thread_sleep(40);
				memset(puibuf2,0,60);
			}
			else if(Num==1)
			{
				/*机器人状态2*/
				char Shot[3]={0};
				if(RobotControl->ComDown.TriggleMode==3)
				{Shot[0]='S';Shot[1]='I';}
				else
				{Shot[0]='M';Shot[1]='U';}

				char Cap[2]={0};
				if(RobotControl->ComDown.CapOpen)
				{Cap[0]='O';}
				else{Cap[0]='C';}

				Msg_Refer->DrawUI.CharacterUpdate(puibuf2,UI_LAYER_2,(char*)"UI2",UI_GRAPHE_OPERATE_CHANGE,UI_COLOR_GREEN,2,1500,570,"%s %s\n%2d\t%2d\n%2d\t%2d",Shot,Cap,RobotControl->ChasisControl.MotorUnits->LEGMotor[0].GetTem(),RobotControl->ChasisControl.MotorUnits->LEGMotor[1].GetTem(),RobotControl->ChasisControl.MotorUnits->LEGMotor[2].GetTem(),RobotControl->ChasisControl.MotorUnits->LEGMotor[3].GetTem());
				SCB_CleanInvalidateDCache_by_Addr((uint32_t*)puibuf2,60);
				Msg_Refer->Transmit_DMA(puibuf2,60);
				tx_thread_sleep(40);
				memset(puibuf2,0,60);
			
			}
			else if(Num==2)
			{
				/*机器人血量*/
				Msg_Refer->DrawUI.CharacterUpdate(puibuf2,UI_LAYER_2,(char*)"UI3",UI_GRAPHE_OPERATE_CHANGE,HPcolor,5,690,722,"%d",LID);
				/*Clear Cache*/
				SCB_CleanInvalidateDCache_by_Addr((uint32_t*)puibuf2,60);
				Msg_Refer->Transmit_DMA(puibuf2,60);
				tx_thread_sleep(40);
				memset(puibuf2,0,60);		
			}
			else if(Num==3)
			{
				/*视觉是否锁定成功*/	
				Msg_Refer->DrawUI.CharacterUpdate(puibuf2,UI_LAYER_2,(char*)"UI4",UI_GRAPHE_OPERATE_CHANGE,UI_COLOR_YELLOW,5,690,880,"%d",RobotControl->ComUP.autoAim_detect_flag);				

				/*Clear Cache*/
				SCB_CleanInvalidateDCache_by_Addr((uint32_t*)puibuf2,60);
				Msg_Refer->Transmit_DMA(puibuf2,60);
				tx_thread_sleep(40);
				memset(puibuf2,0,60);					
			}
			if(RobotControl->isUIRefresh)
			{
			tx_thread_sleep(100);
			break;
			}
		}
	
//	 }
	}
 
}

void UART5ThreadFun(ULONG initial_input)
{
	/*指定接收缓冲区*/
	uint8_t *precbuf = 0;
	tx_byte_allocate(&UARTPool, (VOID **)&precbuf, 4096, TX_NO_WAIT);
	Msg_Refer->UART_BufferConfig(precbuf,4096);
	
	/*指定裁判系统数据区域*/
	Msg_Refer->RefDataConfig(&refdata);
	
	/*初始化串口收发程序*/
	Msg_Refer->UART_Init(UART5,DMA1,LL_DMA_STREAM_7,0,DMA2,LL_DMA_STREAM_1);
	
	/*初始化M2M*/
	Msg_Refer->M2M_Init(DMA2,LL_DMA_STREAM_0);
	
	tx_thread_sleep(1000);
	/*开始数据接收*/
	Msg_Refer->Recieve_DMA(&Msg_Refer->Ref_Msg.data ,256);
	
	/*裁判系统数据包信息*/
	ref_msg_t msg_buf={0};
//	tx_thread_sleep(TX_WAIT_FOREVER);
	for(;;)
	{
		/*获取裁判系统包信息*/
		if(tx_queue_receive(&RefQue, (VOID*)&msg_buf, 1000)==0x0A)
		{
			//超时,裁判系统离线
			Msg_Refer->IsRefOffline = 1;
			tx_queue_receive(&RefQue, (VOID*)&msg_buf, TX_WAIT_FOREVER);
		}
		/*裁判系统连接正常*/
		Msg_Refer->IsRefOffline = 0;
		SCB_CleanInvalidateDCache_by_Addr((uint32_t*)msg_buf.data,msg_buf.Size);
		/*更新发送数据*/
		Msg_Refer->M2M_Update(msg_buf.data,msg_buf.Size);
		/*DMA发送*/
		for(uint8_t i=0;i<Msg_Refer->DMA_Msg.packnum;i++)
		{
			Msg_Refer->M2M_Transmit(i);
			tx_semaphore_get(&RefSem,TX_WAIT_FOREVER);	
			/*Clear Cache*/
			SCB_CleanInvalidateDCache_by_Addr((uint32_t*)Msg_Refer->DMA_Msg.dstaddr[i],Msg_Refer->DMA_Msg.packlen[i]);
		}
	}
}

void UART5_IRQHandler(void)
{
	if(LL_USART_IsActiveFlag_IDLE(UART5))
	{
		Msg_Refer->Ref_Msg.Size = Msg_Refer->IRQ_Rx();
		if(Msg_Refer->Ref_Msg.data[0]==0xA5)
		{	/*包头正确，通过邮箱发送*/
			tx_queue_send(&RefQue, (VOID*)&Msg_Refer->Ref_Msg, TX_NO_WAIT);
		}
		Msg_Refer->Recieve_DMA(&Msg_Refer->Ref_Msg.data ,256);
	}
}


void DMA2_Stream1_IRQHandler(void)
{
	Msg_Refer->IRQ_Tx();
}

void DMA2_Stream0_IRQHandler(void)
{
	if(LL_DMA_IsActiveFlag_TC0(DMA2))
	{
		LL_DMA_ClearFlag_TC0(DMA2);
		LL_DMA_DisableStream(DMA2,LL_DMA_STREAM_0);
		tx_semaphore_put(&RefSem);
	}
	else{
		LL_DMA_ClearFlag_DME0(DMA2);
		LL_DMA_ClearFlag_TE0(DMA2);
		LL_DMA_ClearFlag_FE0(DMA2);
	}
}