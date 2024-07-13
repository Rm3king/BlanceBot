#include "ControlTask.h"
#include "IMUTask.h"

#include "Filter.h"
#include "RemoterTask.h"

#include <string>
#include <map>
using namespace std;



extern cRef *Msg_Refer;
extern cRemoter *Msg_Remoter;
extern cMotorUnit *MotorUnit;
extern cINS *INS;


float ForwardSpeed = 0.0f;
float SpeedTarget = 0.0f;
float TargetAngel = 0.0f;
float Lens  = LEGMID;
uint8_t fireflag = 0; 
uint8_t jump_count=0;
uint8_t sub_len_flag=0;
uint8_t add_len_flag=0;
extern TX_SEMAPHORE CANUpSem;

cRobotControl RobotControlt;
cRobotControl *RobotControl = &RobotControlt;

TX_THREAD RoboCTRThread;
uint8_t RoboCTRThreadStack[2048]={0};

void RoboCTRThreadFun(ULONG initial_input)
{
	auto fun =[=](int x,int y)->bool{return x%10<y%10;};
	map<int,string> index_map{{1,"hello"},{2,"world"},{3,"!"}};
	for(const auto &e : index_map)
	{
	
	}
	
	RobotControl->ChasisControl.MotorUnits = MotorUnit;
	
	cVelFusionKF tVelKF;
	RobotControl->ChasisControl.SetVelKF(&tVelKF);
	
	tx_thread_sleep(100);

	/*Input refer system data point*/
	RobotControl->SetRefSysDataSource(&Msg_Refer->robot_referee_status->ext_game_robot_status,&Msg_Refer->robot_referee_status->ext_power_heat_data);
	RobotControl->SetINS(INS);
	
	/*Used to determind RobotMode change*/
	uint8_t LastKey_SW1=Msg_Remoter->RC_Data->rmt.SW1;
	uint8_t LastSW2=2;
	
	/*This value is determind by cool rate*/
	uint8_t RobotPart_TroggleMMode = ROBOTPART_TRIGGLE_MMODE0;
	/*This value is determind by ammo speed limit*/
	uint8_t RobotPart_BoosterMode  = ROBOTPART_BOOSTER_15MPS;
	
	
	ULONG timer = tx_time_get();
	for(;;)
	{
		tx_thread_sleep_until(&timer,5);
		timer = tx_time_get();
		
		/*Check if remoter offline*/
		if(Msg_Remoter->IsRCOffline)
		{
			/*if remote offline, RobotMode and all part will get into IDLE*/
			RobotControl->SetRobotMode(ROBOTMODE_IDLE);
			
			RobotControl->SetPartMode(ROBOTPART_ID_LEG,ROBOTPART_STATUS_IDLE);                          
			RobotControl->SetPartMode(ROBOTPART_ID_BLC,ROBOTPART_STATUS_IDLE);
			RobotControl->SetPartMode(ROBOTPART_ID_BST,ROBOTPART_STATUS_IDLE);
			RobotControl->SetPartMode(ROBOTPART_ID_GIM,ROBOTPART_STATUS_IDLE);
		}
		
		/*Error Detection*/
		
		/*Refresh RobotDeath*/
		if(!RobotControl->GetRobotHP())
		{
		RobotControl->ComDown.RobotDeath = 1;
		}
		else
		{RobotControl->ComDown.RobotDeath = 0;}
		
		
/*********************************************************************************/
		/*RobotMode Change*/
		/*Press Z to emergancy turn to RobotESCAPE*/
		uint8_t IsModeChangeValid = 0;//To show is mode change happened.

		if( (LastKey_SW1!=Msg_Remoter->RC_Data->rmt.SW1)||((Msg_Remoter->RC_Data->key.Z==1)&&(Msg_Remoter->RC_Data->key.CTRL==1)) )
		{
			/*SW1->1, robot in SafeMode*/
			if(Msg_Remoter->RC_Data->rmt.SW1==1)
			{
				RobotControl->SetRobotMode(ROBOTMODE_IDLE);
				/*
					机器人空闲模式
				
					Leg:		IDLE
					Blance:		IDLE
					Gimbal:		IDLE
					Booster:	IDLE
				*/
				/*检查状态,设置为IDLE,异常状态将保留*/
				RobotControl->SetPartMode(ROBOTPART_ID_LEG,ROBOTPART_STATUS_IDLE);
				RobotControl->SetPartMode(ROBOTPART_ID_BLC,ROBOTPART_STATUS_IDLE);
				RobotControl->SetPartMode(ROBOTPART_ID_BST,ROBOTPART_STATUS_IDLE);
				RobotControl->SetPartMode(ROBOTPART_ID_GIM,ROBOTPART_STATUS_IDLE);
			}
			/*逃脱模式*/
			else if((Msg_Remoter->RC_Data->rmt.SW1==2)||((Msg_Remoter->RC_Data->key.Z==1)&&(Msg_Remoter->RC_Data->key.CTRL==1)) )
			{
				RobotControl->SetRobotMode(ROBOTMODE_ESCAPE);
				/*
					机器人逃脱模式
					Leg:		IDLE
					Blance:		IDLE
					Gimbal:		NORMAL
					Booster:	IDLE
				*/
				

				/*检查状态*/
				RobotControl->SetPartMode(ROBOTPART_ID_LEG,ROBOTPART_STATUS_IDLE);
				RobotControl->SetPartMode(ROBOTPART_ID_BLC,ROBOTPART_STATUS_IDLE);
				
				if(RobotControl->CheckPartMode(ROBOTPART_ID_BST)!=ROBOTPART_STATUS_NORMAL)
				{RobotControl->SetPartMode(ROBOTPART_ID_BST,ROBOTPART_STATUS_START);}
				if(RobotControl->CheckPartMode(ROBOTPART_ID_GIM)!=ROBOTPART_STATUS_NORMAL)
				{RobotControl->SetPartMode(ROBOTPART_ID_GIM,ROBOTPART_STATUS_START);}
				
			}
			/*正常模式*/
			else if(Msg_Remoter->RC_Data->rmt.SW1==3)
			{	
				RobotControl->SetRobotMode(ROBOTMODE_NORMAL);
				/*
					机器人正常模式
				
					Leg:		NORMAL 需要启动模式
					Blance:		NORMAL 需要启动模式
					Gimbal:		NORMAL
					Booster:	NORMAL
				*/
				
				/*检查状态*/
				if((RobotControl->CheckPartMode(ROBOTPART_ID_LEG)&0x03)!= ROBOTPART_STATUS_NORMAL)
				{RobotControl->SetPartMode(ROBOTPART_ID_LEG,ROBOTPART_STATUS_START);}
				if((RobotControl->CheckPartMode(ROBOTPART_ID_BLC)&0x03)!= ROBOTPART_STATUS_NORMAL)
				{RobotControl->SetPartMode(ROBOTPART_ID_BLC,ROBOTPART_STATUS_START);}
				if((RobotControl->CheckPartMode(ROBOTPART_ID_BST)&0x03)!= ROBOTPART_STATUS_NORMAL)
				{RobotControl->SetPartMode(ROBOTPART_ID_BST,ROBOTPART_STATUS_START);}
				if((RobotControl->CheckPartMode(ROBOTPART_ID_GIM)&0x03)!= ROBOTPART_STATUS_NORMAL)
				{RobotControl->SetPartMode(ROBOTPART_ID_GIM,ROBOTPART_STATUS_START);}
				
				/*
					ENABLE LEG
				*/
//				if(RobotControl->GetRobotRemainHeat() < ROBOTPART_HEATSAFT_VALUE)
//	  	 {
//				RobotControl->rc_data_3.gimbal_fire_flag = 0;
//			 }
//			 else
//			 {
//				 RobotControl->rc_data_3.gimbal_fire_flag = 1;
//			 }
			}
			
			IsModeChangeValid = 1;
			/*Refresh*/
			LastKey_SW1 = Msg_Remoter->RC_Data->rmt.SW1;
		}
			
/*********************************************************************************/
		/*Robot input activity*/	

				
		/*UI Refresh*/
		if(Msg_Remoter->RC_Data->key.X == 1 && Msg_Remoter->RC_Data->key.CTRL == 1)
		{
		RobotControl->isUIRefresh = 1;
		}
		if(Msg_Remoter->RC_Data->key.X != 1 || Msg_Remoter->RC_Data->key.CTRL != 1)
		{
		RobotControl->isUIRefresh = 0;
		}
		
		
		/*Chasis activity*/
		static uint8_t Lastkey_CV;
		static uint8_t LastKey_Wheel_Zero;
		static uint8_t LastKey_V = 0;
		static uint8_t deltlen=0;
		static uint8_t LastKey_R;
		switch(RobotControl->CheckRobotMode()&0x03)
		{	
			case ROBOTMODE_IDLE:
			{
				
				/*Check if Chasis error happened*/
				if(RobotControl->CheckPartMode(ROBOTPART_ID_BLC)&0x04)
				{
				
				}
					
				/*Not Balance*/
				RobotControl->ChasisControl.SetBalanceFlag(0);
				
				/*Reset open-loop value*/
				RobotControl->ChasisControl.EscapeCloseSpeed[0] = 0.0f;
				RobotControl->ChasisControl.EscapeCloseSpeed[1] = 0.0f;
				
				/*Yaw security*/
				RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw = RobotControl->ChasisControl.ObserveVal.ChasisPsaiYaw;
				

				break;
			}
			case ROBOTMODE_ESCAPE:
			{
				/*Not Balance*/
				RobotControl->ChasisControl.SetBalanceFlag(0);
				RobotControl->ChasisControl.SetChasisMode(ROBOTPART_CHASIS_SHUTTLE);
				
				
				/*Yaw security*/
				RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw = RobotControl->ChasisControl.ObserveVal.ChasisPsaiYaw;
				float EsacpeSpeed[2] = {0};
				
								/*Click E to enable chasis follow gimbal and keep chasis coincidiently follow gimbal*/
					if(Msg_Remoter->RC_Data->key.E)
					{
					RobotControl->SetRobotMode(ROBOTMODE_NORMAL);
					if((RobotControl->CheckPartMode(ROBOTPART_ID_LEG)&0x03)!= ROBOTPART_STATUS_NORMAL)
						{RobotControl->SetPartMode(ROBOTPART_ID_LEG,ROBOTPART_STATUS_START);}
					if((RobotControl->CheckPartMode(ROBOTPART_ID_BLC)&0x03)!= ROBOTPART_STATUS_NORMAL)
						{RobotControl->SetPartMode(ROBOTPART_ID_BLC,ROBOTPART_STATUS_START);}
//					RobotControl->ChasisControl.SetBalanceFlag(1);
//					RobotControl->SetRobotMode(ROBOTMODE_NORMAL);
					}
				/*前后控制*/
				/*This if...else... is designed to make sure remoter has higher priority than keyboard*/
				if(Msg_Remoter->RC_Data->rmt.CH3!=0.0f)
				{	/*遥控器直接赋值即可*/
					EsacpeSpeed[0] =  Msg_Remoter->RC_Data->rmt.CH3*ESCAPE_MAXI_FORWARD;
					EsacpeSpeed[1] = -Msg_Remoter->RC_Data->rmt.CH3*ESCAPE_MAXI_FORWARD;	
				}
				else//鼠标键盘权限
				{	/*鼠键做下积分*/
					static float Tem_KeyV = 0;
					Tem_KeyV = (Msg_Remoter->RC_Data->key.W||Msg_Remoter->RC_Data->key.S) ? Tem_KeyV+0.01f*(Msg_Remoter->RC_Data->key.S - Msg_Remoter->RC_Data->key.W) : 0;
					if(Tem_KeyV>ESCAPE_MAXI_FORWARD){Tem_KeyV = ESCAPE_MAXI_FORWARD;}
					else if(Tem_KeyV<-ESCAPE_MAXI_FORWARD){Tem_KeyV = -ESCAPE_MAXI_FORWARD;}
					
					EsacpeSpeed[0] =  Tem_KeyV;
					EsacpeSpeed[1] = -Tem_KeyV;
				}
				/*旋转控制*/
				if(Msg_Remoter->RC_Data->rmt.CH0!=0.0f)
				{
					
					EsacpeSpeed[0] -= Msg_Remoter->RC_Data->rmt.CH2*ESCAPE_MAXI_ROLL;
					EsacpeSpeed[1] -= Msg_Remoter->RC_Data->rmt.CH2*ESCAPE_MAXI_ROLL;

				}
				else
				{
					/*鼠键做下积分*/
					static float Tem_KeyV = 0;
					Tem_KeyV = (Msg_Remoter->RC_Data->key.A||Msg_Remoter->RC_Data->key.D) ? Tem_KeyV+0.01f*(Msg_Remoter->RC_Data->key.D - Msg_Remoter->RC_Data->key.A) : 0;
					if(Tem_KeyV>ESCAPE_MAXI_ROLL){Tem_KeyV = ESCAPE_MAXI_ROLL;}
					else if(Tem_KeyV<-ESCAPE_MAXI_ROLL){Tem_KeyV = -ESCAPE_MAXI_ROLL;}
					
					if(RobotControl->ChasisControl.GetChasisHead()){
					EsacpeSpeed[0] -= Tem_KeyV;
					EsacpeSpeed[1] -= Tem_KeyV;}
					else{
					EsacpeSpeed[0] += Tem_KeyV;
					EsacpeSpeed[1] += Tem_KeyV;}
					


				}
				RobotControl->ChasisControl.EscapeCloseSpeed[0] =  EsacpeSpeed[0];
				RobotControl->ChasisControl.EscapeCloseSpeed[1] =  EsacpeSpeed[1];			
				break;
			}
			case ROBOTMODE_NORMAL:
			{	
				
				/*Check if Chasis error happened*/
				if(RobotControl->CheckPartMode(ROBOTPART_ID_BLC)&0x04)
				{
					RobotControl->SetRobotMode(ROBOTMODE_IDLE);
					RobotControl->ClearPartError(ROBOTPART_ID_BLC);
				}
				
				/*Get into Chasis StartMode*/
				if(RobotControl->CheckPartMode(ROBOTPART_ID_BLC)==ROBOTPART_STATUS_START)
				{
					/*Wait until banlance is OK*/
					if(RobotControl->ChasisControl.GetBalanceFlag()==2)
					{RobotControl->SetPartMode(ROBOTPART_ID_BLC,ROBOTPART_STATUS_NORMAL);}
				}
				else
				{
					/*SHUTTLE MODE*/
					if(RobotControl->ChasisControl.GetChasisMode()==ROBOTPART_CHASIS_SHUTTLE)
					{
						RobotControl->ChasisControl.SetCFGENStatue(1);
						RobotControl->ChasisControl.SetCFGSStatue(1);
						/*Mode Change Check*/
						if(((Msg_Remoter->RC_Data->rmt.SW2==3)&&fabs(Msg_Remoter->RC_Data->rmt.CH2 >0.9f))||Msg_Remoter->RC_Data->key.Q)
						{
							RobotControl->ChasisControl.SetChasisMode(ROBOTPART_CHASIS_SIDEMODE);
							RobotControl->ChasisControl.ChasisForwardTargetVelocity = 0;
							RobotControl->ChasisControl.RefreshChasisHead();
							RobotControl->ChasisControl.SetCFGENStatue(1);
							RobotControl->ChasisControl.SetCFGSStatue(1);							
							break;
						}
						
						/*Leg length control*/
						if(Msg_Remoter->RC_Data->rmt.SW2==2)/*Booster safty mode can do*/
						{
//							RobotControl->ChasisControl.SetCFGENStatue(1);
//							RobotControl->ChasisControl.SetCFGSStatue(1);
							if((fabs(Msg_Remoter->RC_Data->rmt.WHEEL)> 0.9f)&&LastKey_Wheel_Zero)
							{
								Lastkey_CV = 1;LastKey_Wheel_Zero=0;
								int8_t tmp = RobotControl->ChasisControl.GetLegLenFlag();
								if(Msg_Remoter->RC_Data->rmt.WHEEL>0.9f){tmp += 1;}
								else if(Msg_Remoter->RC_Data->rmt.WHEEL<-0.9f){tmp -= 1;}
								
								if(tmp<LEG_BOTTOM){tmp = LEG_BOTTOM;}
								else if(tmp>LEG_TOP){tmp=LEG_TOP;}

								RobotControl->ChasisControl.SetLegLen((eRobotLegID)tmp);
							}
							

						}
									
					
						

				if(Msg_Remoter->RC_Data->key.CTRL&&Msg_Remoter->RC_Data->key.G)
				{
					RobotControl->ChasisControl.Set_MoveMode(Move_Fly);			
          RobotControl->ChasisControl.SetLegLen(LEG_LEN2);					
				}
				else if(Msg_Remoter->RC_Data->key.CTRL!=1&&Msg_Remoter->RC_Data->key.G)
				{
					RobotControl->ChasisControl.Set_MoveMode(Move_Normal);
					RobotControl->ChasisControl.SetLegLen(LEG_LEN1);
				}
				
				if(RobotControl->ChasisControl.Get_MoveMode()==Move_Fly)  
				{
				  RobotControl->ChasisControl.Set_CheckOG_Value(40.0f);
					RobotControl->ChasisControl.LoopLen[0].Set_Kp(400);
					RobotControl->ChasisControl.LoopLen[0].Set_Kd(2000);
					RobotControl->ChasisControl.LoopLen[1].Set_Kp(400);
					RobotControl->ChasisControl.LoopLen[1].Set_Kd(2000);
					RobotControl->ChasisControl.LoopLen_Dot[0].Set_Kp(1.0f);
					RobotControl->ChasisControl.LoopLen_Dot[1].Set_Kp(1.0f);
				}
				if(RobotControl->ChasisControl.Get_MoveMode()==Move_Normal)  
				{
				  RobotControl->ChasisControl.Set_CheckOG_Value(40.0f);
					RobotControl->ChasisControl.LoopLen[0].Set_Kp(400);
					RobotControl->ChasisControl.LoopLen[0].Set_Kd(800);
					RobotControl->ChasisControl.LoopLen[1].Set_Kp(400);
					RobotControl->ChasisControl.LoopLen[1].Set_Kd(800);
					RobotControl->ChasisControl.LoopLen_Dot[0].Set_Kp(0.5f);
					RobotControl->ChasisControl.LoopLen_Dot[1].Set_Kp(0.5f);
				}
				if(Msg_Remoter->RC_Data->key.V == 1)
				{
					RobotControl->ChasisControl.Set_MoveMode(Move_Jump);
				}
				if(RobotControl->ChasisControl.Get_MoveMode()==Move_Jump)
						{
//							if(Msg_Remoter->RC_Data->rmt.WHEEL>0.9f){
							RobotControl->ChasisControl.SetChasisMode(ROBOTPART_CHASIS_JUMP);
							RobotControl->ChasisControl.SetJumpMode(JUMP_STOP);
							RobotControl->ChasisControl.LastJumpMode=JUMP_STOP;
							RobotControl->ChasisControl.Set_CheckOG_Value(40.0f);
							RobotControl->ChasisControl.Set_JumpFlag(1);
//							}
//							else
//							{
//								RobotControl->ChasisControl.SetChasisMode(ROBOTPART_CHASIS_SHUTTLE);
//							}
//							RobotControl->ChasisControl.SetChasisMode(ROBOTPART_CHASIS_JUMP);
//							RobotControl->ChasisControl.SetJumpMode(JUMP_STOP);
//							RobotControl->ChasisControl.LastJumpMode=JUMP_STOP;
							
						}
						//联盟赛不开小陀螺CV
//						if((Msg_Remoter->RC_Data->key.C&&Msg_Remoter->RC_Data->key.CTRL))
//							{
//								sub_len_flag=1;
//							}
//							else if(Msg_Remoter->RC_Data->key.C&&Msg_Remoter->RC_Data->key.CTRL!=1)
//							{
//							add_len_flag=1;
//							}
//							else
//							{
//								sub_len_flag=0;
//								add_len_flag=0;
//							}
//							if(sub_len_flag)
//							{
//							--deltlen;
//								sub_len_flag=0;
//							}
//							else if(add_len_flag)
//							{
//							++deltlen;
//							add_len_flag=0;
//							}
//						if(deltlen!=0)
//						{
//							if(!Lastkey_CV)
//							{
//								Lastkey_CV = 1;
//								int8_t tmp = RobotControl->ChasisControl.GetLegLenFlag() + deltlen;
//								if(tmp<LEG_BOTTOM){tmp = LEG_BOTTOM;}
//								else if(tmp>LEG_TOP){tmp=LEG_TOP;}
//								RobotControl->ChasisControl.SetLegLen((eRobotLegID)tmp);
//							
//							}
//						}
//						else
//						{Lastkey_CV = 0;}
						if(Msg_Remoter->RC_Data->key.R)
						{
							if(!Lastkey_CV)
							{
								Lastkey_CV = 1;
								int8_t tmp = LEG_TOP;
								if(tmp<LEG_BOTTOM){tmp = LEG_BOTTOM;}
								else if(tmp>LEG_TOP){tmp=LEG_TOP;}
								RobotControl->ChasisControl.SetLegLen((eRobotLegID)tmp);
							}
						}	
          else if(Msg_Remoter->RC_Data->key.CTRL!=1&&Msg_Remoter->RC_Data->key.G)
				   {
					RobotControl->ChasisControl.Set_MoveMode(Move_Normal);
					RobotControl->ChasisControl.SetLegLen(LEG_LEN1);
					Lastkey_CV = 0;
				     }						
						else
						{Lastkey_CV = 0;}
//						LastKey_R = Msg_Remoter->RC_Data->key.R;
						/*前后控制*/
						/*This if...else... is designed to make sure remoter has higher priority than keyboard*/
						if(Msg_Remoter->RC_Data->rmt.CH3!=0.0f)
						{
							RobotControl->ChasisControl.ChasisForwardTargetVelocity =  Msg_Remoter->RC_Data->rmt.CH3*RobotControl->ChasisControl.GetControlMaxVel(0);
						}/*Close-loop control with fix this*/
						else//鼠标键盘权限Low
						{								
							/*鼠键做下积分*/
							static float keyval = 0;
							if(Msg_Remoter->RC_Data->key.W||Msg_Remoter->RC_Data->key.S)
							{
								keyval -= RobotControl->ChasisControl.GetControlMaxAcc(Msg_Remoter->RC_Data->key.SHIFT)*(Msg_Remoter->RC_Data->key.S - Msg_Remoter->RC_Data->key.W);
								
								if(keyval>RobotControl->ChasisControl.GetControlMaxVel(Msg_Remoter->RC_Data->key.SHIFT)){keyval = RobotControl->ChasisControl.GetControlMaxVel(Msg_Remoter->RC_Data->key.SHIFT);}
								else if(keyval<-RobotControl->ChasisControl.GetControlMaxVel(Msg_Remoter->RC_Data->key.SHIFT)){keyval = -RobotControl->ChasisControl.GetControlMaxVel(Msg_Remoter->RC_Data->key.SHIFT);}
								
							}
							else/*Do not emergency brake!*/
							{	
								if(keyval>10*RobotControl->ChasisControl.GetControlMaxAcc(Msg_Remoter->RC_Data->key.SHIFT)){keyval-=RobotControl->ChasisControl.GetControlMaxReAcc(Msg_Remoter->RC_Data->key.SHIFT);}
								else if(keyval<-10*RobotControl->ChasisControl.GetControlMaxAcc(Msg_Remoter->RC_Data->key.SHIFT)){keyval+=RobotControl->ChasisControl.GetControlMaxReAcc(Msg_Remoter->RC_Data->key.SHIFT);}
								else{keyval=0.0f;}
							}
							
							RobotControl->ChasisControl.ChasisForwardTargetVelocity =  keyval;
						}
						                   
						/*Speed up, CFGS Enable*/
						if(fabs(RobotControl->ChasisControl.ChasisForwardTargetVelocity))
						{
							/*Enable chasis follow gimbal*/
							RobotControl->ChasisControl.SetCFGENStatue(1);
							RobotControl->ChasisControl.SetCFGSStatue(1);
						}
							
						/*没有前进后退发生时允许进行旋转*/
						/*change*/
						
							/*Directly add to ref psai*/
							if(Msg_Remoter->RC_Data->rmt.CH2!=0.0f)
							{
								if(Msg_Remoter->RC_Data->rmt.SW2 == 2)
								{
								/*Disbale Chasis follow gimbal*/
									RobotControl->ChasisControl.SetCFGENStatue(0);
	//								RobotControl->ChasisControl.SetCFGENStatue(0);
	//							  RobotControl->ChasisControl.SetCFGSStatue(0);
									RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw -= CHASIS_ANGYLAR_VELOCITY_MAX*Msg_Remoter->RC_Data->rmt.CH2;
								}									
							}
							else
							{
//								RobotControl->ChasisControl.SetCFGENStatue(1);
//							  RobotControl->ChasisControl.SetCFGSStatue(1);
								if(Msg_Remoter->RC_Data->key.A||Msg_Remoter->RC_Data->key.D)
								{
									/*Disbale Chasis follow gimbal*/
									RobotControl->ChasisControl.SetCFGENStatue(0);
									RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw += 2.0f * CHASIS_ANGYLAR_VELOCITY_MAX*(Msg_Remoter->RC_Data->key.A - Msg_Remoter->RC_Data->key.D);
									
								}	
							}		
							if(RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw>7.0f)
							{
								RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw = 7.0f;
							}
							else if(RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw<-7.0f)
							{
								RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw = -7.0f;
							}

					}
					/*SIDE MODE*/
					else if(RobotControl->ChasisControl.GetChasisMode()==ROBOTPART_CHASIS_SIDEMODE)
					{
						if(Msg_Remoter->RC_Data->key.R)
						{
							if(!Lastkey_CV)
							{
								Lastkey_CV = 1;
								int8_t tmp = LEG_TOP;
								if(tmp<LEG_BOTTOM){tmp = LEG_BOTTOM;}
								else if(tmp>LEG_TOP){tmp=LEG_TOP;}
								RobotControl->ChasisControl.SetLegLen((eRobotLegID)tmp);
							}
						}	
          else if(Msg_Remoter->RC_Data->key.CTRL!=1&&Msg_Remoter->RC_Data->key.G)
				   {
					RobotControl->ChasisControl.Set_MoveMode(Move_Normal);
					RobotControl->ChasisControl.SetLegLen(LEG_LEN1);
					Lastkey_CV = 0;
				     }						
						else
						{Lastkey_CV = 0;}
						
						
						/*Mode Change Check*/
						if(Msg_Remoter->RC_Data->key.W||Msg_Remoter->RC_Data->key.S||Msg_Remoter->RC_Data->key.E||(fabs(Msg_Remoter->RC_Data->rmt.CH3)>0.2f))
						{
							RobotControl->ChasisControl.SetChasisMode(ROBOTPART_CHASIS_SHUTTLE);
							RobotControl->ChasisControl.RefreshChasisHead();
							RobotControl->ChasisControl.ChasisForwardTargetVelocity = 0;
							break;
						}
						
						/*Click E to enable chasis follow gimbal and keep chasis coincidiently follow gimbal*/
						if(Msg_Remoter->RC_Data->key.E)
						{RobotControl->ChasisControl.SetChasisMode(ROBOTPART_CHASIS_SHUTTLE);}
						
						/*Leg length control*/
						if(Msg_Remoter->RC_Data->rmt.SW2==2)/*Booster safty mode can do*/
						{
							if((fabs(Msg_Remoter->RC_Data->rmt.WHEEL)> 0.9f)&&LastKey_Wheel_Zero)
							{
								Lastkey_CV = 1;LastKey_Wheel_Zero=0;
								int8_t tmp = RobotControl->ChasisControl.GetLegLenFlag();
								if(Msg_Remoter->RC_Data->rmt.WHEEL>0.9f){tmp += 1;}
								else if(Msg_Remoter->RC_Data->rmt.WHEEL<-0.9f){tmp -= 1;}
								
								if(tmp<LEG_BOTTOM){tmp = LEG_BOTTOM;}
								else if(tmp>LEG_TOP){tmp=LEG_TOP;}

								RobotControl->ChasisControl.SetLegLen((eRobotLegID)tmp);
							}

						}
						//联盟赛不用CV伸腿
//						if((Msg_Remoter->RC_Data->key.C||Msg_Remoter->RC_Data->key.V))
//						{
//							if(!Lastkey_CV)
//							{
//								Lastkey_CV = 1;
//								int8_t tmp = RobotControl->ChasisControl.GetLegLenFlag() + ((Msg_Remoter->RC_Data->key.V - Msg_Remoter->RC_Data->key.C));
//								if(tmp<LEG_BOTTOM){tmp = LEG_BOTTOM;}
//								else if(tmp>LEG_TOP){tmp=LEG_TOP;}
//								RobotControl->ChasisControl.SetLegLen((eRobotLegID)tmp);
//							}
//						}
//						else
//						{Lastkey_CV = 0;}
						
						/*左右控制*/
						/*This if...else... is designed to make sure remoter has higher priority than keyboard*/
						if(Msg_Remoter->RC_Data->rmt.CH2!=0.0f)
						{
							RobotControl->ChasisControl.ChasisForwardTargetVelocity =  -Msg_Remoter->RC_Data->rmt.CH2*RobotControl->ChasisControl.GetControlMaxVel(0);
						}/*Close-loop control with fix this*/
						else//鼠标键盘权限
						{							
							/*鼠键做下积分*/
							static float keyval = 0;
							
							if(Msg_Remoter->RC_Data->key.A||Msg_Remoter->RC_Data->key.D)
							{
								keyval -= RobotControl->ChasisControl.GetControlMaxAcc(Msg_Remoter->RC_Data->key.SHIFT)*(Msg_Remoter->RC_Data->key.D - Msg_Remoter->RC_Data->key.A);
								
								if(keyval>RobotControl->ChasisControl.GetControlMaxVel(Msg_Remoter->RC_Data->key.SHIFT)){keyval = RobotControl->ChasisControl.GetControlMaxVel(Msg_Remoter->RC_Data->key.SHIFT);}
								else if(keyval<-RobotControl->ChasisControl.GetControlMaxVel(Msg_Remoter->RC_Data->key.SHIFT)){keyval = -RobotControl->ChasisControl.GetControlMaxVel(Msg_Remoter->RC_Data->key.SHIFT);}
								
							}
							else/*Do not emergency brake!*/
							{	
								if(keyval>10*RobotControl->ChasisControl.GetControlMaxAcc(Msg_Remoter->RC_Data->key.SHIFT)){keyval-=RobotControl->ChasisControl.GetControlMaxReAcc(Msg_Remoter->RC_Data->key.SHIFT);}
								else if(keyval<-10*RobotControl->ChasisControl.GetControlMaxAcc(Msg_Remoter->RC_Data->key.SHIFT)){keyval+=RobotControl->ChasisControl.GetControlMaxReAcc(Msg_Remoter->RC_Data->key.SHIFT);}
								else{keyval=0.0f;}
							}
							RobotControl->ChasisControl.ChasisForwardTargetVelocity =  keyval;						
						}
						 
						/*侧对F 进入小陀螺 否则返回侧对*/
						if(Msg_Remoter->RC_Data->key.F)
						{
							RobotControl->ChasisControl.SetChasisMode(ROBOTPART_CHASIS_ROLLING);
//							RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw = 0;
//							RobotControl->ChasisControl.ChasisForwardTargetVelocity = 0;
						}
					}
					else if(RobotControl->ChasisControl.GetChasisMode()==ROBOTPART_CHASIS_JUMP)
					{
						float lentmp=RobotControl->ChasisControl.MotorUnits->LinkSolver[0].GetPendulumLen();
//						if(Msg_Remoter->RC_Data->rmt.SW2==2)
//						{RobotControl->ChasisControl.SetChasisMode(ROBOTPART_CHASIS_SHUTTLE);}
						//if(Msg_Remoter->RC_Data->rmt.SW2==1 && RobotControl->ChasisControl.LastJumpMode==JUMP_STOP && LastSW2!=1 && lentmp >0.22f)//0.18
						if(RobotControl->ChasisControl.Get_MoveMode()==Move_Jump && RobotControl->ChasisControl.LastJumpMode==JUMP_STOP&& lentmp >0.20f)//0.18
						{
							RobotControl->ChasisControl.LoopLen[0].Reset();
							RobotControl->ChasisControl.LoopLen[1].Reset();
							RobotControl->ChasisControl.LoopLen_Dot[0].Reset();
							RobotControl->ChasisControl.LoopLen_Dot[1].Reset();
							RobotControl->ChasisControl.LastJumpMode=JUMP_READY;
							RobotControl->ChasisControl.SetLegLen((eRobotLegID)0);
		//					RobotControl->ChasisControl.SetOG(0);
//							RobotControl->ChasisControl.LoopLen[0].Set_Kp(400);
//							RobotControl->ChasisControl.LoopLen[0].Set_Kd(2000);
//							RobotControl->ChasisControl.LoopLen[1].Set_Kp(400);
//							RobotControl->ChasisControl.LoopLen[1].Set_Kd(2000);
					RobotControl->ChasisControl.LoopLen_Dot[0].Set_Kp(0.1f);
					RobotControl->ChasisControl.LoopLen_Dot[1].Set_Kp(0.1f);
						}
						//else if(Msg_Remoter->RC_Data->rmt.SW2==1 && RobotControl->ChasisControl.LastJumpMode==JUMP_READY && lentmp <=0.19f)//0.18
						else if(RobotControl->ChasisControl.Get_MoveMode()==Move_Jump && RobotControl->ChasisControl.LastJumpMode==JUMP_READY && lentmp <=0.20f)//0.18
						{
							RobotControl->ChasisControl.LastJumpMode=JUMP_STRETCH;
							RobotControl->ChasisControl.SetLegLen((eRobotLegID)6);
	//						RobotControl->ChasisControl.SetOG(2);
							RobotControl->ChasisControl.LoopLen[0].Reset();
							RobotControl->ChasisControl.LoopLen[1].Reset();
							RobotControl->ChasisControl.LoopLen_Dot[0].Reset();
							RobotControl->ChasisControl.LoopLen_Dot[1].Reset();
							RobotControl->ChasisControl.LoopLen[0].Set_Kp(1800);
							RobotControl->ChasisControl.LoopLen[0].Set_Kd(2000);
							RobotControl->ChasisControl.LoopLen[1].Set_Kp(1800);
							RobotControl->ChasisControl.LoopLen[1].Set_Kd(2000);
					RobotControl->ChasisControl.LoopLen_Dot[0].Set_Kp(0.1f);
					RobotControl->ChasisControl.LoopLen_Dot[1].Set_Kp(0.1f);
						}
						else if(RobotControl->ChasisControl.Get_MoveMode()==Move_Jump&& RobotControl->ChasisControl.LastJumpMode==JUMP_STRETCH && lentmp>0.26f)//0.36
						//else if(RobotControl->ChasisControl.Get_JumpFlag()==1 && RobotControl->ChasisControl.LastJumpMode==JUMP_STRETCH && lentmp>0.26f)//0.36
						{
							RobotControl->ChasisControl.LastJumpMode=JUMP_SHRINK;
							RobotControl->ChasisControl.SetLegLen((eRobotLegID)0);
							RobotControl->ChasisControl.LoopLen[0].Reset();
							RobotControl->ChasisControl.LoopLen[1].Reset();
							RobotControl->ChasisControl.LoopLen_Dot[0].Reset();
							RobotControl->ChasisControl.LoopLen_Dot[1].Reset();
							RobotControl->ChasisControl.LoopLen[0].Set_Kp(1100);
							RobotControl->ChasisControl.LoopLen[0].Set_Kd(2000);
							RobotControl->ChasisControl.LoopLen[1].Set_Kp(1100);
							RobotControl->ChasisControl.LoopLen[1].Set_Kd(2000);
					RobotControl->ChasisControl.LoopLen_Dot[0].Set_Kp(1.0f);
					RobotControl->ChasisControl.LoopLen_Dot[1].Set_Kp(1.0f);
//							RobotControl->ChasisControl.SetOG(1);
						}
//						else if(Msg_Remoter->RC_Data->rmt.SW2==1 && RobotControl->ChasisControl.LastJumpMode==JUMP_SHRINK && lentmp<0.18f)//0.18
						else if(RobotControl->ChasisControl.Get_MoveMode()==Move_Jump && RobotControl->ChasisControl.LastJumpMode==JUMP_SHRINK && lentmp<0.18f)//0.18
						{
							RobotControl->ChasisControl.LastJumpMode=JUMP_STOP;
							RobotControl->ChasisControl.SetLegLen((eRobotLegID)3);
							RobotControl->ChasisControl.LoopLen[0].Reset();
							RobotControl->ChasisControl.LoopLen[1].Reset();
							RobotControl->ChasisControl.LoopLen_Dot[0].Reset();
							RobotControl->ChasisControl.LoopLen_Dot[1].Reset();
							RobotControl->ChasisControl.LoopLen[0].Set_Kp(400);
							RobotControl->ChasisControl.LoopLen[0].Set_Kd(1200);
							RobotControl->ChasisControl.LoopLen[1].Set_Kp(400);
							RobotControl->ChasisControl.LoopLen[1].Set_Kd(1200);
							//RobotControl->ChasisControl.SetOG(1);
					RobotControl->ChasisControl.LoopLen_Dot[0].Set_Kp(1.0f);
					RobotControl->ChasisControl.LoopLen_Dot[1].Set_Kp(1.0f);
						}
			//			else if(Msg_Remoter->RC_Data->rmt.SW2==1 && RobotControl->ChasisControl.LastJumpMode==JUMP_STOP && lentmp<0.25f && RobotControl->ChasisControl.CheckLand())
							else if(RobotControl->ChasisControl.Get_MoveMode()==Move_Jump&& RobotControl->ChasisControl.LastJumpMode==JUMP_STOP && lentmp<0.29f && RobotControl->ChasisControl.CheckLand())
						{
							RobotControl->ChasisControl.LastJumpMode=JUMP_LANDING;
							RobotControl->ChasisControl.SetLegLen((eRobotLegID)2);
							RobotControl->ChasisControl.SetChasisMode(ROBOTPART_CHASIS_SHUTTLE);
							RobotControl->ChasisControl.LoopLen[0].Reset();
							RobotControl->ChasisControl.LoopLen[1].Reset();
							RobotControl->ChasisControl.LoopLen_Dot[0].Reset();
							RobotControl->ChasisControl.LoopLen_Dot[1].Reset();
							RobotControl->ChasisControl.LoopLen[0].Set_Kp(400);
							RobotControl->ChasisControl.LoopLen[0].Set_Kd(1200);
							RobotControl->ChasisControl.LoopLen[1].Set_Kp(400);
							RobotControl->ChasisControl.LoopLen[1].Set_Kd(1200);
							//RobotControl->ChasisControl.SetOG(0);
					RobotControl->ChasisControl.LoopLen_Dot[0].Set_Kp(1.0f);
					RobotControl->ChasisControl.LoopLen_Dot[1].Set_Kp(1.0f);
							RobotControl->ChasisControl.Set_JumpFlag(0);
							RobotControl->ChasisControl.Set_MoveMode(Move_Normal);
						}
						LastSW2=Msg_Remoter->RC_Data->rmt.SW2;
				    LastKey_V =Msg_Remoter->RC_Data->key.V;
						jump_count=0;
					}
					else if(RobotControl->ChasisControl.GetChasisMode()==ROBOTPART_CHASIS_ROLLING)
					{
						/* 不按F 进入侧对模式  */
						if(Msg_Remoter->RC_Data->key.F)
						{
										/*Disbale Chasis follow gimbal*/
							RobotControl->ChasisControl.SetCFGENStatue(0);
							if(RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw>0.0f)
							{
								RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw += CHASIS_ANGYLAR_VELOCITY_MAX*(Msg_Remoter->RC_Data->key.F);
							}
							else
							{
								RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw -= CHASIS_ANGYLAR_VELOCITY_MAX*(Msg_Remoter->RC_Data->key.F);
							}
							//转速最大值
							if(RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw>7.0f)
							{
								RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw = 7.0f;
							}
							else if(RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw<-7.0f)
							{
								RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw = -7.0f;
							}		
						}	
						else
						{
							RobotControl->ChasisControl.SetChasisMode(ROBOTPART_CHASIS_SIDEMODE);
							RobotControl->ChasisControl.ChasisForwardTargetVelocity = 0;
							RobotControl->ChasisControl.RefreshChasisHead();
							RobotControl->ChasisControl.SetCFGENStatue(1);
							RobotControl->ChasisControl.SetCFGSStatue(1);	
						}
					}
				}
				break;
			}
		}
		if(fabs(Msg_Remoter->RC_Data->rmt.WHEEL)<0.1f)
		{LastKey_Wheel_Zero = 1;}
			
/********** Gimbal activity **********/
		switch(RobotControl->CheckRobotMode()&0x03)
		{
			case ROBOTMODE_IDLE:
				RobotControl->ComDown.YawInc		= 0;
				RobotControl->ComDown.PitInc		= 0;
				RobotControl->ComDown.GimDeathFlag	= 1;
				
				/*Error in IDLE mode*/
				if(RobotControl->CheckPartMode(ROBOTPART_ID_GIM)&0x04)
				{
				
				}
			break;
			
			/*Gimbal has same activity when robot be in Escape and Normal mode*/
			case ROBOTMODE_ESCAPE:
			case ROBOTMODE_NORMAL:	
				/*Check if Gimbal error happened*/
				if(RobotControl->CheckPartMode(ROBOTPART_ID_GIM)&0x04)
				{
					/*Change Normal into Start*/
					if(RobotControl->CheckPartMode(ROBOTPART_ID_GIM)==ROBOTPART_STATUS_NORMAL)
					{RobotControl->SetPartMode(ROBOTPART_ID_GIM,ROBOTPART_STATUS_START);}
					
					/*Set GimDeathFlag*/
					RobotControl->ComDown.GimDeathFlag = 1;
				}
				
				/*Get into Gimbal StartMode*/
				if(RobotControl->CheckPartMode(ROBOTPART_ID_GIM)==ROBOTPART_STATUS_START)
				{
					/*Return to normal mode. Robot is alive and no Gimbal error*/
					if( RobotControl->GetRobotHP() && !(RobotControl->CheckPartMode(ROBOTPART_ID_GIM)&0x04)  )
					{
						RobotControl->SetPartMode(ROBOTPART_ID_GIM,ROBOTPART_STATUS_NORMAL);
						RobotControl->ComDown.GimDeathFlag = 0;
					}
				}
				/*Get into Gimbal NormalMode*/
				else if(RobotControl->CheckPartMode(ROBOTPART_ID_GIM)==ROBOTPART_STATUS_NORMAL)
				{	
					/*Gimbal normal activity here*/					
					/*Yaw and Pithc radian increase*/
					RobotControl->ComDown.PitInc =	-GIMBAL_PITCH_ANGYLAR_VELOCITY	* (Msg_Remoter->RC_Data->rmt.CH1*2 - (float)(Msg_Remoter->RC_Data->mouse.y)/150.0f );//Pitch need to reverse
					RobotControl->ComDown.YawInc =	-GIMBAL_YAW_ANGYLAR_VELOCITY 	* (Msg_Remoter->RC_Data->rmt.CH0*2 + (float)(Msg_Remoter->RC_Data->mouse.x)/150.0f );//Pitch need to reverse
				}		
		}
		
/********** Booster activity **********/
		switch(RobotControl->CheckRobotMode()&0x03)
		{
			
			case ROBOTMODE_IDLE:
//				RobotControl->ComDown.Fire			= 0;
				RobotControl->ComDown.BoosterMode	= ROBOTPART_BOOSTER_STOP;
				RobotControl->ComDown.TriggleMode	= ROBOTPART_TRIGGLE_STOP;
			
				/*Error in IDLE mode*/
				if(RobotControl->CheckPartMode(ROBOTPART_ID_BST)&0x04)
				{
				
				}
			break;
			
			case ROBOTMODE_ESCAPE:
			case ROBOTMODE_NORMAL:
				
				/*Check if Booster error happened*/
				if(RobotControl->CheckPartMode(ROBOTPART_ID_BST)&0x04)
				{
					/*Jam is not a very important error*/
					
				
				}
				
				/*Update real-time paramaters*/
				
				/*Update Troggle mode*/
				if(RobotControl->GetRobotCoolRate()<ROBOTPART_TROOGLEMODE_VALUE)
				{RobotPart_TroggleMMode = ROBOTPART_TRIGGLE_MMODE0;}
				else
				{RobotPart_TroggleMMode = ROBOTPART_TRIGGLE_MMODE1;}
				
				/*Update Booster Velocity*/
//				switch(RobotControl->GetRobotAmmoSpeed())
				
				/*Control Cap*/
				
				static float LastKey_Wheel;
				/*KeyBoard Control*/
//				if( (Msg_Remoter->RC_Data->key.R)&&(!LastKey_R) )
//				{RobotControl->ComDown.CapOpen = RobotControl->ComDown.CapOpen?0:1;}
//				/*Remoter control. Push Timewise*/
//				else if((Msg_Remoter->RC_Data->rmt.WHEEL<-0.5f)&&(LastKey_Wheel>-0.5f))
//				{RobotControl->ComDown.CapOpen = RobotControl->ComDown.CapOpen?0:1;}					
				/*Close Cap when mode change*/
				if(IsModeChangeValid)
				{RobotControl->ComDown.CapOpen = 0;}	
				
				
				LastKey_Wheel = Msg_Remoter->RC_Data->rmt.WHEEL;
				
				/*Get into Ammobooster StartMode*/
				if(RobotControl->CheckPartMode(ROBOTPART_ID_BST)==ROBOTPART_STATUS_START)
				{
					/*Return to normal mode. Robot is alive and no Booster error*/
					if( RobotControl->GetRobotHP() && !(RobotControl->CheckPartMode(ROBOTPART_ID_BST)&0x04)  )
					{
						RobotControl->ComDown.TriggleMode	= RobotPart_TroggleMMode;
						RobotControl->ComDown.BoosterMode	= RobotPart_BoosterMode;
						RobotControl->SetPartMode(ROBOTPART_ID_BST,ROBOTPART_STATUS_NORMAL);
					}
					else
					{
//						RobotControl->ComDown.Fire			= 0;
						RobotControl->ComDown.BoosterMode	= ROBOTPART_BOOSTER_STOP;
						RobotControl->ComDown.TriggleMode	= ROBOTPART_TRIGGLE_STOP;
					}
				}	
				/*Get into Ammobooster NormalMode*/
				else if(RobotControl->CheckPartMode(ROBOTPART_ID_BST)==ROBOTPART_STATUS_NORMAL)
				{	
					/*Boooster normal activity here*/
					
					static uint8_t LastKey_B = 0;
					static uint8_t LastKey_SW2 = 1;
          
					
				if(RobotControl->GetFireFlag())
				{
					RobotControl->rc_data_3.gimbal_fire_flag = 1;
				}
				else
				{
					RobotControl->rc_data_3.gimbal_fire_flag = 0;
				}
					/* When SW2->1, safy lock enable, robot booster disable and can not open fire. */
//					if(Msg_Remoter->RC_Data->rmt.SW2==1)
//					{
//						RobotControl->ComDown.Fire			= 0;
//						RobotControl->ComDown.BoosterMode	= ROBOTPART_BOOSTER_STOP;
//						RobotControl->ComDown.TriggleMode	= ROBOTPART_TRIGGLE_STOP;
//					}
//					else
//					{
//						RobotControl->ComDown.BoosterMode	= RobotPart_BoosterMode;				
						/*Singel-Shot or Multi-Shot mode change*/
						/*switch change*/
//						if(Msg_Remoter->RC_Data->rmt.SW2!=LastKey_SW2)
//						{
//							RobotControl->ComDown.TriggleMode = (Msg_Remoter->RC_Data->rmt.SW2==2)?ROBOTPART_TRIGGLE_SINGLE:RobotPart_TroggleMMode;
//						}
//						/*keyboard change*/
//						if( (Msg_Remoter->RC_Data->key.B) && (!LastKey_B) )
//						{
//							RobotControl->ComDown.TriggleMode = (RobotControl->ComDown.TriggleMode	== RobotPart_TroggleMMode)?ROBOTPART_TRIGGLE_SINGLE:RobotPart_TroggleMMode;
//						}
						
						/*Fire Flag Set*/
						/*Check is threr enough heat remain*/					

//						else
//						{
//							/*OpenFire Control. Singel-Shot or Multi-Shot should be processed in gimbal-control-board*/						
//							/*Mouse control*/
//							RobotControl->ComDown.Fire = Msg_Remoter->RC_Data->mouse.press_l;
//							/*Remoter control. Push Anti-Timewise*/
//							if(Msg_Remoter->RC_Data->rmt.WHEEL>0.5f)
//							{RobotControl->ComDown.Fire = 1;}								
//						}
//					}
					LastKey_B = Msg_Remoter->RC_Data->key.B;
					LastKey_SW2 = Msg_Remoter->RC_Data->rmt.SW2;
					 LastKey_V =Msg_Remoter->RC_Data->key.V;
				}	
		}

	}
}
