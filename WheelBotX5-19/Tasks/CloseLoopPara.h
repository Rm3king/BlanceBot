#ifndef	CLOSELOOPPARA_H
#define	CLOSELOOPPARA_H
#ifdef __cplusplus
#include "ControlTask.h"

#define GACCEL	9.8011f
#define MWHEEL	2.824

#define LEGMAX 0.400f
#define LEGMID 0.160f
#define LEGMIN 0.150f

#define LQRRESOLUTION 0.010f
#define LQRKNUM 100

/*Paramaters of accelerater and max velocity*/
enum eRobotLegID
{
	LEG_BOTTOM=0,
	LEG_LEN0=1,
	LEG_LEN1=2,	
	LEG_LEN2=3,
	LEG_LEN3=4,
	LEG_LEN4=5,
	LEG_TOP=6,	
};


/*关节长度 加速度 减速度 最大速度 Boost模式系数 Side模式系数*/
//static float LegMovingPara[LEG_TOP+1][6]=
//{
//	{0.180f, 0.010f, 0.015f, 1.0f, 1.2f, 1.0f},
//	{0.220f, 0.011f, 0.020f, 2.0f, 1.5f, 0.8f},
//	{0.250f, 0.012f, 0.018f, 1.5f, 2.0f, 0.8f},
//	{0.280f, 0.010f, 0.015f, 1.0f, 1.0f, 0.7f},
//};
static float LegMovingPara[LEG_TOP+1][6]=
{
	{0.140f, 0.010f, 0.015f, 1.0f, 1.2f, 1.0f},
	{0.180f, 0.010f, 0.015f, 1.5f, 1.3f, 0.8f},
	{0.210f, 0.010f, 0.018f, 1.5f, 1.67f, 0.8f},
	{0.240f, 0.010f, 0.015f, 1.5f, 1.67f, 0.7f},//2.5f
	{0.300f, 0.010f, 0.015f, 1.0f, 1.0f, 0.7f},
	{0.340f, 0.010f, 0.015f, 1.0f, 1.0f, 0.7f},
	{0.390f, 0.010f, 0.015f, 1.0f, 1.0f, 0.7f},
};


/*行进控制*/
class cLQR
{		
	protected:
	/*LQR -K matrix*/
	/*2N means normal LQR*/
	/*2N+1 means off-ground LQR*/
	float LQRKbuf[12]={0};
	float LQROutBuf[2]={0};
	float LQRXerrorBuf[6]={0};
	
	arm_matrix_instance_f32 *LQRXRefX;
	arm_matrix_instance_f32 *LQRXObsX;
	
	
	arm_matrix_instance_f32 MatLQRNegK = {2, 6, LQRKbuf};
	arm_matrix_instance_f32 MatLQRErrX = {6, 1, LQRXerrorBuf};
	arm_matrix_instance_f32 MatLQROutU = {2, 1, LQROutBuf};
	
	
	
	public:
		
	/*Calculate X. Output is u (T,Tp)`*/
	void LQRCal(float* Tout)
	{
		//Calculate error
		arm_mat_sub_f32(this->LQRXObsX,this->LQRXRefX,&this->MatLQRErrX);
		//Calculate output value
		arm_mat_mult_f32(&this->MatLQRNegK,&this->MatLQRErrX,&this->MatLQROutU);
		//return Value
		Tout[0] = this->LQROutBuf[0];
		Tout[1] = this->LQROutBuf[1];
	}

	/* 	
		SetLQR -K paramaters
		LegLenth : Really length of leg
		isFly	 : Is robot in the sky, should compare with offground detection
	*/
	void RefreshLQRK(float LegLenth, uint8_t isFly)
	{
		LegLenth = (LegLenth<LEGMIN)?LEGMIN:LegLenth;
		LegLenth = (LegLenth>LEGMAX)?LEGMAX:LegLenth;
//		volatile uint8_t ID = roundf((LegLenth - LEGMIN)/LQRRESOLUTION);
//		this->MatLQRNegK.pData = (float*)LQRKbuf[4*ID+isFly];
    float L_3 = LegLenth*LegLenth*LegLenth;
		float L_2 = LegLenth*LegLenth;
		float L_1 = LegLenth;
    float LQR_para[12];
	LQR_para[0]=137.787334*L_3-171.289780*L_2+87.612008*L_1+25.233411;
	LQR_para[1]=15.669252*L_3-17.835430*L_2+8.941497*L_1+5.984353;
	LQR_para[2]=14.145266*L_3-17.796434*L_2+8.522327*L_1+6.016545;
	LQR_para[3]=-2.385954*L_3+1.519393*L_2+0.041331*L_1+10.413518;
	LQR_para[4]=98.680447*L_3-139.831385*L_2+83.181625*L_1-27.067160;
	LQR_para[5]=43.950969*L_3-56.493362*L_2+29.709131*L_1-8.682802;

	LQR_para[6]=7.051266*L_3-39.242731*L_2+43.724841*L_1-21.994519;
	LQR_para[7]=26.384862*L_3-36.844074*L_2+21.323659*L_1-6.609568;
	LQR_para[8]=14.950743*L_3-25.091986*L_2+17.568155*L_1-6.559303;
	LQR_para[9]=39.177993*L_3-53.778858*L_2+30.642922*L_1-9.328690;
	LQR_para[10]=-132.643905*L_3+158.309641*L_2-71.010249*L_1-27.733899;
	LQR_para[11]=-44.492204*L_3+52.047003*L_2-22.795018*L_1-2.225013;
		if(isFly == 0)
		{
			this->MatLQRNegK.pData[0] =  LQR_para[0];this->MatLQRNegK.pData[1] =  LQR_para[1];this->MatLQRNegK.pData[2] =  LQR_para[2];this->MatLQRNegK.pData[3] =  LQR_para[3];
			this->MatLQRNegK.pData[4] =  LQR_para[4];this->MatLQRNegK.pData[5] =  LQR_para[5];this->MatLQRNegK.pData[6] =  LQR_para[6];this->MatLQRNegK.pData[7] =  LQR_para[7];
			this->MatLQRNegK.pData[8] =  LQR_para[8];this->MatLQRNegK.pData[9] =  LQR_para[9];this->MatLQRNegK.pData[10] =  LQR_para[10];this->MatLQRNegK.pData[11] =  LQR_para[11];
		}
		else if(isFly == 1)
		{
			this->MatLQRNegK.pData[0] =  0;this->MatLQRNegK.pData[1] =  0;this->MatLQRNegK.pData[2] =  0;this->MatLQRNegK.pData[3] =  0;
			this->MatLQRNegK.pData[4] =  0;this->MatLQRNegK.pData[5] =  0;this->MatLQRNegK.pData[6] =  LQR_para[6];this->MatLQRNegK.pData[7] =  LQR_para[7];
			this->MatLQRNegK.pData[8] =  0;this->MatLQRNegK.pData[9] =  0;this->MatLQRNegK.pData[10] =  0;this->MatLQRNegK.pData[11] =  0;			
		}
		else if(isFly == 2)
		{
			this->MatLQRNegK.pData[0] =  0;this->MatLQRNegK.pData[1] =  0;this->MatLQRNegK.pData[2] =  0;this->MatLQRNegK.pData[3] =  0;
			this->MatLQRNegK.pData[4] =  0;this->MatLQRNegK.pData[5] =  0;this->MatLQRNegK.pData[6] =  LQR_para[6];this->MatLQRNegK.pData[7] =  LQR_para[7];
			this->MatLQRNegK.pData[8] =  0;this->MatLQRNegK.pData[9] =  0;this->MatLQRNegK.pData[10] =  LQR_para[10];this->MatLQRNegK.pData[11] =  LQR_para[11];				
		}
	}
	
	/* Set LQR Error Variate*/
	void InitMatX(arm_matrix_instance_f32* pMatXRef, arm_matrix_instance_f32* pMatXObs)
	{
		this->LQRXRefX = pMatXRef;
		this->LQRXObsX = pMatXObs;
	}
};

/*航向角追踪*/
//class cLoopYaw : public cPIDPla
//{
//	public:

//	cLoopYaw(void)
//	{PID_Init();}
//	
//	void PID_Init(void)
//	{
//		this->Fs = 0.0f;
//		this->Kp = 2;//3.0f;
//		this->Ki = 0;
//		this->Kd = 20;//400.0f;
//		this->Kf = 0.0f;
//		this->IN_RANGE_EN_D = 0;// Pi/3
//		this->IN_RANGE_EN_I = 0;
//		this->MaxOutValue = 5;
//		this->MinOutValue = -5;
//		this->Maxintegral = 0;
//		this->Minintegral = 0;
//	}
//};
/* 航向角跟踪 */
class cLoopYaw : public cPIDPla
{
	public:

	cLoopYaw(void)
	{PID_Init();}
	
	void PID_Init(void)
	{
		this->Fs = 0.0f;
		this->Kp = 20;//3.0f;
		this->Ki = 0;
		this->Kd = 50;//400.0f;
		this->Kf = 0.0f;
		this->IN_RANGE_EN_D = 0;// Pi/3
		this->IN_RANGE_EN_I = 0;
		this->MaxOutValue = 10;
		this->MinOutValue = -10;
		this->Maxintegral = 0;
		this->Minintegral = 0;
	}
};

/* 小陀螺 */
class cLoopGyro : public cPIDPla
{
  public:
		cLoopGyro(void)
		{PID_Init();}
	
	void PID_Init(void)
	{
				this->Fs = 0.0f;
		this->Kp = 3.0f;//3.0f;
		this->Ki = 0;
		this->Kd = 0;//400.0f;
		this->Kf = 0.0f;
		this->IN_RANGE_EN_D = 0;// Pi/3
		this->IN_RANGE_EN_I = 0;
		this->MaxOutValue = 6;
		this->MinOutValue = -6;
		this->Maxintegral = 0;
		this->Minintegral = 0;
	}
};
/*倒立摆长度*/
class cLoopLen : public cPIDPla
{
	public:
	cLoopLen(void)
	{PID_Init();}
	
	void PID_Init(void)
	{
		this->Fs = 0.0f;
		this->Kp = 400;//800//400.
		this->Ki = 1.0f;
		this->Kd = 1200;//500.0f;//-24;//10
		this->Kf = 0.0f;
		this->IN_RANGE_EN_D = 0;// Pi/3
		this->IN_RANGE_EN_I = 0;
		this->MaxOutValue = 10000.0f;
		this->MinOutValue = -10000.0f;
		this->Maxintegral = 5.0f;
		this->Minintegral = -5.0f;
	}
};
class cLoopLen_Dot : public cPIDPla
{
	public:
	cLoopLen_Dot(void)
	{PID_Init();}
	
	void PID_Init(void)
	{
		this->Fs = 0.0f;
		this->Kp = 1.0f;//飞坡1.0
		this->Ki = 0.0f;
		this->Kd = 0;//500.0f;//-24;//10
		this->Kf = 0.0f;
		this->IN_RANGE_EN_D = 0;// Pi/3
		this->IN_RANGE_EN_I = 0;
		this->MaxOutValue = 10000.0f;
		this->MinOutValue = -10000.0f;
		this->Maxintegral = 5.0f;
		this->Minintegral = -5.0f;
	}
};
class cLoopLen_Spf : public cPIDSpf
{
	public:
	cLoopLen_Spf(void)
	{
		PID_Init();
	}
	void PID_Init(void)
	{
		this->Kp = 300;//800//400.
		this->Ki = 0.0f;
		this->Kd = 45;//500.0f;//-24;//10
		this->MaxOutValue = 10000.0f;
		this->MinOutValue = -10000.0f;
		this->Maxintegral = 5.0f;
		this->Minintegral = -5.0f;
	}
};

class cLooplen_Ready:public cPIDPla
{
	public:
	cLooplen_Ready(void)
	{PID_Init();}
	
	void PID_Init(void)
	{
		this->Fs = 0.0f;
		this->Kp = 400;//800//400.
		this->Ki = 1.0f;
		this->Kd = 500;//500.0f;//-24;//10
		this->Kf = 0.0f;
		this->IN_RANGE_EN_D = 0;// Pi/3
		this->IN_RANGE_EN_I = 0;
		this->MaxOutValue = 10000.0f;
		this->MinOutValue = -10000.0f;
		this->Maxintegral = 5.0f;
		this->Minintegral = -5.0f;
	}
};
class cLooplen_Stop:public cPIDPla
{
	public:
	cLooplen_Stop(void)
	{PID_Init();}
	
	void PID_Init(void)
	{
		this->Fs = 0.0f;
		this->Kp = 400;//800//400.
		this->Ki = 1.0f;
		this->Kd = 500;//500.0f;//-24;//10
		this->Kf = 0.0f;
		this->IN_RANGE_EN_D = 0;// Pi/3
		this->IN_RANGE_EN_I = 0;
		this->MaxOutValue = 10000.0f;
		this->MinOutValue = -10000.0f;
		this->Maxintegral = 5.0f;
		this->Minintegral = -5.0f;
	}
};	
/*斜坡平稳*/
class cLoopRoll : public cPIDPla
{
	public:
	cLoopRoll(void)
	{PID_Init();}
	
	void PID_Init(void)
	{
		this->Fs = 0.0f;
		this->Kp = 0.8f;
		this->Ki = 0;//0.001f;
		this->Kd = 10.0f;//0.100f;
		this->Kf = 0.0f;
		this->IN_RANGE_EN_D = 0;// Pi/3
		this->IN_RANGE_EN_I = 0;
		this->MaxOutValue = 0.05;
		this->MinOutValue = -0.05;
		this->Maxintegral = 50;
		this->Minintegral = -50;
	}
};

/*底盘跟随云台*/
class cLoopCFG : public cPIDPla
{
	public:
	cLoopCFG(void)
	{PID_Init();}
	
	void PID_Init(void)
	{
		this->Fs = 0.0f;
		this->Kp = 20.0f;//0.05f;
		this->Ki = 0;
		this->Kd = 5.0f;//2.0f;
		this->Kf = 0.0f;
		this->IN_RANGE_EN_D = 0;// Pi/3
		this->IN_RANGE_EN_I = 0;
		this->MaxOutValue = 5.0f;//0.005
		this->MinOutValue = -5.0f;
		this->Maxintegral = 0;
		this->Minintegral = 0;
	}
};

/*避免劈叉*/
class cLoopTheta : public cPIDPla
{
	public:
	cLoopTheta(void)
	{PID_Init();}
	
	float MAXTHETA=0.0f;
	
	void PID_Init(void)
	{
		this->Fs = 0.0f;
		this->Kp = 75;//120.0f;//100.0f;
		this->Ki = 0;
		this->Kd = 600;//500.0f;//400;
		this->Kf = 0.0f;
		this->IN_RANGE_EN_D = 0;// Pi/3
		this->IN_RANGE_EN_I = 0;
		this->MaxOutValue = 200;
		this->MinOutValue = -200;
		this->Maxintegral = 0;
		this->Minintegral = 0;
	}
};
class cLoopTheta_Spf : public cPIDSpf
{
	public:
	cLoopTheta_Spf(void)
	{PID_Init();} 
	float MAXTHETA=0.0f;
	
	void PID_Init(void)
	{
	 this->Kp = 75;
	 this->Ki = 0;
	 this->Kd = 5;
   this->MaxOutValue = 100;
   this->MinOutValue = -100;
   this->Maxintegral = 0;
   this->Minintegral = 0;	 
	}
};
/*避免劈叉*/
class cLoopEscape : public cPIDPla
{
	public:
	cLoopEscape(void)
	{PID_Init();}

	
	void PID_Init(void)
	{
		this->Fs = 0.0f;
		this->Kp = 5.0f;// 12.0f;//100.0f;
		this->Ki = 0;//1.0f;
		this->Kd = 0;//1.0f;//400;
		this->Kf = 0.0f;
		this->IN_RANGE_EN_D = 0;// Pi/3
		this->IN_RANGE_EN_I = 0;
		this->MaxOutValue = 2000;
		this->MinOutValue = -2000;
		this->Maxintegral = 0;
		this->Minintegral = 0;
	}
};

/* 抵御外界扰动 */
class cLoopRollOffset : public cPIDPla
{
	public:
	cLoopRollOffset(void)
	{PID_Init();}	
	void PID_Init(void)
	{
		this->Fs = 0.0f;
		this->Kp = 300.0f;// 12.0f;//100.0f;
		this->Ki = 0;//1.0f;
		this->Kd = 0;//1.0f;//400;
		this->Kf = 0.0f;
		this->IN_RANGE_EN_D = 0;// Pi/3
		this->IN_RANGE_EN_I = 0;
		this->MaxOutValue = 100;
		this->MinOutValue = -100;
		this->Maxintegral = 0;
		this->Minintegral = 0;
	}	
};

#define t  0.002f
#define t2 0.000001f
#define t3 0.000000001f
#define t4 0.000000000001f
#define t5 0.000000000000001f

class cVelFusionKF
{
	protected:
	const float qq=5.0f;//10
	const float rv=0.1f;
	const float ra=60.0f;//25.0f
		
	const float A_Init[9]={1,t,t2/2,0,1,t,0,0,1};
	const float Q_Init[9]={t5/20*qq,t4/8*qq,t3/6*qq,t4/8*qq,t3/3*qq,t2/2*qq,t3/6*qq,t2/2*qq,t*qq}; 
	const float H_Init[6]={0,1,0,0,0,1};
	const float P_Init[9]={10,0,0,0,10,0,0,0,10};
	const float R_Init[4]={rv,0,0,ra};
	
	public:
	VelFusionKF_t KF;
	cVelFusionKF()
	{
		VelFusionKF_Init(&this->KF,3,2);//Inertia odome 3 State 2 observation
		memcpy(this->KF.P_data, P_Init, sizeof(P_Init));
		memcpy(this->KF.A_data, A_Init, sizeof(A_Init));
		memcpy(this->KF.Q_data, Q_Init, sizeof(Q_Init));
		memcpy(this->KF.H_data, H_Init, sizeof(H_Init));
		memcpy(this->KF.R_data, R_Init, sizeof(R_Init));
	}
	void ResetKF(void)
	{VelFusionKF_Reset(&this->KF);}
	void UpdateKalman(float Velocity, float AccelerationX)
	{
		this->KF.MeasuredVector[0] = Velocity;
		this->KF.MeasuredVector[1] = AccelerationX;
		VelFusionKF_Update(&this->KF);
	}
	
};
		
#endif
#endif
