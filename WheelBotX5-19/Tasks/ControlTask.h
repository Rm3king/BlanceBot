/*
	2023/7/8
	Update Message package
	Benefit for wheelbotlite
*/
#ifndef	CONTROLTASK_H
#define	CONTROLTASK_H
#ifdef __cplusplus
#include "main.h"
#include "arm_math.h"
#include "PID.h"
#include "QCSLite.h"
#include "KF_VelFusion.h"
#include "CloseLoopPara.h"

#include "LinkSolver.h"
#include "RemoterTask.h"
#include "MotorTask.h"
#include "IMUTask.h"
#include "ReferDriver.h"



#define RADMAX  2.6f  //PI/3
#define RADMIN  0.52f
#define RADINIT 1.570796f // PI/2

#define RADCOR 0.068f

/*Torque*/
#define ESCAPE_MAXI_FORWARD 2
#define ESCAPE_MAXI_ROLL 2		

#define GIMBAL_PITCH_MAX
#define GIMBAL_YAW_MIN


#define STATUS_ERROR_BIT (uint8_t)4

#define ROBOTPART_HEATSAFT_VALUE 60
#define ROBOTPART_TROOGLEMODE_VALUE 20

#define CHASIS_FOLLOW_GIMBAL_ENABLE_V		0.01 // 严格底盘跟随云台的速度限制
#define CHASIS_FOLLOW_GIMBAL_RANGE 			0.0f // 0 dgrees 底盘追踪云台不严格范围


//#define CHASIS_SPEED_MAX 			2.0f // 2m/S
//#define CHASIS_SPEED_MAX_BOOST		3.0f
//#define CHASIS_SPEED_SIDE_MAX 		1.0f // 1m/S
//#define CHASIS_SPEED_SIDE_MAX_BOOST	2.0f

//#define CHASIS_SPEED_ACCEL			0.017f 			/*  2m/s^2 (val = accel/200)	*/
//#define CHASIS_SPEED_ACCELSHUT		0.025f 			/*  2m/s^2 (val = accel/200)	*/
//#define CHASIS_SPEED_SIDE_ACCEL		0.015f 			/*  1m/s^2 (val = accel/200)	*/


#define CHASIS_ANGYLAR_VELOCITY_MAX	0.01f 			/*	1r/s (val = rps*2pi/200)	*/

#define GIMBAL_PITCH_ANGYLAR_VELOCITY 600			/*	PI rad/s	Radian*10000/(PI*200) */
#define GIMBAL_YAW_ANGYLAR_VELOCITY	  600			/*	PI rad/s	Radian*10000/(PI*200) */

#define Y6020CORRECT 6500/*0 degree*/

#define SCMAXVOL 26.1f	/*10P SuperCapacity*/



/*机器人整体状态*/
/*
	Robot total mode.
	
	ROBOTMODE_IDLE means robot in IDLE mode, all functions will be disabled.
	
	ROBOTMODE_ESCAPE means robot in escape mode. Gimbal and Ammobooster will be enabled.
	Balance will disabel, but 2 wheel motors will work at open loop mode to help robot escape danger.
	
	MDOE_NORMAL means robot in normal mode. Gimbal,Ammobooster.Leg,Banlance will be enbaled.
*/
enum eRobotMode
{
	ROBOTMODE_IDLE=0,
	ROBOTMODE_ESCAPE,
	ROBOTMODE_NORMAL,
};


/*机器人部件序号*/
enum eRobotPart
{
	ROBOTPART_ID_LEG = 0,   //腿
	ROBOTPART_ID_BLC,		//平衡
	ROBOTPART_ID_GIM,		//云台
	ROBOTPART_ID_BST,		//发射机构
};

enum eMoveMode
{
	Move_Normal=0,
	Move_Fly,
	Move_Jump
};

/*机器人部件状态*/
/*
	Robot part statue.
	
	ROBOTPART_STATUS_IDLE 	means part  work in IDLE statue. Functions will be disabled.
	ROBOTPART_STATUS_START 	means part is going to work in normal statue. Start mode is a buffer besides in IDLE and Normal.
	ROBOTPART_STATUS_NORMAL 	means part work i normal statue. This statue shouldn't change in directly.
*/
enum eRobotStatus
{
	ROBOTPART_STATUS_IDLE=1,
	ROBOTPART_STATUS_START,
	ROBOTPART_STATUS_NORMAL,
};


/*Some functions' flag*/
/*Flag of BoosterMode*/
enum eRobotPartBoosterMode
{
	ROBOTPART_BOOSTER_STOP=0,	/* Booster Stop */
	ROBOTPART_BOOSTER_15MPS,	/* Boost ammos at 15m/s */
	ROBOTPART_BOOSTER_18MPS,	/* Boost ammos at 18m/s */
	ROBOTPART_BOOSTER_30MPS,	/* Boost ammos at 30m/s */
};

enum eRobotPartTriggleMode
{
	ROBOTPART_TRIGGLE_STOP=0,	/* Triggle Stop */
	ROBOTPART_TRIGGLE_MMODE0,	/* Triggle Muliti Mode 0 */
	ROBOTPART_TRIGGLE_MMODE1,	/* Triggle Muliti Mode 1 */
	ROBOTPART_TRIGGLE_SINGLE,	/* Triggle Single	Mode */
};

enum eRobotPartShotStatus
{
	ROBOTPART_SHOT_FREE=0,		/* Free to Shoot */
	ROBOTPART_SHOT_SHOOTING,	/* Shooting */
	ROBOTPART_SHOT_JAM,			/* Ammo Jam */
	ROBOTPART_SHOT_JAMSOLVING,	/* Ammo Jam Solving */
};

enum eRobotPartClipStatus
{
	ROBOTPART_CLIP_OVER75=0,	/* More than 75% ammos in Clip */
	ROBOTPART_CLIP_OVER50,		/* More than 50% ammos in Clip */
	ROBOTPART_CLIP_OVER25,	/* More than 25% ammos in Clip */
	ROBOTPART_CLIP_NONE,		/* No ammos in Clip */
};

enum eRobotPartChasisMode
{
	ROBOTPART_CHASIS_SHUTTLE=0,	/* Normal moving */
	ROBOTPART_CHASIS_FLYING,	/* Feipo */
	ROBOTPART_CHASIS_SIDEMODE,	/* Side mode */
	ROBOTPART_CHASIS_ROLLING,	/* Xiaotuoluo */
	ROBOTPART_CHASIS_JUMP,      /*Jump*/
};

enum eRobotJumpMode
{
	JUMP_READY=0,      //正常阻抗，收腿
	JUMP_STRETCH,      //跳跃阻抗，快速伸腿
	JUMP_SHRINK,      //收腿阻抗，空中收腿以提升跳跃高度
	JUMP_STOP,	      //正常阻抗，无腿部动作
	JUMP_LANDING,    //落地阻抗，阻抗增加以缓冲
};

/*
	Please read README.md to confirm the exactly functions
	
	This struct is the control package from CHASIS to GIMBAL.
	COMDOWNSIZE sizes of package in byte
	YawInc		Increase value of Yaw 	in radian.
	PitInc		Increase value of Pitch in radian.
	Fire		Open fire flag. Should be treat differently in one-shot or free-fire mode.
	BoosterMode Boost mode. Show ammo's velocity.
	TriggleMode	Triggle mode. Shows shoot frequence.
	CapOpen		Flag of open cap.
	DeathFlag	Is robot 0 HP.
	RSTFLAG		Robot reboot flag.May be not use.
*/
#define COMDOWNSIZE 5
typedef __PACKED_STRUCT
{
	int16_t YawInc;            //Yaw增量  -10000~10000表示-PI~PI 其他表示失能
	int16_t PitInc;            //Pitch增量 -10000~10000表示-PI~PI 其他表示失能
	uint8_t Fire  : 1;         //开火标志位
	uint8_t BoosterMode  : 2;  //摩擦轮射速模式
	uint8_t TriggleMode  : 2;  //拨盘模式
	uint8_t CapOpen      : 1;  //是否开启弹舱盖
	uint8_t GimDeathFlag : 1;  //云台关断标记位
	uint8_t RobotDeath   : 1;  //机器人阵亡标志位
}tComDown;

/*
	Please read README.md to confirm the exactly functions
	
	This struct is the message package from GIMBAL to CHASIS.
	
*/
#define COMUPSIZE 1
typedef __PACKED_STRUCT
{
	uint8_t autoAim_detect_flag;
}tComUP;


//传递遥控器数据包kmj0311
typedef __PACKED_STRUCT
{
  int16_t ch0;
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
}RC_Data_1;

typedef __PACKED_STRUCT
{

  uint8_t sw_left;
  uint8_t sw_right;
  int16_t vx;
  int16_t vy;
	int16_t vz;
}RC_Data_2;

typedef __PACKED_STRUCT
{
  uint8_t press_l;
  uint8_t press_r;
	int16_t wheel;
  union
  {
    uint16_t key_code;
    struct
    {
      uint16_t W : 1;
      uint16_t S : 1;
      uint16_t A : 1;
      uint16_t D : 1;
      uint16_t SHIFT : 1;
      uint16_t CTRL : 1;
      uint16_t Q : 1;
      uint16_t E : 1;
      uint16_t R : 1;
      uint16_t F : 1;
      uint16_t G : 1;
      uint16_t Z : 1;
      uint16_t X : 1;
      uint16_t C : 1;
      uint16_t V : 1;
      uint16_t B : 1;
    } key_bit;
  } keyboard;
	uint8_t gimbal_fire_flag;
}RC_Data_3;


struct tChasisVal
{
	float ChasisPsaiYaw;	//Chasis yaw angel
	float ChasisRoll;	//Chasis roll angel
	float ChasisPitch;	//Chasis roll angel
		
	float ChasisLegLen[2];	//Chasis leg length
	float ChasisLenDot;
	float ChasisFn;
	float ChasisLenAccel;
	float ChasisThetaAccel;
	float X[6]={0};			//Theta ThetaDot Xbody XbodyDot Phi PhitDot
							//PendulumRadian PendulumRadianSpeed Distance Velocity BodyPitchRadian BodyPitchRadianSpeed 
	arm_matrix_instance_f32 MatX = {6, 1, X};
};

class cChasisControl
{
	public:
	cChasisControl()
	{
		this->ChasisMode = ROBOTPART_CHASIS_SHUTTLE;
		this->LegLen = LEG_BOTTOM;
//		this->LQR.InitMatX(&this->TargetVal.MatX,&this->ObserveVal.MatX);
    this->Lleg_LQR.InitMatX(&this->Lleg_TargetVal.MatX,&this->Lleg_ObserveVal.MatX);
		this->Rleg_LQR.InitMatX(&this->Rleg_TargetVal.MatX,&this->Rleg_ObserveVal.MatX);
	}		
	
/********** Chasis statue manage **********/
	public:
			eRobotPartChasisMode LastChasisMode;
	protected:
	/*This flag is designed to make sure chasis control mode*/
	eRobotPartChasisMode ChasisMode;


	/*Is blance ok*/
	uint8_t BanlanFlag = 0;
	
	public:	
	/*To make sure is banlance ok*/
	inline void SetBalanceFlag(uint8_t IsOK)
	{this->BanlanFlag = IsOK;}
	inline uint8_t GetBalanceFlag(void)
	{return this->BanlanFlag;}	
/********** Escape moving **********/
	public:
	cLoopEscape LoopEscape[2];
	float EscapeCloseSpeed[2];
	void GetEsacpeVelocity(float* WheelSpeed)
	{
		WheelSpeed[0]=this->ChasisHead? -EscapeCloseSpeed[0]: EscapeCloseSpeed[0];
		WheelSpeed[1]=this->ChasisHead? -EscapeCloseSpeed[1]: EscapeCloseSpeed[1];	
	}
/********** Chasis moving **********/
	protected:
	uint8_t OffGround;
	public:
	cMotorUnit *MotorUnits;
	cVelFusionKF *VelKF;
	/*Set velocity kalman paramaters*/
	void SetVelKF(cVelFusionKF *pVelKF)
	{this->VelKF = pVelKF;}
	/*Get moving control paramaters*/
	float GetControlMaxVel(uint8_t boosterEnable)
	{
		if(this->GetChasisMode()==ROBOTPART_CHASIS_SIDEMODE)
		{
			return (boosterEnable)?LegMovingPara[(uint8_t)this->LegLen][3]*LegMovingPara[(uint8_t)this->LegLen][4]*LegMovingPara[(uint8_t)this->LegLen][5]:\
							LegMovingPara[(uint8_t)this->LegLen][3]*LegMovingPara[(uint8_t)this->LegLen][5];
		}
		else
		{
			return (boosterEnable)?LegMovingPara[(uint8_t)this->LegLen][3]*LegMovingPara[(uint8_t)this->LegLen][4]:\
							LegMovingPara[(uint8_t)this->LegLen][3];		
		}
	}
	
	float GetControlMaxAcc(uint8_t boosterEnable)
	{
		if(this->GetChasisMode()==ROBOTPART_CHASIS_SIDEMODE)
		{
			return (boosterEnable)?LegMovingPara[(uint8_t)this->LegLen][1]*LegMovingPara[(uint8_t)this->LegLen][4]*LegMovingPara[(uint8_t)this->LegLen][5]:\
							LegMovingPara[(uint8_t)this->LegLen][1]*LegMovingPara[(uint8_t)this->LegLen][5];
		}
		else
		{
			return (boosterEnable)?LegMovingPara[(uint8_t)this->LegLen][1]*LegMovingPara[(uint8_t)this->LegLen][4]:\
							LegMovingPara[(uint8_t)this->LegLen][1];		
		}	
	}
	
	float GetControlMaxReAcc(uint8_t boosterEnable)
	{
		if(this->GetChasisMode()==ROBOTPART_CHASIS_SIDEMODE)
		{
			return (boosterEnable)?LegMovingPara[(uint8_t)this->LegLen][2]*LegMovingPara[(uint8_t)this->LegLen][4]*LegMovingPara[(uint8_t)this->LegLen][5]:\
							LegMovingPara[(uint8_t)this->LegLen][2]*LegMovingPara[(uint8_t)this->LegLen][5];
		}
		else
		{
			return (boosterEnable)?LegMovingPara[(uint8_t)this->LegLen][2]*LegMovingPara[(uint8_t)this->LegLen][4]:\
							LegMovingPara[(uint8_t)this->LegLen][2];		
		}	
	}
	
	float ChasisForwardTargetVelocity=0.0f;	
	float Greavity_Forward=71.0f;
	float Check_OG_Value=20.0f;
	uint8_t JumpFlag;
	uint8_t MoveMode;
	uint8_t Move_Mid=0;
	uint8_t	Over_Flag=0;
	uint8_t Can_Jump_Flag=0;
	uint8_t	Jump_One_Flag=0;	
	/*Get Forward velocity*/
	float GetForwardVelocity(void)
	{
		if(this->GetChasisMode()==ROBOTPART_CHASIS_SIDEMODE)
		{
			if(CFGPositionRawErr>0.0f)
			{return -this->ChasisForwardTargetVelocity;}
			else
			{return  this-> ChasisForwardTargetVelocity;}
		
		}
		else{return this->ChasisHead?-ChasisForwardTargetVelocity:ChasisForwardTargetVelocity;}	
	}
	uint8_t CheckOG(void)
	{
		return this->OffGround;
	}
	void SetOG(uint8_t OG)
	{
		this->OffGround = OG;
	}
	
/********** Chassis Jump **********/
	float wz_set=4;
	float obervalx[2]={0};
	float accelX;
protected:
	eRobotJumpMode JumpMode;
	uint8_t LANDING=0;
public:
	inline void SetJumpMode(eRobotJumpMode _JumpMode)
	{this->JumpMode=_JumpMode;}
	inline eRobotJumpMode GetJumpMode(void)
	{return this->JumpMode;}
	eRobotJumpMode LastJumpMode;
	uint8_t CheckLand(void)
	{return this->LANDING;}
	void SetLand(uint8_t land)
	{this->LANDING=land;}
/********** Leg length manage **********/
	protected:
	/*Leg length level*/
	eRobotLegID LegLen;
	
	public:
	/*Close loop*/
	cLoopLen LoopLen[2];/*Left Right*/
	cLoopLen_Spf LoopLen_Spf[2];
	cLoopLen_Dot LoopLen_Dot[2];
	cLooplen_Ready LoopLen_Ready[2];
	cLooplen_Stop LoopLen_Stop[2];	
	/*Set length ID*/
	inline void SetLegLen(eRobotLegID ID)
	{this->LegLen = ID;}
	
	/*Return length ID*/
	inline eRobotLegID GetLegLenFlag(void)
	{return this->LegLen;}
	
	/*Return length in mm*/
	inline float GetLegLen(void)
	{return LegMovingPara[(uint8_t)this->LegLen][0];}	

/********** Chasis follow gimbal **********/
	protected:
	/*Variates about chasis follow gimbal */
	/*0 for X+, 1 for X-*/
	uint8_t ChasisHead = 0;			/*0 for revtime*/
	float CFGPositionCorVal = 0.0f;		/*Correct value*/
	float CFGPositionRawErr=0.0f;		/*Really Raw Error after Position Correct. Singel loop, each loop from +1 to -1*/
	int32_t CFGPositionLoops = 0;		/*Raw mulity loop*/
	
	uint8_t ChasisFollowGimbalConincidetly = 0; /*Is Chasis strict tracking*/
	uint8_t ChasisFollowGimbalEn = 0; 			/*Is chasis folow gimbal enable*/
	
	public:
	cLoopCFG LoopCFG;
	
	/*To make sure in shuttle side or other mods*/
	inline eRobotPartChasisMode GetChasisMode(void)
	{return this->ChasisMode;}
	inline void SetChasisMode(eRobotPartChasisMode Mode)
	{this->ChasisMode = Mode;}
	
	/*Functions about chasis head.(Postive direction) */
	/*
		Set raw error of chasis and gimbal.
		This value should be absolutely error of chasis and gimbal.
		Chasis is zero.
		Mulity loop, each loop from +1 to -1
		Rev time is positive
	*/
	void SetErrWithGim(float value)
	{
		if((value - this->CFGPositionRawErr)>1){this->CFGPositionLoops-=1;}
		else if((value - this->CFGPositionRawErr)<-1){this->CFGPositionLoops+=1;}
		this->CFGPositionRawErr = value;
	}
	
	float GetErrWithGimRaw(void)
	{
		return this->CFGPositionRawErr;
	}
	
	/*Get head direction*/
	inline uint8_t GetChasisHead(void)
	{return this->ChasisHead;}

	
	/*Change chasis and gimbal error's direction*/
	void RefreshChasisHead(void)
	{
		/*Find minimum radian to change head*/
		if(this->CFGPositionRawErr < -0.5)
		{this->CFGPositionCorVal = -1;this->ChasisHead=1;}
		else if(this->CFGPositionRawErr > 0.5)
		{this->CFGPositionCorVal =  1;this->ChasisHead=1;}
		else{this->CFGPositionCorVal = 0;this->ChasisHead=0;}
	}
	
	/*Get chasis follow gimbal error*/
	/*Adopt for side mode or follow mode*/
	/*Uesd for PID feedback, while ref = 0*/
	float GetCFGControlError(void)
	{ 
		/*Refresh positive position*/
		this->RefreshChasisHead();
		/*Observation value*/
		float obv = CFGPositionLoops + this->CFGPositionRawErr;
		/*Target value*/
		float target = CFGPositionLoops + this->CFGPositionCorVal;
		/*Error*/
		float error = target - obv;
		/*Chech Sidemode statue*/
		if(this->GetChasisMode()==ROBOTPART_CHASIS_SIDEMODE)
		{
			if(error<0.0f)
			{error+=0.5f;}
			else{error-=0.5f;}
		}
		/*	
				loop = 2 raw = 0.3 ->cor =0
				obv = 2.3
				target = 2
				side target = 2.5,side cor = 0.5
				
				loop =2 raw = 0.6 ->cor = 1
				obv = 2.6
				target = 3
				side mode = 2.5,side cor = -0.5
				
				loop = 2 raw = -0.3 ->cor =0
				obv = 1.7
				target = 2
				side target = 1.5,side cor = -0.5
				
				loop =2 raw = -0.6 ->cor = -1
				obv = 1.4
				target = 1
				side mode = 1.5,side cor = 0.5
		*/
		/*Error*/
		return error;
	}

	/*Set chasis follow gimbal conincidently enable or not*/
	inline void SetCFGSStatue(uint8_t IsEnable)
	{this->ChasisFollowGimbalConincidetly = IsEnable?1:0;}
	/*Check is strict tracking*/
	inline uint8_t GetCFGSFlag(void)
	{return this->ChasisFollowGimbalConincidetly;}
	/*Set is chasis folow gimbal enable*/
	inline void SetCFGENStatue(uint8_t IsEnable)
	{this->ChasisFollowGimbalEn = IsEnable?1:0;}
	/*Check is chasis folow gimbal enable*/
	inline uint8_t GetCFGENFlag(void)
	{return this->ChasisFollowGimbalEn;}
	
	void Set_GravityForward(float gravityx)
	{
		this->Greavity_Forward=gravityx;
	}
	
	float Get_GravityForward(void)
	{
		return this->Greavity_Forward;
	}
	
	void Set_CheckOG_Value(float OGx)
	{
	this->Check_OG_Value=OGx;
	}
	
	float Get_CheckOG_Value(void)
	{
	return this->Check_OG_Value;
	}
	
	void Set_MoveMode(uint8_t MoveModex)
	{
		this->MoveMode=MoveModex;
	}
	
	uint8_t Get_MoveMode(void)
	{
	return	this->MoveMode;
	}
	void Set_JumpFlag(uint8_t JumpFlagx)
	{
		this->JumpFlag=JumpFlagx;
	}
	
	uint8_t Get_JumpFlag(void)
	{
	return	this->JumpFlag;
	}
	void Set_FlyMid_Flag(uint8_t Move_Midx)
	{
		this->Move_Mid=Move_Midx;
	}
	
	uint8_t Get_FlyMid_Flag(void)
	{
	return	this->Move_Mid;
	}
	void Set_Jump_One_Flag(uint8_t Jump_One_Flagx)
	{
		this->Jump_One_Flag=Jump_One_Flagx;
	}
			void Set_Can_Jump_Flag(uint8_t Can_Jump_Flagx)
	{
		this->Can_Jump_Flag=Can_Jump_Flagx;
	}
	 void Set_JumpOver_Flag(uint8_t Over_Flagx)
	{
		this->Over_Flag=Over_Flagx;
	}
	
	uint8_t Get_JumpOver_Flag(void)
	{
	return	this->Over_Flag;
	}
	uint8_t Get_Can_Jump_Flag(void)
	{
	return	this->Can_Jump_Flag;
	}
	uint8_t Get_Jump_One_Flag(void)
	{
	return	this->Jump_One_Flag;
	}	
/********** Loop ROLL **********/
	public:
	cLoopRoll LoopRoll;
	
/********** Loop YAW **********/	
	public:
	cLoopYaw LoopYaw;		

/********** Loop GYRO **********/	
	public:
	cLoopGyro LoopGyro;	

/********** Loop RollOffset	**********/	

	public:
	cLoopRollOffset LoopRollOffset;

/********** Loop Theta **********/	
	public:
	cLoopTheta LoopTheta;
	cLoopTheta_Spf LoopTheta_Spf;
/********** LQR Balance **********/
	protected:
	
		
	public:
	cLQR Lleg_LQR;
	cLQR Rleg_LQR;
	//底盘数据
	tChasisVal TargetVal;	/*Refer value*/
	tChasisVal ObserveVal;	/*Observed value*/	
	//腿部数据
	tChasisVal Lleg_TargetVal;	/*Refer value*/
	tChasisVal Lleg_ObserveVal;	/*Observed value*/	
	tChasisVal Rleg_TargetVal;	/*Refer value*/
	tChasisVal Rleg_ObserveVal;	/*Observed value*/	  
/********** Super Capacity manage **********/
	protected:
	float RemainPower = 0.0f;
	float ScVoltage = 0.0f;
	float CoeffSCP = 1.0f/(SCMAXVOL*SCMAXVOL);
	
	public:
	void SCPowerRemianInput(uint8_t *datapack)
	{
		int32_t voltage = 0;
		voltage = ((uint32_t)datapack[0]<<24) | ((uint32_t)datapack[1]<<16) | ((uint32_t)datapack[2]<<8) | ((uint32_t)datapack[3]);
		this->ScVoltage = 0.0001f*voltage;
		this->RemainPower = CoeffSCP*this->ScVoltage*this->ScVoltage;
	}
	inline float SCPowerGet(void)
	{return this->RemainPower;}
};


class cRobotControl
{
	public:
	float testTorqueR[2]={0};
	float testTorque1[2]={0};
	float testTorque2[2]={0};
	float lqr[2]={0};
	protected:
	/*[0,1]表示状态 [2]表示有无异常*/
	/*状态设置与异常标志位完全分离*/
	uint8_t	RobotMode:3;			//整机器人
	uint8_t LegStatus:3;			//腿状态
	uint8_t BlcStatus:3;			//平衡状态
	uint8_t GimbalStatus:3;			//云台状态
	uint8_t BoosterStatus:3;		//发射状态
	
	robot_referee_status_t::ext_game_robot_status_t *ext_game_robot_status;//Robot status 0x0202
	robot_referee_status_t::ext_power_heat_data_t *ext_power_heat_data;//Robot power heat data 0x0203
	
	
	public:
	cRobotControl()
	{
		this->LegStatus = ROBOTPART_STATUS_IDLE;
		this->BlcStatus = ROBOTPART_STATUS_IDLE;
		this->GimbalStatus = ROBOTPART_STATUS_IDLE;
		this->BoosterStatus = ROBOTPART_STATUS_IDLE;
	}
		
	cINS *INS=0;
	uint32_t UpLose = 0;
	
	tComDown ComDown;
	tComUP	 ComUP;
	cChasisControl ChasisControl;
	
		/*  kmj 0311*/
	RC_Data_1 rc_data_1;
	RC_Data_2 rc_data_2;	
	RC_Data_3 rc_data_3;	
	
	uint8_t isUIRefresh = 0;
	
	void SetINS(cINS *pINS)
	{this->INS = pINS;}
	void UpdataABSGimbal(void);
	void UpdateABSChasis(void);
	
	/*Set refer system data*/
	void SetRefSysDataSource(robot_referee_status_t::ext_game_robot_status_t *robot_status,robot_referee_status_t::ext_power_heat_data_t *power_heat_data)
	{
		this->ext_game_robot_status = robot_status;
		this->ext_power_heat_data = power_heat_data;
	}
	uint16_t GetRobotHP(void)
	{return this->ext_game_robot_status->remain_HP;}
//	uint16_t GetRobotRemainHeat(void)
//	{return (this->ext_game_robot_status->shooter_barrel_heat_limit - this->ext_power_heat_data->shooter_id1_17mm_cooling_heat);}
	uint8_t GetFireFlag(void)
	{
		return this->ext_power_heat_data->shooter_id1_17mm_cooling_heat < this->ext_game_robot_status->shooter_barrel_heat_limit-ROBOTPART_HEATSAFT_VALUE;
	}
	uint16_t GetRobotCoolRate(void)
	{return this->ext_game_robot_status->shooter_barrel_cooling_value;}
//	uint16_t GetRobotAmmoSpeed(void)
//	{return this->ext_game_robot_status->shooter_id1_17mm_speed_limit;}
	
	/**@Input RobotMode which follow eRobotMode*/
	/*This function will set robot mode, which will not change error flag*/
	inline void SetRobotMode(eRobotMode mode)
	{
		this->RobotMode &= 0x04;
		this->RobotMode |= mode;
	}
	/*This function will SET robot error flag*/
	inline void SetRobotError(void)
	{this->RobotMode |= 0x04;}
	/*This function will CLEAR robot error flag*/
	inline void ClearRobotError(void)
	{this->RobotMode &= 0x03;}
	
	/**@Input part follow eRobotPart*/
	/**@Input mode follow eRobotStatus*/
	/*This function will set robot mode, which will not change error flag*/
	void SetPartMode(eRobotPart Part, eRobotStatus mode)
	{
		switch (Part)
		{
			case ROBOTPART_ID_LEG:
			{this->LegStatus &= 0x04;this->LegStatus |= mode;}
			break;
			case ROBOTPART_ID_BLC:
			{this->BlcStatus &= 0x04;this->BlcStatus |= mode;}
			break;
			case ROBOTPART_ID_GIM:
			{this->GimbalStatus &= 0x04;this->GimbalStatus |= mode;}
			break;
			case ROBOTPART_ID_BST:
			{this->BoosterStatus &= 0x04;this->BoosterStatus |= mode;}
			break; 
		}
	}
	
	/**@Input part follow eRobotPart*/
	/*This function will SET error flag*/
	void SetPartError(eRobotPart Part)
	{
		switch (Part)
		{
			case ROBOTPART_ID_LEG:
			{this->LegStatus |= 0x04;}
			break;
			case ROBOTPART_ID_BLC:
			{this->BlcStatus |= 0x04;}
			break;
			case ROBOTPART_ID_GIM:
			{this->GimbalStatus |= 0x04;}
			break;
			case ROBOTPART_ID_BST:
			{this->BoosterStatus |= 0x04;}
			break; 
		}
	}
	/**@Input part follow eRobotPart*/
	/*This function will CLEAR error flag*/
	void ClearPartError(eRobotPart Part)
	{
		switch (Part)
		{
			case ROBOTPART_ID_LEG:
			{this->LegStatus &= 0x03;}
			break;
			case ROBOTPART_ID_BLC:
			{this->BlcStatus &= 0x03;}
			break;
			case ROBOTPART_ID_GIM:
			{this->GimbalStatus &= 0x03;}
			break;
			case ROBOTPART_ID_BST:
			{this->BoosterStatus &= 0x03;}
			break; 
		}
	}
	
	inline uint8_t CheckRobotMode(void)
	{return this->RobotMode;}
	
	uint8_t CheckPartMode(eRobotPart Part)
	{
		switch (Part)
		{
			case ROBOTPART_ID_LEG:
			{return this->LegStatus;}
			break;
			case ROBOTPART_ID_BLC:
			{return this->BlcStatus;}
			break;
			case ROBOTPART_ID_GIM:
			{return this->GimbalStatus;}
			break;
			case ROBOTPART_ID_BST:
			{return this->BoosterStatus;}
			break; 
		}
	}	
};

extern "C"{

}
#endif
#endif
