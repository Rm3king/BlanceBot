#ifndef MOTORTASK_H
#define MOTORTASK_H
#ifdef __cplusplus
#include "main.h"
#include "fdcan.h"
#include "Filter.h"
#include "arm_math.h"
#include "DMDriver.h"
#include "LKMotorDriver.h"
#include "RS485.h"
#include "LinkSolver.h"
#include "fdcan.h"
#include "RMMotorDriver.h"

#define AngelCalib   0.34472f
#define AngelCalib0  1.915516f // PI/2 + CAL
#define AngelCalib1  1.226076f // PI/2 - CAL

#define WHEELCOEFF	0.0765f /*半径*/



#define ANGTORAD  0.00017453f   //0.01°/LSB转弧度
#define MOTOR_DATA_LEN  64
#define EncoderLSB  	0.00009587380f //16bit to Radian
#define MOTORUPLIMIT    0.1745330f    //10 degree
#define MOTORUPLIMITRES 2.9670597f    //180-10 degree
#define PWR_LIMIT 		(int16_t)200

class cMotorBUS : public cRS485
{
	public:
	uint8_t* pprecbuf = 0;
};

class cMG8016 : public cMotor
{
	protected:
	/*[0]表示左右 1左0右*/
	/*[1]表示前后 0前1后*/
	uint8_t  MotorMode[2];
	
	/*限位导致的置零校准值  5 degree安装夹角*/
	float	 LimitOffset = 0.08726646f;
	
	/*校正后的弧度制, X正向为车头 ,Y正向竖直向下 ,Z箭矢向左 ,左右侧车轮都这样*/
	float 	 Radian	 = 0.0f;
	
	
	public:
	/*通讯丢包计数*/
	uint32_t loseCom = 0;
	
	/*减速箱编码器计算, 按照设定坐标系进行*/
	void UpdateMotor(void)
	{
		/*先直接按电机校正后的编码器计数*/
		float tmp = this->Msr->ecd * EncoderLSB;
		
		/*左侧翻转电机*/
		if(this->MotorMode[0])
		{tmp = 2*PI - tmp;}
		
		/*零位校准补偿*/
		if(this->MotorMode[1])
		{tmp = tmp + PI - this->LimitOffset;}
		else
		{tmp = tmp + this->LimitOffset;}
				
		
		/* 归一化到 [0,2PI) */
		if(tmp<0){this->Radian = tmp + 2*PI;}
		else if(tmp>2*PI){this->Radian = tmp - 2*PI;}
		else {this->Radian = tmp;}
	}
	
	inline float GetRadian(void)
	{return this->Radian;}
	
	/*[0]表示左右 0左1右*/
	/*[1]表示前后 0前1后*/
	void SetMotorMode(uint8_t Para0, uint8_t Para1)
	{this->MotorMode[0]=Para0;this->MotorMode[1]=Para1;}
	
	void Init(){}
};

class cKF9025 : public cMotor
{
	protected:
	float KWheelD = 0.160f * PI / 360.0f;	//轮直径 * PI / 360
	float Speed = 0.0f;		//轮线速度
	float Radian	 = 0.0f;
	uint16_t LastAng=0;
	public:
	uint32_t loseCom = 0;
	void Init(void)
	{};
	inline void UpdateData(void)
	{
		this->Speed = this->Msr->speed * this->KWheelD;
		this->Radian= this->Msr->ecd *EncoderLSB;
	}
	inline float GetSpeed(void)
	{
		return this->Speed;
	}
	inline float GetRadian(void)
	{	
		return this->Radian;
	}
};

class cDM8006 : public cDMMotor
{
	/*FDCAN Driver*/
	protected:
	uint16_t CANID;
	FDCAN_HandleTypeDef *hfdcan;
	FDCAN_TxHeaderTypeDef TxHeader;
	void CAN_Transmit(uint8_t *pdata)
	{HAL_FDCAN_AddMessageToTxFifoQ(this->hfdcan, &this->TxHeader, pdata);}
	
	public:
	void SetID(FDCAN_HandleTypeDef *hfdcan,uint16_t ID)
	{
		this->CANID	 = ID;
		this->hfdcan = hfdcan;
		
		this->TxHeader.Identifier = ID;
		this->TxHeader.IdType = FDCAN_STANDARD_ID;
		this->TxHeader.TxFrameType = FDCAN_DATA_FRAME;
		this->TxHeader.DataLength = FDCAN_DLC_BYTES_8;
		this->TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
		this->TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
		this->TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
		this->TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
		this->TxHeader.MessageMarker = 0;
	}	

	/*Angel Calibration*/
	protected:
	/*[0]表示左右 1左0右*/
	/*[1]表示前后 0前1后*/
	uint8_t  MotorMode[2];
	
	/*限位导致的置零校准值  5 degree安装夹角*/
	//float	 LimitOffset = 0.08726646f;
	float	 LimitOffset = 0.00296705f;
	
	/*校正后的弧度制, X正向为车头 ,Y正向竖直向下 ,Z箭矢向左 ,左右侧车轮都这样*/
	float 	 Radian	 = 0.0f;
	
	
	public:
	/*通讯丢包计数*/
	uint32_t loseCom = 5;
	
	/*减速箱编码器计算, 按照设定坐标系进行*/
	void UpdateMotor(void)
	{
		/*先直接按电机校正后的编码器计数*/
		float tmp = this->cDMMotor::GetRadian();	
		/*右侧翻转电机*/
		if(!this->MotorMode[0])
		{tmp = 2*PI - tmp;}
		
		/*零位校准补偿*/
		if(this->MotorMode[1])
		{tmp = tmp + PI - this->LimitOffset;}
		else
		{tmp = tmp + this->LimitOffset;}

		/* 归一化到 [0,2PI) */
		if(tmp<0){tmp = tmp + 2*PI;}
		else if(tmp>2*PI){tmp = tmp - 2*PI;}
		else {tmp = tmp;}
		this->Radian=tmp;
	}
	
	inline float GetRadian(void)
	{return this->Radian;}
	/*[0]表示左右 0左1右*/
	/*[1]表示前后 0前1后*/
	void SetMotorMode(uint8_t Para0, uint8_t Para1)
	{this->MotorMode[0]=Para0;this->MotorMode[1]=Para1;}
};

class cMotorUnit
{
	protected:
	float velocity = 0.0f;
	float dlast[2] = {0};
	float displacement = 0.0f;
	
	cFilterBTW2_40Hz FilterV;
	cFilterBTW2_40Hz FilterD;
	
	public:
	cMotorBUS BUS[2];/*L R*/
	cKF9025 KF9025[2];/*L R*/
	//cMG8016	LEGMotor[4];/*L2 L3 R2 R3*/
	uint32_t LEGMotorLoseCom = 0;
	
	cDM8006	LEGMotor[4];/*L2 L3 R2 R3*/
	
	cRMMotor Y6020;
	
	cLinkSolver LinkSolver[2];/*0左1右*/
	
	void UpdateLink(void);
	void UpdateOdomentor(void);
	
	void InitOdomentor(void)
	{this->displacement=0.0f;}
	
	inline float GetVel(void)
	{return this->velocity;}
	inline float GetDis(void)
	{return this->displacement;}

	
	/*0左关节后 1左关节前 4左轮子*/
	/*3右关节后 2右关节前 5右轮子*/
};

extern "C" {
void USART3_IRQHandler(void);
void USART2_IRQHandler(void);
}

#endif
#endif