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

#define WHEELCOEFF	0.0765f /*�뾶*/



#define ANGTORAD  0.00017453f   //0.01��/LSBת����
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
	/*[0]��ʾ���� 1��0��*/
	/*[1]��ʾǰ�� 0ǰ1��*/
	uint8_t  MotorMode[2];
	
	/*��λ���µ�����У׼ֵ  5 degree��װ�н�*/
	float	 LimitOffset = 0.08726646f;
	
	/*У����Ļ�����, X����Ϊ��ͷ ,Y������ֱ���� ,Z��ʸ���� ,���Ҳ೵�ֶ�����*/
	float 	 Radian	 = 0.0f;
	
	
	public:
	/*ͨѶ��������*/
	uint32_t loseCom = 0;
	
	/*���������������, �����趨����ϵ����*/
	void UpdateMotor(void)
	{
		/*��ֱ�Ӱ����У����ı���������*/
		float tmp = this->Msr->ecd * EncoderLSB;
		
		/*��෭ת���*/
		if(this->MotorMode[0])
		{tmp = 2*PI - tmp;}
		
		/*��λУ׼����*/
		if(this->MotorMode[1])
		{tmp = tmp + PI - this->LimitOffset;}
		else
		{tmp = tmp + this->LimitOffset;}
				
		
		/* ��һ���� [0,2PI) */
		if(tmp<0){this->Radian = tmp + 2*PI;}
		else if(tmp>2*PI){this->Radian = tmp - 2*PI;}
		else {this->Radian = tmp;}
	}
	
	inline float GetRadian(void)
	{return this->Radian;}
	
	/*[0]��ʾ���� 0��1��*/
	/*[1]��ʾǰ�� 0ǰ1��*/
	void SetMotorMode(uint8_t Para0, uint8_t Para1)
	{this->MotorMode[0]=Para0;this->MotorMode[1]=Para1;}
	
	void Init(){}
};

class cKF9025 : public cMotor
{
	protected:
	float KWheelD = 0.160f * PI / 360.0f;	//��ֱ�� * PI / 360
	float Speed = 0.0f;		//�����ٶ�
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
	/*[0]��ʾ���� 1��0��*/
	/*[1]��ʾǰ�� 0ǰ1��*/
	uint8_t  MotorMode[2];
	
	/*��λ���µ�����У׼ֵ  5 degree��װ�н�*/
	//float	 LimitOffset = 0.08726646f;
	float	 LimitOffset = 0.00296705f;
	
	/*У����Ļ�����, X����Ϊ��ͷ ,Y������ֱ���� ,Z��ʸ���� ,���Ҳ೵�ֶ�����*/
	float 	 Radian	 = 0.0f;
	
	
	public:
	/*ͨѶ��������*/
	uint32_t loseCom = 5;
	
	/*���������������, �����趨����ϵ����*/
	void UpdateMotor(void)
	{
		/*��ֱ�Ӱ����У����ı���������*/
		float tmp = this->cDMMotor::GetRadian();	
		/*�Ҳ෭ת���*/
		if(!this->MotorMode[0])
		{tmp = 2*PI - tmp;}
		
		/*��λУ׼����*/
		if(this->MotorMode[1])
		{tmp = tmp + PI - this->LimitOffset;}
		else
		{tmp = tmp + this->LimitOffset;}

		/* ��һ���� [0,2PI) */
		if(tmp<0){tmp = tmp + 2*PI;}
		else if(tmp>2*PI){tmp = tmp - 2*PI;}
		else {tmp = tmp;}
		this->Radian=tmp;
	}
	
	inline float GetRadian(void)
	{return this->Radian;}
	/*[0]��ʾ���� 0��1��*/
	/*[1]��ʾǰ�� 0ǰ1��*/
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
	
	cLinkSolver LinkSolver[2];/*0��1��*/
	
	void UpdateLink(void);
	void UpdateOdomentor(void);
	
	void InitOdomentor(void)
	{this->displacement=0.0f;}
	
	inline float GetVel(void)
	{return this->velocity;}
	inline float GetDis(void)
	{return this->displacement;}

	
	/*0��ؽں� 1��ؽ�ǰ 4������*/
	/*3�ҹؽں� 2�ҹؽ�ǰ 5������*/
};

extern "C" {
void USART3_IRQHandler(void);
void USART2_IRQHandler(void);
}

#endif
#endif