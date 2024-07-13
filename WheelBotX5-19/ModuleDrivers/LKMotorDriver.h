/*********************************************************************************
  *FileName:	  LKMotorDriver.h
  *Author:  	  zyx
  *Details: 	  瓴控电机驱动
  
  *Version:  	1.0
  *Date:  		2023/05/09
  *Describe:    创建代码
**********************************************************************************/
#ifndef LKMOTORDRIVER_H
#define LKMOTORDRIVER_H
#include "main.h"
#ifdef __cplusplus
#define TORQUECONSTANT_9025_16T 0.32f
#define TORQUECONSTANT_9025_35T 0.81f
#define TORQUECONSTANT_8016_EV2 0.24f*6.0f
//typedef __PACKED_STRUCT



//typedef __PACKED_STRUCT
//{
//	int16_t max_ecd;
//	uint8_t direction;
//	uint16_t max_speed;
//	uint8_t Motor_id;
//	
//}Motor_Params_t;

typedef __PACKED_STRUCT
{
	
	uint8_t Motor_id;//电机id
	uint8_t error;//错误标志
	

	uint16_t ecd;//编码器数值
	int32_t accel;//加速度
	int16_t iq;//转矩电流值
	int32_t speedcontrol;//速度控制输入
	int32_t anglecontrol;//位置控制输入
	int16_t iqcontrol;//转矩电流控制值
	uint16_t ecd_raw;//编码器原始位置
	uint16_t ecd_offset;//编码器零偏
	int64_t motor_angle;//电机角度
	uint32_t circle_angle;//电机单圈角度
	uint16_t voltage;//电压
	int16_t power;//电机输出功率
	int8_t temperature;//温度
	int16_t speed;//当前转速
	uint8_t TorqueReverse;

}Motor_Measurement_t;

typedef __PACKED_STRUCT
{
	uint16_t   anglekp;//角度环
	uint16_t   angleki;
	uint16_t   speedkp;//速度环
	uint16_t   speedki;
	uint16_t   iqkp;   //转矩环
	uint16_t   iqki;
}Motor_Pid_t;


class cMotor
{
	protected:
	float TqConst = 0.0f;
	
	public:
	//	cMotor(){} ;
	//	~cMotor() {};
	void UpdateTorque(float Torque);
	void SetIqConst(float value);
	
	void SetMsr(Motor_Measurement_t* _com_msr);
	void SetParams(Motor_Measurement_t* _msr);
	void SetIqControl(uint8_t* buf);
	void SetEcdOffset(uint8_t* buf);
	void SetSpeedControl(uint8_t* buf);
	void SetMultiAngleControl(uint8_t* buf);

	void AskForAccel(uint8_t* buf);
	void AskForEncoder(uint8_t* buf);
	void AskForMultiAngle(uint8_t* buf);
	void AskForSingleAngle(uint8_t* buf);
	void AskForCircleAngle(uint8_t* buf);
	void AskForError(uint8_t* buf);
	void AskForClearError(uint8_t* buf);
	void AskForStopMotor(uint8_t* buf);

	void ReceiveAccel(uint8_t* buf);
	void ReceiveEncoder(uint8_t* buf);
	void ReceiveMutltiAngel(uint8_t* buf);
	void ReceiveSingleAngle(uint8_t* buf);
	void ReceiveIqMsr(uint8_t* buf);
	void ReceiveError(uint8_t* buf);

	void SetInitPla(uint8_t* buf);
	virtual void Init() = 0;

	Motor_Measurement_t* Msr ;//测量
	Motor_Measurement_t* Params;
//	Motor_Pid_t* com_pid;//测量
//	Motor_Pid_t* pid;
};

extern "C" {
	uint8_t checksum(uint8_t* p_data, int8_t data_len);
	int8_t header_check(uint8_t* buf);
	int8_t data_check(uint8_t* buf, int8_t len);
}
#endif
#endif
