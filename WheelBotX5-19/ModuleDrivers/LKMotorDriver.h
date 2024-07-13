/*********************************************************************************
  *FileName:	  LKMotorDriver.h
  *Author:  	  zyx
  *Details: 	  겿ص������
  
  *Version:  	1.0
  *Date:  		2023/05/09
  *Describe:    ��������
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
	
	uint8_t Motor_id;//���id
	uint8_t error;//�����־
	

	uint16_t ecd;//��������ֵ
	int32_t accel;//���ٶ�
	int16_t iq;//ת�ص���ֵ
	int32_t speedcontrol;//�ٶȿ�������
	int32_t anglecontrol;//λ�ÿ�������
	int16_t iqcontrol;//ת�ص�������ֵ
	uint16_t ecd_raw;//������ԭʼλ��
	uint16_t ecd_offset;//��������ƫ
	int64_t motor_angle;//����Ƕ�
	uint32_t circle_angle;//�����Ȧ�Ƕ�
	uint16_t voltage;//��ѹ
	int16_t power;//����������
	int8_t temperature;//�¶�
	int16_t speed;//��ǰת��
	uint8_t TorqueReverse;

}Motor_Measurement_t;

typedef __PACKED_STRUCT
{
	uint16_t   anglekp;//�ǶȻ�
	uint16_t   angleki;
	uint16_t   speedkp;//�ٶȻ�
	uint16_t   speedki;
	uint16_t   iqkp;   //ת�ػ�
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

	Motor_Measurement_t* Msr ;//����
	Motor_Measurement_t* Params;
//	Motor_Pid_t* com_pid;//����
//	Motor_Pid_t* pid;
};

extern "C" {
	uint8_t checksum(uint8_t* p_data, int8_t data_len);
	int8_t header_check(uint8_t* buf);
	int8_t data_check(uint8_t* buf, int8_t len);
}
#endif
#endif
