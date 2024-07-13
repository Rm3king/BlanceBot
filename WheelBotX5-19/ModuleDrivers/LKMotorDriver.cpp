/*********************************************************************************
  *FileName:	  LKMotorDriver.cpp
  *Author:  	  zyx
  *Details: 	  瓴控电机驱动
  *Watchout		  9025正电流逆时针, 8016正电流顺时针
  
  *Version:  	1.0
  *Date:  		2023/05/09
  *Describe:    创建代码
**********************************************************************************/
#include "LKMotorDriver.h"
#include "stdio.h"
#include "string.h"
/*检测数据 累加和校验*/
uint8_t checksum(uint8_t* p_data, int8_t data_len)
{
	uint32_t sum = 0;
	while (data_len--) {
		sum += *(p_data++);
	}
	return (sum & 0xFF);
}
int8_t header_check(uint8_t* buf)
{
	uint8_t sum=checksum(buf, 4);
	if (sum == *(buf + 4))
		return 0;
	else
		return 1;
}
int8_t data_check(uint8_t* buf,int8_t len )
{
	uint8_t sum = checksum(buf, len);
	if (sum == *(buf + len))
		return 0;
	else
		return 1;
}

/*Update Tq const*/
void cMotor::SetIqConst(float value)
{
	this->TqConst = 2000.0f/(value*32.0f);
}

/*float torque to int16_t Iq*/
void cMotor::UpdateTorque(float Torque)
{
	int16_t iq = (int16_t)(Torque*this->TqConst);
	if(iq>2000){iq=2000;}
	else if(iq<-2000){iq=-2000;}
	this->Params->iqcontrol = iq;
}

/*设置参数指针*/
void cMotor::SetMsr(Motor_Measurement_t* _com_msr)                            
{
	this->Msr = _com_msr;
}
void cMotor::SetParams(Motor_Measurement_t* _msr)
{
	this->Params = _msr;
}

/*读取加速度*/
void cMotor::AskForAccel(uint8_t* buf)
{
	buf[0] = 0x3E;
	buf[1] = 0x33;
	buf[2] = this->Params->Motor_id;
	buf[3] = 0x00;
	buf[4] = checksum(buf, 4);
}

/*解析加速度*/
void cMotor::ReceiveAccel(uint8_t* buf)
{
	this->Msr->accel = (buf[3] << 24 | buf[2] << 16 | buf[1] << 8 | buf[0]);
}

/*读取编码器*/
void cMotor::AskForEncoder(uint8_t* buf)
{

	buf[0] = 0x3E;
	buf[1] = 0x90;
	buf[2] = this->Params->Motor_id;
	buf[3] = 0x00;
	buf[4] = checksum(buf, 4);
}

/*解析编码器数据*/
void cMotor::ReceiveEncoder(uint8_t* buf)
{
	this->Msr->ecd 			= (buf[1] << 8 | buf[0]);
	this->Msr->ecd_raw		= (buf[3] << 8 | buf[2]);
	this->Msr->ecd_offset	= (buf[5] << 8 | buf[4]);
}

/*读取多圈角度*/
void cMotor::AskForMultiAngle(uint8_t* buf)
{
	buf[0] = 0x3E;
	buf[1] = 0x92;
	buf[2] = this->Params->Motor_id;
	buf[3] = 0x00;
	buf[4] = checksum(buf, 4);
}

/*解析多圈角度*/
void cMotor::ReceiveMutltiAngel(uint8_t* buf)
{
	memcpy(&this->Msr->motor_angle,buf,8);
}

/*读取单圈角度*/
void cMotor::AskForSingleAngle(uint8_t* buf)
{
	buf[0] = 0x3E;
	buf[1] = 0x94;
	buf[2] = this->Params->Motor_id;
	buf[3] = 0x00;
	buf[4] = checksum(buf, 4);
}

/*解析单圈角度*/
void cMotor::ReceiveSingleAngle(uint8_t* buf)
{
	this->Msr->circle_angle = (buf[3] << 24 | buf[2] << 16 | buf[1] << 8 | buf[0]);
}

/*转矩闭环指令*/
void cMotor::SetIqControl(uint8_t* buf)
{
	buf[0] = 0x3E;
	buf[1] = 0xA1;
	buf[2] = this->Params->Motor_id;
	buf[3] = 0x02;
	buf[4] = checksum(buf, 4);
	buf[5] = this->Params->iqcontrol & 0xFF;
	buf[6] = this->Params->iqcontrol >>8;
	buf[7] = checksum(buf+5, 2); 
}

/*解析转矩闭环返回信息*/
void cMotor::ReceiveIqMsr(uint8_t* buf)
{
	this->Msr->temperature	=	buf[0];
	this->Msr->iq			=	((buf[2]<<8)|buf[1]);
	this->Msr->speed		=	((buf[4]<<8)|buf[3]);
	this->Msr->ecd			=	((buf[6]<<8)|buf[5]);
}

/*写入编码器值作为电机零点*/
void cMotor::SetEcdOffset(uint8_t* buf)
{
	buf[0] = 0x3E;
	buf[1] = 0x91;
	buf[2] = this->Params->Motor_id;
	buf[3] = 0x02;
	buf[4] = checksum(buf, 4);
	buf[5] = this->Params->ecd_offset & 0xFF;
	buf[6] = this->Params->ecd_offset>>8;
	buf[7] = checksum(buf+5, 2);
}

/*读取电机状态1和错误标志命令*/
void cMotor::AskForError(uint8_t* buf)
{
	buf[0] = 0x3E;
	buf[1] = 0x9A;
	buf[2] = this->Params->Motor_id;
	buf[3] = 0x00;
	buf[4] = checksum(buf, 4);
}

/*清除电机错误标志命令*/
void cMotor::AskForClearError(uint8_t* buf)
{
	buf[0] = 0x3E;
	buf[1] = 0x9B;
	buf[2] = this->Params->Motor_id;
	buf[3] = 0x00;
	buf[4] = checksum(buf, 4);
}

/*解析电机状态1和错误标志命令*/
/*解析清除电机错误标志命令*/
void cMotor::ReceiveError(uint8_t* buf)
{
	this->Msr->temperature=buf[0];
	this->Msr->voltage=((buf[3]<<8)|buf[2]);
	this->Msr->error=buf[6];
}


/*电机停止命令*/
void cMotor::AskForStopMotor(uint8_t* buf)
{
	buf[0] = 0x3E;
	buf[1] = 0x81;
	buf[2] = this->Params->Motor_id;
	buf[3] = 0x00;
	buf[4] = checksum(buf, 4);
}

/*速度闭环控制命令*/
void cMotor::SetSpeedControl(uint8_t* buf)
{
	buf[0] = 0x3E;
	buf[1] = 0xA2;
	buf[2] = this->Params->Motor_id;
	buf[3] = 0x04;
	buf[4] = checksum(buf, 4);
	buf[5] = this->Params->speedcontrol & 0xFF;
	buf[6] = this->Params->speedcontrol>>8;
	buf[7] = this->Params->speedcontrol>>16;
	buf[8] = this->Params->speedcontrol>>24;
	buf[9] = checksum(buf+5, 4);
}

/*多圈闭环控制命令*/
void cMotor::SetMultiAngleControl(uint8_t* buf)
{
	buf[0] = 0x3E;
	buf[1] =  0xA3;
	buf[2] =  this->Params->Motor_id;
	buf[3] =  0x08;
	buf[4] =  checksum(buf, 4);
	buf[5] =  this->Params->motor_angle & 0xFF;
	buf[6] =  this->Params->motor_angle>>8;
	buf[7] =  this->Params->motor_angle>>16;
	buf[8] =  this->Params->motor_angle>>24;
	buf[9] =  this->Params->motor_angle>>32;
	buf[10] = this->Params->motor_angle>>40;
	buf[11] = this->Params->motor_angle>>48;
	buf[12] = this->Params->motor_angle>>56;
	buf[13] = checksum(buf+5, 8);
}

void cMotor::SetInitPla(uint8_t* buf)
{
	buf[0] = 0x3E;
	buf[1] = 0x19;
	buf[2] = this->Params->Motor_id;
	buf[3] = 0x00;
	buf[4] = checksum(buf, 4);
}
	