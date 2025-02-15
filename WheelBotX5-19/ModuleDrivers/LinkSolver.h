#ifndef 	LINKSOLVER_H
#define		LINKSOLVER_H

#include "main.h"
#include "arm_math.h"
#ifdef __cplusplus

enum eLinkStatue
{
	LINK_NORMAL	= 0,
	LINK_ERROR	= 1
};

class cLinkSolver
{
	protected:
	float JacobianBuf[4]={0};
	float JacobianRevBuf[4]={0};
	float MTRTJ_mat[4] = {0};
	/*Jacobian 矩阵*/
	arm_matrix_instance_f32 MatVMCJ = {2, 2, JacobianBuf};
	arm_matrix_instance_f32 MatVMCJRev = {2, 2, JacobianRevBuf};
	/*单位mm*/
	//腿长
	float L1 = 0.130f; 
	float L2 = 0.270f;
	float MotoDistance = 0.150f;
	float HalfMotoDistance = MotoDistance/2.0f;
	
	/*关节电机弧度*/
	float Theta2 = 0.0f;
	float Theta3 = 0.0f;
	/*关节电机旋转角速度*/
	float w1;
	float w4;
	/* 五连杆从动杆相对于机体角速度 */
	float w2 = 0.0f;
	float w3 = 0.0f;
	/*极限值*/
	float Theta2Min = PI/2;
	float Theta3Max = PI/2;
	/*杆状态*/
	eLinkStatue LinkStatue = LINK_ERROR;
	
	/*倒立摆长度*/
	float PendulumLength = 0.0f;
	/*倒立摆角度*/
	float PendulumRadian = PI/2;
	/*倒立摆坐标*/
	float CoorC[2]={0.0f,0.0f};
	/*第二象限节点坐标*/
	float CoorB[2]={0.0f,0.0f};
	float U2 = 0.0f;
	/*第二象限节点坐标*/
	float CoorD[2]={0.0f,0.0f};
	float U3 = 0.0f;

	
	public:
	void	Resolve(void);
	void	VMCUpdate(void);
	void	VMCCal(float *F, float *T);
	void	VMCRevCal(float *F, float *T);
	void VMCRevCal_Radian(float *R,float *x_dot);
	void VMCVelCal(float *phi_dot,float *x_dot);
	void	SetRadLimit(float Theta3Max, float Theta2Min)
	{
		this->Theta3Max = Theta3Max;
		this->Theta2Min = Theta2Min;
	}
	
	uint8_t	InputLink(float Theta3, float Theta2,float w1,float w4);
	
	inline uint8_t GetLinkStatue(void)
	{return (uint8_t)this->LinkStatue;}
	
	inline float GetPendulumLen(void)
	{return PendulumLength;}
	
	inline float GetPendulumRadian(void)
	{return PendulumRadian;}
	inline float Getw2(void)
	{
	return w2;
	}
	inline float Getw3(void)
	{
	return w3;
	}
	inline void GetPendulumCoor(float* Coor) 
	{Coor[0]=this->CoorC[0];Coor[1]=this->CoorC[1];}
	inline void GetCoorB(float* Coor)
	{Coor[0]=this->CoorB[0];Coor[1]=this->CoorB[1];}
	inline void GetCoorD(float*Coor)
	{Coor[0]=this->CoorD[0];Coor[1]=this->CoorD[1];}
};

#endif
#endif