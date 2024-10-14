#ifndef __CHASSISR_TASK_H
#define __CHASSISR_TASK_H

#include "main.h"
#include "dm4310_drv.h"
#include "pid.h"
#include "VMC_calc.h"
#include "INS_task.h"

#define TP_PID_KP 10.0f
#define TP_PID_KI 0.0f 
#define TP_PID_KD 0.1f
#define TP_PID_MAX_OUT  2.0f
#define TP_PID_MAX_IOUT 0.0f

#define TURN_PID_KP 2.0f
#define TURN_PID_KI 0.0f 
#define TURN_PID_KD 0.2f
#define TURN_PID_MAX_OUT  1.0f//��챵���ĶŤ��
#define TURN_PID_MAX_IOUT 0.0f

#define ROLL_PID_KP 100.0f
#define ROLL_PID_KI 0.0f 
#define ROLL_PID_KD 0.0f
#define ROLL_PID_MAX_OUT  100.0f//��챵���ĶŤ��
#define ROLL_PID_MAX_IOUT 0.0f

#define Mg 13.0f
typedef struct
{
  Joint_Motor_t joint_motor[4];
  Wheel_Motor_t wheel_motor[2];
	
	float v_set;//�����ٶȣ���λ��m/s
	float target_v;
	float x_set;//����λ�ã���λ��m
	float turn_set;//����yaw�ỡ��
	float target_turn;
	float leg_set;//�����ȳ�����λ��m
	float leg_lx_set;
	float target_leg_lx_set;
	float leg_left_set;
	float leg_right_set;
	float last_leg_set;
	float last_leg_left_set;
	float last_leg_right_set;
	float roll_set;
	float roll_target;
	float now_roll_set;

	float v_filter;//�˲���ĳ����ٶȣ���λ��m/s
	float x_filter;//�˲���ĳ���λ�ã���λ��m
	
	float myPithR;
	float myPithGyroR;
	float myPithL;
	float myPithGyroL;
	float roll;
	float total_yaw;
	float theta_err;//���ȼн����
		
	float turn_T;//yaw�Ჹ��
	float leg_tp;//�����油��
	
	uint8_t start_flag;//������־
	
	uint8_t recover_flag;//һ������µĵ��������־
	
	uint32_t count_key;
	uint8_t jump_flag;
	float jump_leg;
	uint32_t jump_time_r;
	uint32_t jump_time_l;
	uint8_t jump_status_r;
	uint8_t jump_status_l;

	
} chassis_t;


extern void ChassisR_init(chassis_t *chassis,vmc_leg_t *vmc,PidTypeDef *legr);
extern void ChassisR_task(void);
extern void Pensation_init(PidTypeDef *Tp,PidTypeDef *turn);
extern void mySaturate(float *in,float min,float max);
extern void chassisR_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins);
extern void chassisR_control_loop(chassis_t *chassis,vmc_leg_t *vmcr,INS_t *ins,float *LQR_K,PidTypeDef *leg);
extern void roll_pid_init(PidTypeDef *roll_pid);
void jump_loop_r(chassis_t *chassis,vmc_leg_t *vmcr,PidTypeDef *leg);
#endif




