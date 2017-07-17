//##################################################
//# PROJECT: P1MC POS_TRAJ.h
//# AUTHOR : li chunjing
//##################################################
/*******************************************************************************
* Copyright (c) 2017, UBT CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of UBT nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#ifndef _POS_TRAJ_H_
#define _POS_TRAJ_H_

#include <P1MC.h>

struct Pos
{
	float x;
	float y;
	float z;
};

struct Phi
{
	float phi_x;
	float phi_y;
	float phi_z;
};

struct Trans_matrix
{
	float t[3][3];
};

class Robot_Model
{
public:
	float g;
	float  L0,L1,L2,L3,L3_y,L3_z,L4,L5,L6,L7;
	float  P0,P1,P2;
	float  M0,M1,M2,M4,M_motor_100W,M_motor_200W;
	struct Pos pos_L[9];
	struct Pos pos_P[9];
	struct Pos pos_joint[9];
	struct Pos pos_com[9];
	struct Phi phi_joint[9];
	struct Trans_matrix C[9];
	struct Trans_matrix C_inv[9];

	float  gravity_comp[12];

public:
	Robot_Model();
	~Robot_Model(){};

	void update_phi(dxl_Actuator *dxl_actuator);
	void update_phi();
	void phi_to_matrix(float phi, int n, struct Trans_matrix *_C);
	void vector_plus_vector( struct Pos *pos1, struct Pos *pos2, struct Pos *pos_result);
	void matrix_mul_matrix(struct Trans_matrix *_C1, struct Trans_matrix *_C2, struct Trans_matrix *_C_result);
	void matrix_mul_vector(struct Trans_matrix *_C, struct Pos *pos, struct Pos *pos_result);
	void vector_to_matrix(struct Phi *phi, struct Trans_matrix *_C);
	void vector_to_inv_matrix(struct Phi *phi, struct Trans_matrix *_C);
	void update_trans_matrix();
	void update_inv_trans_matrix();
	void update_joint_position();
	void update_com_position();
	void calculate_gravity_compsation();
	void matrix_test();
};



class Motion_Ctrl
{
public:
	float _time;
	float stand_zero[12]={0,0,0,0,0,0,
							0,0,0,0,0,0};
	//VSW parameter
	float com;
	float p;
	float c;
  	float l;
  	float cankle1;
  	float cankle2;
  	float T;


public:
	Motion_Ctrl(float time);
	~Motion_Ctrl(){};

    void	update_time(float time);
    void	update_ati(gAti_Data *_ati);
    void	update_mti(gMti_Data _mti);
	bool	goto_zero(dxl_Actuator *dxl_actuator);
	void	stand(dxl_Actuator *dxl_actuator, int nNum);     //added by Howard Dong 2017.03.20
	void	read_para();//added by Howard Dong 2017.04.17
	void 	run_vsw(dxl_Actuator *dxl_actuator, int nNum);	 //added by Howard Dong 2017.03.20
	void 	get_vsw_stand(float *_stand, int nNum);	 //added by Howard Dong 2017.03.20
	void	get_inverse_kinematics(float L1, float L2, float X, float Y, float *Theta1, float *Theta2);//added by Howard Dong 2017.03.29
	void	get_forward_kinematics(float L1, float L2, float *X, float *Y, float Theta1, float Theta2);//added by Howard Dong 2017.03.29
	void	run_tra(dxl_Actuator *dxl_actuator, int nNum);//added by Howard Dong 2017.03.29
private:
	gAti_Data	m_gati_data[2];//ATI Force Sensor

	gMti_Data	m_gmti_data;//MTI IMU
};

#endif 
