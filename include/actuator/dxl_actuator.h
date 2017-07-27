//##################################################
//# PROJECT: P1MC DXL_ACTUATOR.h
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
#ifndef _DXL_ACTUATOR_H_
#define _DXL_ACTUATOR_H_

#include <port/port.h>
#include <port/dxl_packet.h>

#define GEAR_RATIO 501.9
#define DEGREE_TO_REG_VALUE_SF (250961.5 / 180.0)
#define REG_VALUE_TO_DEGREE_TO_SF (180.0 / 250961.5)
#define R_PER_MIN_TO_DEGREE_PER_SECOND_SF (360 / 60.0)
#define CTRL_PERIOD_S   0.005
#define DXL_PRO_20_UNIT                 (180./151875)
class dxl_Actuator
{
	public:
		uint8_t id;
		uint8_t Mode;

		int32_t present_position;
	    int32_t present_velocity;

	    int32_t previous_position;
	    int32_t previous_velocity;

	    int32_t goal_position;
	    float   goal_theta;

	    float   present_theta;
	    float   previous_theta;
	    float   start_theta;

	    float   theta_min;
	    float   theta_max;
	    bool    flag_theta_InRange;

	    float   delta_theta_range;
	    bool    flag_delta_theta_InRange;

	    int32_t goal_torque;

	    int     comm_result;
	    int     getdata_result;
	    bool    addparam_result;
	    uint8_t error;

	    float zero_pos;//calibrated zero pos
	    int nDir;//rotation dir

	public:
	    float   traj_theta;   //for static variable use
	    float   traj_theta_1;

	public:
		dxl_Actuator(uint8_t _id);
		dxl_Actuator(){flag_theta_InRange = 1;flag_delta_theta_InRange=1;};
		~dxl_Actuator(){};

		void Torque_Disable(PortHandler *dxl_portHandler, dxl_PacketHandler *dxl_packetHandler);
		void Torque_Enable(PortHandler *dxl_portHandler, dxl_PacketHandler *dxl_packetHandler);
		void Set_Mode_Torque(PortHandler *dxl_portHandler, dxl_PacketHandler *dxl_packetHandler);
		void Set_Mode_Position(PortHandler *dxl_portHandler, dxl_PacketHandler *dxl_packetHandler);  //added lichunjing 2017-03-25		
		void SyncRead_init(dxl_GroupSyncRead *dxl_groupSyncRead);
		void SyncRead_Send(dxl_PacketHandler *dxl_packetHandler, dxl_GroupSyncRead *dxl_groupSyncRead);
		void Update_Pos_Vel(dxl_PacketHandler *dxl_packetHandler, dxl_GroupSyncRead *dxl_groupSyncRead);
		void SyncWrite_init(dxl_GroupSyncWrite *dxl_groupSyncWrite);
		void SyncWrite_Send_Goal_Torque(dxl_GroupSyncWrite *dxl_groupSyncWrite, dxl_PacketHandler *dxl_packetHandler);

		void SyncWrite_Set_Position(dxl_GroupSyncWrite *dxl_groupSyncWrite, dxl_PacketHandler *dxl_packetHandler);
		void SyncWrite_Send_Goal_Position(dxl_GroupSyncWrite *dxl_groupSyncWrite, dxl_PacketHandler *dxl_packetHandler);		

		void set_goal_theta(float theta);
		void set_goal_position(int32_t position);

		void set_theta_range(float min, float max);  //added lichunjing 2017-05-27	
		void check_theta_range();  //added lichunjing 2017-05-27
		void set_delta_theta_range(float range);  //added lichunjing 2017-05-27	
		void check_delta_theta_range();  //added lichunjing 2017-05-27

		float get_start_theta();
		float get_present_theta();
		float get_previous_theta();
		float get_goal_theta();
		float get_present_velocity();
		void  SetID(uint8_t ID);
		void  SetZeroPos(float fz);
		void  SetDir(int nd);

		void  goto_theta_mode_position_50w(PortHandler *dxl_portHandler, dxl_PacketHandler *dxl_packetHandler, float theta);
		void  goto_theta_mode_position_200w(PortHandler *dxl_portHandler, dxl_PacketHandler *dxl_packetHandler, float theta);
};


#endif