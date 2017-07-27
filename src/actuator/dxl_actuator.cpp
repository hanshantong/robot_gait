//##################################################
//# PROJECT: P1MC DXL_ACTUATOR.cpp
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
#include <stdio.h>
#include <actuator/dxl_actuator.h>
#include <math.h>


dxl_Actuator::dxl_Actuator(uint8_t _id)
{
	id = _id;
	comm_result = COMM_TX_FAIL;
	addparam_result = false;
	error = 0;
	flag_theta_InRange = 1;
	flag_delta_theta_InRange=1;

	goal_position = 0;
	goal_theta = 0.0;
	traj_theta = 0.0;
	traj_theta_1 = 0.0;
}

void dxl_Actuator::Torque_Disable(PortHandler *dxl_portHandler, dxl_PacketHandler *dxl_packetHandler)
{

	comm_result = dxl_packetHandler->write1ByteTxRx(dxl_portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &error);
	if (comm_result != COMM_SUCCESS)
	{
		dxl_packetHandler->printTxRxResult(comm_result);
	}
	else if (error != 0)
	{
		dxl_packetHandler->printRxPacketError(error);
	}
}

void dxl_Actuator::Set_Mode_Torque(PortHandler *dxl_portHandler, dxl_PacketHandler *dxl_packetHandler)
{
	Torque_Disable(dxl_portHandler, dxl_packetHandler);
	comm_result = dxl_packetHandler->write1ByteTxRx(dxl_portHandler,id, ADDR_MODE, TORQUE_MODE, &error);
	Torque_Enable(dxl_portHandler, dxl_packetHandler);
	if (comm_result != COMM_SUCCESS)
	{
		dxl_packetHandler->printTxRxResult(comm_result);
	}
	else if (error != 0)
	{
	    dxl_packetHandler->printRxPacketError(error);
	}
	else
	{
	  	if(Mode == POSITION_MODE)
	    	printf("Dynamixel %d has been Position Mode!!! \n",id);
	    else if(Mode == TORQUE_MODE)
	    	printf("Dynamixel %d has been Torque Mode!!! \n",id);
	}
}

void dxl_Actuator::Set_Mode_Position(PortHandler *dxl_portHandler, dxl_PacketHandler *dxl_packetHandler)
{
	Torque_Disable(dxl_portHandler, dxl_packetHandler);
	comm_result = dxl_packetHandler->write1ByteTxRx(dxl_portHandler,id, ADDR_MODE, POSITION_MODE, &error);
	Torque_Enable(dxl_portHandler, dxl_packetHandler);
	if (comm_result != COMM_SUCCESS)
	{
		dxl_packetHandler->printTxRxResult(comm_result);
	}
	else if (error != 0)
	{
	    dxl_packetHandler->printRxPacketError(error);
	}
	else
	{
    	printf("Dynamixel %d has been Position Mode!!! \n",id);
	}
}

void dxl_Actuator::Torque_Enable(PortHandler *dxl_portHandler, dxl_PacketHandler *dxl_packetHandler)
{
	comm_result = dxl_packetHandler->write1ByteTxRx(dxl_portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &error);
	if (comm_result != COMM_SUCCESS)
	{
		dxl_packetHandler->printTxRxResult(comm_result);
	}
	else if (error != 0)
	{
		dxl_packetHandler->printRxPacketError(error);
	}
}
void dxl_Actuator::SyncRead_init(dxl_GroupSyncRead *dxl_groupSyncRead)
{
	comm_result = dxl_groupSyncRead->addParam(id);
	if (comm_result != true)
	{
	  fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", id);
	}
}

void dxl_Actuator::SyncRead_Send(dxl_PacketHandler *dxl_packetHandler, dxl_GroupSyncRead *dxl_groupSyncRead)
{
	comm_result = dxl_groupSyncRead->txRxPacket();
	if(comm_result != COMM_SUCCESS) 
	{
		dxl_packetHandler->printTxRxResult(comm_result);
	}
	else if (error != 0)
	{
		dxl_packetHandler->printRxPacketError(error);
	}

}

void dxl_Actuator::Update_Pos_Vel(dxl_PacketHandler *dxl_packetHandler, dxl_GroupSyncRead *dxl_groupSyncRead)
{
	
	 // Check if groupsyncread data of Dynamixel is available
	 getdata_result = dxl_groupSyncRead->isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
	 if (getdata_result != true)
	 {
	   fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", id);
	 }
	 else
	 // Get Dynamixel#1 present position value
	 present_position = dxl_groupSyncRead->getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);

	 // Check if groupsyncread data of Dynamixel is available
	 getdata_result = dxl_groupSyncRead->isAvailable(id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_POSITION);
	 if (getdata_result != true)
	 {
	   fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", id);
	 }
	 else
	 // Get Dynamixel#1 present position value
	 present_velocity= dxl_groupSyncRead->getData(id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_POSITION);

	 if((present_velocity > 16613.0)||(present_velocity < -16613.0))
     {
       present_velocity  = previous_velocity;   //
     }

	 if((present_position > 250960)||(present_position < -250960))
     {
        present_position = previous_position +present_velocity / GEAR_RATIO * R_PER_MIN_TO_DEGREE_PER_SECOND_SF * CTRL_PERIOD_S * DEGREE_TO_REG_VALUE_SF;
     }

	 present_theta = present_position * REG_VALUE_TO_DEGREE_TO_SF;
	 previous_theta = previous_position * REG_VALUE_TO_DEGREE_TO_SF;

     previous_position = present_position;
     previous_velocity = present_velocity;	
}

void dxl_Actuator::SyncWrite_init(dxl_GroupSyncWrite *dxl_groupSyncWrite)
{
	uint8_t param_goal_torque[2];
	param_goal_torque[0] = DXL_LOBYTE(DXL_LOWORD(goal_torque));
    param_goal_torque[1] = DXL_HIBYTE(DXL_LOWORD(goal_torque));

    // Add goal torque value to the Syncwrite storage
	    addparam_result = dxl_groupSyncWrite->addParam(id, param_goal_torque);
    if (addparam_result != true)
    {
    	fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", id);
    }
}

void dxl_Actuator::SyncWrite_Send_Goal_Torque(dxl_GroupSyncWrite *dxl_groupSyncWrite, dxl_PacketHandler *dxl_packetHandler)
{
	comm_result = dxl_groupSyncWrite->txPacket();
   if(comm_result != COMM_SUCCESS) 
   	  dxl_packetHandler->printTxRxResult(comm_result);

   // Clear syncwrite parameter storage
   dxl_groupSyncWrite->clearParam();
}

void dxl_Actuator::SyncWrite_Set_Position(dxl_GroupSyncWrite *dxl_groupSyncWrite, dxl_PacketHandler *dxl_packetHandler)
{
	uint8_t param_goal_position[4];
	param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal_position));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal_position));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal_position));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal_position));

    // Add goal torque value to the Syncwrite storage
	    addparam_result = dxl_groupSyncWrite->addParam(id, param_goal_position);
    if (addparam_result != true)
    {
    	fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", id);
    }	
}

void dxl_Actuator::SyncWrite_Send_Goal_Position(dxl_GroupSyncWrite *dxl_groupSyncWrite, dxl_PacketHandler *dxl_packetHandler)
{
	comm_result = dxl_groupSyncWrite->txPacket();
   if(comm_result != COMM_SUCCESS) 
   	  dxl_packetHandler->printTxRxResult(comm_result);

   // Clear syncwrite parameter storage
   dxl_groupSyncWrite->clearParam();
}

void dxl_Actuator::set_goal_theta(float theta)
{
	goal_theta = theta*nDir;
	goal_position = goal_theta * DEGREE_TO_REG_VALUE_SF;
}

void dxl_Actuator::set_goal_position(int32_t position)
{
	goal_position = position;
}

float dxl_Actuator::get_start_theta()
{
	return start_theta;
}

void dxl_Actuator::set_theta_range(float min, float max) //added lichunjing 2017-05-27	
{
	theta_min = min;
	theta_max = max;
}

void dxl_Actuator::check_theta_range()  //added lichunjing 2017-05-27
{
	if(get_present_theta() > theta_max) flag_theta_InRange= 0;
	if(get_present_theta() < theta_min) flag_theta_InRange= 0;
}

void dxl_Actuator::set_delta_theta_range(float range) //added lichunjing 2017-05-27	
{
	delta_theta_range = range;
}

void dxl_Actuator::check_delta_theta_range()  //added lichunjing 2017-05-27
{
	if(fabs(get_present_theta() - get_previous_theta())> delta_theta_range) 
		flag_delta_theta_InRange = 0;
}

float dxl_Actuator::get_present_theta()
{
	return present_theta*nDir;
}

float dxl_Actuator::get_previous_theta()
{
	return previous_theta*nDir;
}

float dxl_Actuator::get_present_velocity()
{
	return present_velocity*nDir;
}

float dxl_Actuator::get_goal_theta()
{
	return goal_theta*nDir;
}

void dxl_Actuator::SetID(uint8_t ID)
{
	id=ID;
}

void dxl_Actuator::SetZeroPos(float fz)
{
	zero_pos = fz;
}
void dxl_Actuator::SetDir(int nd)
{
	nDir = nd;
}

void  dxl_Actuator::goto_theta_mode_position_50w(PortHandler *dxl_portHandler, dxl_PacketHandler *dxl_packetHandler, float theta)
{
	int positon = (int)(theta / DXL_PRO_20_UNIT);
	comm_result = dxl_packetHandler->write4ByteTxRx(dxl_portHandler, id, ADDR_GOAL_POSITION, positon, &error);
	if (comm_result != COMM_SUCCESS)
	{
		dxl_packetHandler->printTxRxResult(comm_result);
	}
	else if (error != 0)
	{
		dxl_packetHandler->printRxPacketError(error);
	}
}

void  dxl_Actuator::goto_theta_mode_position_200w(PortHandler *dxl_portHandler, dxl_PacketHandler *dxl_packetHandler, float theta)
{
	int positon = (int)(theta / REG_VALUE_TO_DEGREE_TO_SF);
	comm_result = dxl_packetHandler->write4ByteTxRx(dxl_portHandler, id, ADDR_GOAL_POSITION, positon, &error);
	if (comm_result != COMM_SUCCESS)
	{
		dxl_packetHandler->printTxRxResult(comm_result);
	}
	else if (error != 0)
	{
		dxl_packetHandler->printRxPacketError(error);
	}
}
