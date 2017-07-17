//##################################################
//# PROJECT: P1MC MTI.h
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
#ifndef _MTI_
	#define _MTI_
	
	#include <stdint.h>
	#include <port/port.h>

	struct mti_outputconfigration{
	  uint8_t Data_ID[2];
	  uint16_t SampleRate;
	};

	struct mti_EulerAngles{
	  float Roll;
	  float Pitch;
	  float Yaw;
	};
	struct mti_Acceleration{
	  float accX;
	  float accY;
	  float accZ;
	};
	struct mti_RateOfTurn{
	  float gyrX;
	  float gyrY;
	  float gyrZ;
	};
	struct mti_DeltaQ{
	  double q0;
	  double q1;
	  double q2;
	  double q3;
	};
	struct mti_DeltaV{
	  double deltaVx;
	  double deltaVy;
	  double deltaVz;
	};

	struct mti_information{
	  uint8_t Product_Code[21];
	  uint8_t Product_SN[5];
	};

    struct gMti_Data
    {
    	struct mti_EulerAngles     eulerangles;
	    struct mti_Acceleration    acceleration;
	    struct mti_RateOfTurn      rateofturn;
    };	


	class Mti300
	{
	public:
	  uint8_t MTI300_RX_BUFFER[255];

	  uint8_t BaudRate;
      struct mti_outputconfigration OutputConfigration[10];

	  struct mti_EulerAngles     eulerangles;
	  struct mti_Acceleration    acceleration;
	  struct mti_RateOfTurn      rateofturn;
	  struct mti_DeltaQ          deltaQ;
	  struct mti_DeltaV          deltaV;
	  struct mti_information     product_info;
	public:
		Mti300(){};
		~Mti300(){};

		void       gotoConfig(PortHandler *mti300_portHandler);
		void       gotoMeasurement(PortHandler *mti300_portHandler);
		void       get_dev_code(PortHandler *mti300_portHandler);
		void       get_dev_SN(PortHandler *mti300_portHandler);
		void       get_dev_baud(PortHandler *mti300_portHandler);
		void       get_dev_output_config(PortHandler *mti300_portHandler);
		uint8_t    rx_nonblock(PortHandler *mti300_portHandler, uint16_t rx_length);
		uint8_t    rx_nonblock(PortHandler *mti300_portHandler, uint8_t* rx_buffer, uint16_t rx_length);
		float      uint32_to_float(uint8_t* rx_buffer);

		void       recv_data(PortHandler *mti300_portHandler);
		void	   recv_data_auto_recovery(PortHandler *mti300_portHandler);  //add by lichunjing 2017-04-14
        void       print_data(PortHandler *mti300_portHandler);
        void       update_to_gloable_var(gMti_Data *data);
	};






#endif
