//##################################################
//# PROJECT: P1MC ATI.CPP
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
#include <sensor/ati.h>
#include <stdio.h>
#include <pthread.h>
#include <string.h>
#include <math.h>


#define ATI_SERVER_ID                 10

/* Modbus Function codes */
#define _FC_READ_COILS                0x01
#define _FC_READ_DISCRETE_INPUTS      0x02
#define _FC_READ_HOLDING_REGISTERS    0x03
#define _FC_READ_INPUT_REGISTERS      0x04
#define _FC_WRITE_SINGLE_COIL         0x05
#define _FC_WRITE_SINGLE_REGISTER     0x06
#define _FC_READ_EXCEPTION_STATUS     0x07
#define _FC_WRITE_MULTIPLE_COILS      0x0F
#define _FC_WRITE_MULTIPLE_REGISTERS  0x10
#define _FC_REPORT_SLAVE_ID           0x11
#define _FC_WRITE_AND_READ_REGISTERS  0x17

/* Modbus Address codes */
const uint16_t ATI_GAGE_REG_ADDRESS = 0x0000;
const uint16_t ATI_CALI_REG_ADDRESS = 0x00E3;
const uint16_t ATI_CALI_STEP        = 0x00C0;
const uint16_t ATI_CALI_LEN         = 0x00A9;
uint16_t START_ADDRESS              = 0x0000;
const uint16_t ATI_CALI_NUM_1       = 0x0068;
const uint16_t ATI_CALI_NUM_2       = 0x0041;
const uint16_t ATI_GAGE_NUM         = 0x000C;
const uint16_t ATI_GAIN_NUM         = 0x0006;
const uint16_t ATI_BAUD_ADDRESS     = 0x001F;
const uint16_t ATI_MODE_ADDRESS     = 0x001E;
           // static const uint8_t table_crc_hi[]
           // static const uint8_t table_crc_lo[]
           // static int _modbus_set_slave(modbus_t *ctx, int slave)

/* Table of CRC values for high-order byte */
static const uint8_t table_crc_hi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

/* Table of CRC values for low-order byte */
static const uint8_t table_crc_lo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

uint16_t crc16(uint8_t *buffer, uint16_t buffer_length)
{
    uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
    uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
    unsigned int i; /* will index into CRC lookup */

    /* pass through message buffer */
    while (buffer_length--) {
        i = crc_hi ^ *buffer++; /* calculate the CRC  */
        crc_hi = crc_lo ^ table_crc_hi[i];
        crc_lo = table_crc_lo[i];
    }

    return (crc_hi << 8 | crc_lo);
}

/* Modbus instructions codes */
uint8_t ATI_READ_CALIBRARION_1[] =      {ATI_SERVER_ID, _FC_READ_HOLDING_REGISTERS, DXL_HIBYTE(ATI_CALI_REG_ADDRESS), DXL_LOBYTE(ATI_CALI_REG_ADDRESS), DXL_HIBYTE(ATI_CALI_NUM_1), DXL_LOBYTE(ATI_CALI_NUM_1), 0xB4, 0xA9};
uint8_t ATI_READ_CALIBRARION_2[] =      {ATI_SERVER_ID, _FC_READ_HOLDING_REGISTERS, DXL_HIBYTE(ATI_CALI_REG_ADDRESS+ATI_CALI_NUM_1), DXL_LOBYTE(ATI_CALI_REG_ADDRESS+ATI_CALI_NUM_1), DXL_HIBYTE(ATI_CALI_NUM_2), DXL_LOBYTE(ATI_CALI_NUM_2), 0xF5, 0x6B};
uint8_t ATI_READ_GAGE[]          =      {ATI_SERVER_ID, _FC_READ_HOLDING_REGISTERS, DXL_HIBYTE(ATI_GAGE_REG_ADDRESS), DXL_LOBYTE(ATI_GAGE_REG_ADDRESS), DXL_HIBYTE(ATI_GAGE_NUM), DXL_LOBYTE(ATI_GAGE_NUM), 0x44, 0xB4};
uint8_t ATI_UNLOCK_GAGE_REG[]    =      {ATI_SERVER_ID, 0x6A, 0xAA, 0xFF, 0x1D};
uint8_t ATI_LOCK_GAGE_REG[]      =      {ATI_SERVER_ID, 0x6A, 0x18, 0x7F, 0x68};
uint8_t ATI_WRITE_GAGE_REG[ATI_GAGE_NUM*2+9]     =      {ATI_SERVER_ID, _FC_WRITE_MULTIPLE_REGISTERS, DXL_HIBYTE(ATI_GAGE_REG_ADDRESS), DXL_LOBYTE(ATI_GAGE_REG_ADDRESS), DXL_HIBYTE(ATI_GAGE_NUM), DXL_LOBYTE(ATI_GAGE_NUM), 2*DXL_LOBYTE(ATI_GAGE_NUM)};
uint8_t START_STREAMING[]        = {ATI_SERVER_ID,0x46,0x55,0xA3,0x9D};
uint8_t STOP_STREAMING[]         = {ATI_SERVER_ID,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

/* Modbus instructions length codes */
const uint16_t ATI_RX_CALI_LEN_1 = ATI_CALI_NUM_1 *2 + 5;
const uint16_t ATI_RX_CALI_LEN_2 = ATI_CALI_NUM_2 *2 + 5;
const uint16_t ATI_RX_GAGE_LEN   = ATI_GAGE_NUM *2 + 5;
const uint16_t ATI_UNLOCK_STORAGE_LEN   = 5;
const uint16_t ATI_LOCK_STORAGE_LEN     = 5;
const uint16_t ATI_WRITE_GAGE_REG_LEN   = 8;
const uint16_t ATI_START_STREAMING_LEN  = 5;
const uint16_t ATI_STREAMING_DATA_LEN   = 13;


Ati_Mini45::Ati_Mini45()
{
	print_counter = 0; 
	check_error = 0; 
	SuccessRead = 0;
	check_error_counter = 0;

	for(uint16_t i=0;i<6;i++)
	{
		FT_data[i] = 0.0;
	}

	for(uint16_t i=0;i<6;i++)
	{
		FT_bais[i] = 0.0;
	}

	for(uint16_t i=0;i<6;i++)
	{
		gages[i] = 0;
	}

	for(uint16_t i=0;i<BIAS_NUM;i++)
	{
		for(uint16_t j=0;j<6;j++)
		{
			FT_bais_matrix[i][j] = 0.0;
		}
	}


}

Ati_Mini45::Ati_Mini45(struct Calibration ati_cali)
{
	Calibration_info = ati_cali;
	print_counter = 0; 
	check_error = 0; 
	SuccessRead = 0;
	check_error_counter = 0;

	flag_SN_right = 1;
	flag_PN_right = 1;
	flag_FID_right = 1;
	flag_FU_right = 1;
	flag_TU_right = 1;

	for(uint16_t i=0;i<6;i++)
	{
		FT_data[i] = 0.0;
	}

	for(uint16_t i=0;i<6;i++)
	{
		FT_bais[i] = 0.0;
	}

	for(uint16_t i=0;i<6;i++)
	{
		gages[i] = 0;
	}

	for(uint16_t i=0;i<BIAS_NUM;i++)
	{
		for(uint16_t j=0;j<6;j++)
		{
			FT_bais_matrix[i][j] = 0.0;
		}
	}


}

void Ati_Mini45::read_calibration(PortHandler *ati_portHandler)
{
	  // uint8_t RX_BUFFER[255];
	  int16_t connect_counter=100;
	  uint8_t  Rx_result = 0;

	 while(connect_counter--)
	 {
		  ati_portHandler->clearPort();
		  ati_portHandler->writePort(ATI_READ_CALIBRARION_1,sizeof(ATI_READ_CALIBRARION_1));
		  ati_portHandler->setPacketTimeout(0.0008*11*(ATI_RX_CALI_LEN_1+1000));  //timeout is set 5 greater than deserved received length
		  Rx_result = rx_nonblock(ati_portHandler, RX_BUFFER, ATI_RX_CALI_LEN_1);
		  if(Rx_result)
		  {
		  	   break;
		  }
		  else
		  {
		  	   if(connect_counter <= 0)
		       {
		 	      printf("ATI Connect failed after %d times!\n",100-connect_counter);
		 	      printf("try again!\n");
		 	      ati_portHandler->closePort();
		 	      pthread_exit(0);          
		       }
		  }
	 }
	 Check_Cali_1(RX_BUFFER);

	 connect_counter=100;
	 while(connect_counter--)
	 {
	  ati_portHandler->clearPort();
	  ati_portHandler->writePort(ATI_READ_CALIBRARION_2,sizeof(ATI_READ_CALIBRARION_2));
	  ati_portHandler->setPacketTimeout(0.0008*11*(ATI_RX_CALI_LEN_2+1000));  //timeout is set 5 greater than deserved received length
	  Rx_result = rx_nonblock(ati_portHandler, RX_BUFFER, ATI_RX_CALI_LEN_2);
	  if(Rx_result)
	  {
	  	   break;
	  }
	  else
	  {
	  	   if(connect_counter <= 0)
	       {
	 	      printf("ATI Connect failed after %d times!\n",100-connect_counter);
	 	      printf("try again!\n");
	 	      ati_portHandler->closePort();
	 	      pthread_exit(0);          
	       }
	  }
	 }
	  Check_Cali_2(RX_BUFFER);

	  for(uint16_t i=0;i<sizeof(Calibration_ONE.CalibSerialNumber);i++)
	  {
	  	if(Calibration_ONE.CalibSerialNumber[i] != Calibration_info.CalibSerialNumber[i])
	  		flag_SN_right = 0;
	  }

  	  for(uint16_t i=0;i<12;i++)
	  {
	  	if(Calibration_ONE.CalibPartNumber[i] != Calibration_info.CalibPartNumber[i])
	  		flag_PN_right = 0;
	  }

	  for(uint16_t i=0;i<sizeof(Calibration_ONE.CalibFamilyId);i++)
	  {
	  	if(Calibration_ONE.CalibFamilyId[i] != Calibration_info.CalibFamilyId[i])
	  		flag_FID_right = 0;
	  }

	  if(Calibration_ONE.ForceUnits != Calibration_info.ForceUnits)
	  	flag_FU_right = 0;

	  if(Calibration_ONE.TorqueUnits != Calibration_info.TorqueUnits)
	  	flag_TU_right = 0;


}

void Ati_Mini45::read_gage_offset(PortHandler *ati_portHandler)
{
 	 // uint8_t RX_BUFFER[255];
	 int16_t connect_counter=100;
	 uint8_t  Rx_result = 0;
	 while(connect_counter--)
	 {
	  ati_portHandler->clearPort();
	  ati_portHandler->writePort(ATI_READ_GAGE,sizeof(ATI_READ_GAGE));
	  ati_portHandler->setPacketTimeout(0.0008*11*(ATI_RX_CALI_LEN_2+1000));
	  Rx_result = rx_nonblock(ati_portHandler, RX_BUFFER, ATI_RX_GAGE_LEN);
	if(Rx_result)
	  {
	  	   break;
	  }
	  else
	  {
	  	   if(connect_counter <= 0)
	       {
	 	      printf("ATI Connect failed after %d times!\n",100-connect_counter);
	 	      printf("try again!\n");
	 	      ati_portHandler->closePort();
	 	      pthread_exit(0);          
	       }
	  }
	 }

	 Check_Gain_offset(RX_BUFFER);
}

void  Ati_Mini45::write_gage(PortHandler *ati_portHandler)
{
	  // uint8_t RX_BUFFER[255];
	  bool unlock_succeeded = Unlock_storage(ati_portHandler);
	  if(unlock_succeeded)
	  {
	     printf("ATI Unlock Gage gain and offset succeeded!\n");
	     for(uint16_t i=0;i<6;i++)
	     {
	     	ATI_WRITE_GAGE_REG[i*2+7] = DXL_HIBYTE(Calibration_ONE.GageGain[i]);
	     	ATI_WRITE_GAGE_REG[i*2+1+7] = DXL_LOBYTE(Calibration_ONE.GageGain[i]);
	     }
	     for(uint16_t i=0;i<6;i++)
	     {
	     	ATI_WRITE_GAGE_REG[i*2+7+12] = DXL_HIBYTE(Calibration_ONE.GageOffset[i]);
	     	ATI_WRITE_GAGE_REG[i*2+1+7+12] = DXL_LOBYTE(Calibration_ONE.GageOffset[i]);
	     }
	     // ATI_WRITE_GAGE_REG[31] = 0x45;
	     // ATI_WRITE_GAGE_REG[32] = 0x41;
 				uint16_t crc_cal;
                crc_cal = crc16(ATI_WRITE_GAGE_REG,sizeof(ATI_WRITE_GAGE_REG)-2);
                ATI_WRITE_GAGE_REG[31] = (crc_cal & 0xFF00)>>8;
                ATI_WRITE_GAGE_REG[32] = crc_cal & 0xFF;


	     for(int i=0;i<10;i++)
	     {
	     	 ati_portHandler->writePort(ATI_WRITE_GAGE_REG,sizeof(ATI_WRITE_GAGE_REG));
		     rx_nonblock(ati_portHandler, RX_BUFFER, ATI_WRITE_GAGE_REG_LEN);

		     uint16_t rev_written_len = (uint16_t)DXL_MAKEWORD(RX_BUFFER[5],RX_BUFFER[4]);

		     if(rev_written_len == ATI_GAGE_NUM)
		     {
		        printf("ATI Write Gage gain and offset succeeded after %d times!\n",i);
		        break;
		     }
		     else
		     {
		        printf("ATI Write Gage gain and offset failed!\n");
		     }
	     }

	     
	  }
	  else
	  {
	      printf("ATI Unlock Gage gain and offset failed!\n");
	  }

	  bool lock_succeeded = Lock_storage(ati_portHandler);
	  if(lock_succeeded)
	  {
		  printf("ATI lock Gage gain and offset succeeded!\n");
	  } 
	  else
	  {
		  printf("ATI lock Gage gain and offset failed!\n");
	  }
}

void Ati_Mini45::start_stream(PortHandler *ati_portHandler)
{
 	 // uint8_t RX_BUFFER[255];
	 int16_t connect_counter=100;
	 uint8_t  Rx_result = 0;
	 while(connect_counter--)
	 {
		ati_portHandler->clearPort(); 
		ati_portHandler->writePort(START_STREAMING,sizeof(START_STREAMING));
		ati_portHandler->setPacketTimeout(0.0008*11*(ATI_START_STREAMING_LEN+500));
		Rx_result = rx_nonblock(ati_portHandler, RX_BUFFER, ATI_START_STREAMING_LEN);
		if((Rx_result == 1)&&(RX_BUFFER[0] == 0x0a)&&(RX_BUFFER[1] == 0x46)&&(RX_BUFFER[2] == 0x01)&&(RX_BUFFER[3] == 0xa2)&&(RX_BUFFER[4] == 0x62))
		  {
		  	   printf("ATI start stream succeeded!\n");
		  	   break;
		  }
		  else
		  {
		  	   if(connect_counter <= 0)
		       {
		 	      printf("ATI Connect failed after %d times!\n",100-connect_counter);
		 	      printf("try again!\n");
		 	      ati_portHandler->closePort();
		 	      pthread_exit(0);          
		       }
		  }
	 }
}

void  Ati_Mini45::stop_stream(PortHandler *ati_portHandler)
{
	ati_portHandler->writePort(STOP_STREAMING,sizeof(STOP_STREAMING));
}

bool Ati_Mini45::Unlock_storage(PortHandler *ati_portHandler)
{
 	 // uint8_t RX_BUFFER[255];
	 uint16_t connect_counter=100;
	 uint8_t  Rx_result = 0;
	 while(connect_counter--)
	 {
	  ati_portHandler->clearPort();  
	  ati_portHandler->writePort(ATI_UNLOCK_GAGE_REG,sizeof(ATI_UNLOCK_GAGE_REG));
	  ati_portHandler->setPacketTimeout(0.0008*11*(ATI_RX_CALI_LEN_2+1000));
	  Rx_result = rx_nonblock(ati_portHandler, RX_BUFFER, ATI_UNLOCK_STORAGE_LEN);
	if(Rx_result)
	  {
	  	   break;
	  }
	  else
	  {
	  	   if(connect_counter <= 0)
	       {
	 	      printf("ATI Connect failed after %d times!\n",100-connect_counter);
	 	      printf("try again!\n");
	 	      ati_portHandler->closePort();
	 	      pthread_exit(0);          
	       }
	  }
	 }

	 return (bool)RX_BUFFER[2];
}

bool Ati_Mini45::Lock_storage(PortHandler *ati_portHandler)
{
	 // uint8_t RX_BUFFER[255];
	 uint16_t connect_counter=100;
	 uint8_t  Rx_result = 0;
	 while(connect_counter--)
	 {
		ati_portHandler->clearPort(); 
		ati_portHandler->writePort(ATI_LOCK_GAGE_REG,sizeof(ATI_LOCK_GAGE_REG));
		ati_portHandler->setPacketTimeout(0.0008*11*(ATI_RX_CALI_LEN_2+1000));
		Rx_result = rx_nonblock(ati_portHandler, RX_BUFFER, ATI_LOCK_STORAGE_LEN);
		if(Rx_result)
		  {
		  	   break;
		  }
		  else
		  {
		  	   if(connect_counter <= 0)
		       {
		 	      printf("ATI Connect failed after %d times!\n",100-connect_counter);
		 	      printf("try again!\n");
		 	      ati_portHandler->closePort();
		 	      pthread_exit(0);          
		       }
		  }
	 }
	 return (bool)RX_BUFFER[2];
}

void Ati_Mini45::update_ft_data(PortHandler *ati_portHandler)
{
	print_counter++;
	ati_portHandler->readPort(RX_BUFFER, ATI_STREAMING_DATA_LEN);
	//ATI_RX_DATA(ati_portHandler, read_buffer, ATI_STREAMING_DATA_LEN);
    uint8_t CheckSum = 0;
    for(uint16_t i = 0;i<12;i++)
    {
      CheckSum = CheckSum + RX_BUFFER[i];
        //printf("%2x ",read_buffer[i] );
    }
    if((RX_BUFFER[12] & 0x80) == 0x80)
    {
      check_error = 1;
      //printf("STATUS ERROR\n" );
    }
    else if( (CheckSum & 0x7f) != (RX_BUFFER[12]& 0x7f))
    {
      check_error = 1;
      //printf("CheckSum = %2x\nCheckSum received = %2x\n",CheckSum,read_buffer[12] );
    }
    else
    {
     // printf("CheckSum OK\n");
      gages[0] = (int)(((uint16_t)RX_BUFFER[0]<<8) + (uint16_t)RX_BUFFER[1]) ;
      gages[2] = (int)(((uint16_t)RX_BUFFER[2]<<8) + (uint16_t)RX_BUFFER[3]) ;
      gages[4] = (int)(((uint16_t)RX_BUFFER[4]<<8) + (uint16_t)RX_BUFFER[5]) ;
      gages[1] = (int)(((uint16_t)RX_BUFFER[6]<<8) + (uint16_t)RX_BUFFER[7]) ;
      gages[3] = (int)(((uint16_t)RX_BUFFER[8]<<8) + (uint16_t)RX_BUFFER[9]) ;
      gages[5] = (int)(((uint16_t)RX_BUFFER[10]<<8) + (uint16_t)RX_BUFFER[11]) ;
      for(uint16_t i = 0;i<6;i++)
      {
        if(gages[i] > 32767)
        {
          gages[i] = gages[i] - 65536;  //Change it to int16
        }
        //printf("%d ", ATI_GAGES[i]);
      }

     // printf("\n" );

      for(uint16_t i = 0;i<6;i++)
      {
        FT_data[i] = 0.0;
        for(uint16_t j = 0;j<6;j++)
        {
          FT_data[i] = FT_data[i] + Calibration_ONE.BasicMatrix[i][j] * (float)gages[j];
        }
        if(i < 3)
        {
          FT_data[i] = FT_data[i] / Calibration_ONE.CountsPerForce;
            //printf("%f\t",ATI_FT_DATA[i] / Calibration_ONE.CountsPerForce );
        }
        else
        {
          FT_data[i] = FT_data[i] / Calibration_ONE.CountsPerTorque;
            //printf("%f\t",ATI_FT_DATA[i] / Calibration_ONE.CountsPerTorque );
        }
      }
     // printf("\n");

      if(SuccessRead < BIAS_NUM)
      {
        for(int i = 0;i<6;i++)
        {
          FT_bais_matrix[SuccessRead][i] = FT_data[i];
        }
      }
      else if(SuccessRead == BIAS_NUM)
      {
        for(uint16_t i = 0;i<6;i++)
        {
          float sum = 0;
          for(uint16_t j = 0;j<BIAS_NUM;j++)
          {
            sum = sum + FT_bais_matrix[j][i];
          }
          FT_bais[i] = (float) sum / BIAS_NUM;
        }
        printf("====================\nCalibration end \n====================\n");
        printf("FT bias: %f, %f, %f, %f, %f, %f\n", FT_bais[0],FT_bais[1],FT_bais[2],FT_bais[3],FT_bais[4],FT_bais[5]);
      }
      else{
        // for(int i = 0;i<6;i++)
        // {
        //     printf("%f\t",ATI_FT_DATA[i] - ATI_FT_bais[i]);
        // }
      }



      SuccessRead = SuccessRead +1;
    }
}

void Ati_Mini45::update_ft_data_restart_stream_mode(PortHandler *ati_portHandler)
{

	print_counter++;                  //a better way: time out mode read.

	ati_portHandler->readPort(RX_BUFFER, ATI_STREAMING_DATA_LEN);
	//ATI_RX_DATA(ati_portHandler, read_buffer, ATI_STREAMING_DATA_LEN);

    uint8_t CheckSum = 0;
    for(uint16_t i = 0;i<12;i++)
    {
      CheckSum = CheckSum + RX_BUFFER[i];
        //printf("%2x ",read_buffer[i] );
    }
    if((RX_BUFFER[12] & 0x80) == 0x80)
    {
      check_error_counter++;
      check_error = 1;
    }
    else if( (CheckSum & 0x7f) != (RX_BUFFER[12]& 0x7f))
    {
      check_error_counter++;
      check_error = 1;
    }
    else
    {
     // printf("CheckSum OK\n");
      gages[0] = (int)(((uint16_t)RX_BUFFER[0]<<8) + (uint16_t)RX_BUFFER[1]) ;
      gages[2] = (int)(((uint16_t)RX_BUFFER[2]<<8) + (uint16_t)RX_BUFFER[3]) ;
      gages[4] = (int)(((uint16_t)RX_BUFFER[4]<<8) + (uint16_t)RX_BUFFER[5]) ;
      gages[1] = (int)(((uint16_t)RX_BUFFER[6]<<8) + (uint16_t)RX_BUFFER[7]) ;
      gages[3] = (int)(((uint16_t)RX_BUFFER[8]<<8) + (uint16_t)RX_BUFFER[9]) ;
      gages[5] = (int)(((uint16_t)RX_BUFFER[10]<<8) + (uint16_t)RX_BUFFER[11]) ;
      for(uint16_t i = 0;i<6;i++)
      {
        if(gages[i] > 32767)
        {
          gages[i] = gages[i] - 65536;  //Change it to int16
        }
        //printf("%d ", ATI_GAGES[i]);
      }

     // printf("\n" );

      for(uint16_t i = 0;i<6;i++)
      {
        FT_data[i] = 0.0;
        for(uint16_t j = 0;j<6;j++)
        {
          FT_data[i] = FT_data[i] + Calibration_ONE.BasicMatrix[i][j] * (float)gages[j];
        }
        if(i < 3)
        {
          FT_data[i] = FT_data[i] / Calibration_ONE.CountsPerForce;
            //printf("%f\t",ATI_FT_DATA[i] / Calibration_ONE.CountsPerForce );
        }
        else
        {
          FT_data[i] = FT_data[i] / Calibration_ONE.CountsPerTorque;
            //printf("%f\t",ATI_FT_DATA[i] / Calibration_ONE.CountsPerTorque );
        }
      }
     // printf("\n");

      if(SuccessRead < BIAS_NUM)
      {
        for(int i = 0;i<6;i++)
        {
          FT_bais_matrix[SuccessRead][i] = FT_data[i];
        }
      }
      else if(SuccessRead == BIAS_NUM)
      {
        for(uint16_t i = 0;i<6;i++)
        {
          float sum = 0;
          for(uint16_t j = 0;j<BIAS_NUM;j++)
          {
            sum = sum + FT_bais_matrix[j][i];
          }
          FT_bais[i] =  (float) sum / BIAS_NUM;
        }
                printf("====================\nCalibration end \n====================\n");
				printf("FT bias: %f, %f, %f, %f, %f, %f\n", FT_bais[0],FT_bais[1],FT_bais[2],FT_bais[3],FT_bais[4],FT_bais[5]);
      }
      else{

        // for(int i = 0;i<6;i++)
        // {
        //     printf("%f\t",ATI_FT_DATA[i] - ATI_FT_bais[i]);
        // }
      }
      SuccessRead = SuccessRead +1;
    }
}

void Ati_Mini45::print_ft()
{
	if(print_counter >= 100)
	    {
	    	if(check_error == 1)
	    	{
	    		printf("STATUS ERROR\n" );
	    		check_error = 0;
	    	}
	    	else
	    	{
	    		
	    		printf("Fx: %f\t",FT_data[0] - FT_bais[0]);
	    		printf("Fy: %f\t",FT_data[1] - FT_bais[1]);
	    		printf("Fz: %f\t",FT_data[2] - FT_bais[2]);
	    		printf("Tx: %f\t",FT_data[3] - FT_bais[3]);
	    		printf("Ty: %f\t",FT_data[4] - FT_bais[4]);
	    		printf("Tz: %f\t",FT_data[5] - FT_bais[5]);
	            printf("\n" );
	    	}
	    	print_counter = 0;
	    }
}

uint8_t Ati_Mini45::rx_nonblock(PortHandler *ati_portHandler, uint8_t* rx_buffer, uint16_t rx_length)
{
	 uint8_t  Rx_result = 0;
	 uint16_t rx_counter = 0;
	 while(1)
	   {
		    rx_counter += ati_portHandler->readPort(&rx_buffer[rx_counter], rx_length - rx_counter);
		    if (rx_counter >= rx_length)
		    {
	    		Rx_result = 1;
	    		break;
		    }
		    if(ati_portHandler->isPacketTimeout())
		    {
	    	    Rx_result = 0;
	    		break;
		    }
	   }

	 for(uint16_t i=0;i<rx_counter;i++)
		printf(" <%.2x> ",rx_buffer[i]);
	    printf("\n ATI Recieve <%d> Bytes data! \n",rx_counter);

	 return Rx_result;
}

void Ati_Mini45::Check_Cali_1(uint8_t* rx_buffer)
{
	for(uint16_t i=0;i<ATI_RX_CALI_LEN_1-3;i++)   
		rx_buffer[i] = rx_buffer[i+3];



	for(uint16_t i=0;i<8;i++)
	  	Calibration_ONE.CalibSerialNumber[i] = rx_buffer[i];
	    Calibration_ONE.CalibSerialNumber[7] = '\0';



	for(uint16_t i=0;i<32;i++)
	    Calibration_ONE.CalibPartNumber[i] = rx_buffer[i+8];
	    Calibration_ONE.CalibPartNumber[31] = '\0';



	for(uint16_t i=0;i<4;i++)
	    Calibration_ONE.CalibFamilyId[i] = rx_buffer[i+40];
	    Calibration_ONE.CalibFamilyId[3] = '\0';


	for(uint16_t i=0;i<20;i++)
	    Calibration_ONE.CalibTime[i] = rx_buffer[i+44];
	    Calibration_ONE.CalibTime[19] = '\0';


	uint32_t temp;
	float matrix[36];
	for(int16_t i=0;i<36;i++)
	{
	    temp = (uint32_t)DXL_MAKEDWORD(DXL_MAKEWORD(rx_buffer[i*4+64+3], rx_buffer[i*4+64+2]), DXL_MAKEWORD(rx_buffer[i*4+64+1], rx_buffer[i*4+64]));
	    memcpy(&matrix[i], &temp, sizeof(float));
	}



	for(uint16_t i=0;i<6;i++)
	{
		for(uint16_t j=0;j<6;j++)
		{
			Calibration_ONE.BasicMatrix[i][j]  = matrix[i*6+j];
		}
	}
}

void Ati_Mini45::Check_Cali_2(uint8_t* rx_buffer)
{
	for(int16_t i=0;i<ATI_RX_CALI_LEN_2-3;i++)
  	 rx_buffer[i] = rx_buffer[i+3];

    Calibration_ONE.ForceUnits = rx_buffer[0];
    Calibration_ONE.TorqueUnits = rx_buffer[1];

    uint32_t temp;
    for(int16_t i=0;i<6;i++)
    {
      temp = (uint32_t)DXL_MAKEDWORD(DXL_MAKEWORD(rx_buffer[i*4+2+3], rx_buffer[i*4+2+2]), DXL_MAKEWORD(rx_buffer[i*4+2+1], rx_buffer[i*4+2]));
      memcpy(&Calibration_ONE.MaxRating[i], &temp, sizeof(float));
    }


    Calibration_ONE.CountsPerForce = (int)(uint32_t)DXL_MAKEDWORD(DXL_MAKEWORD(rx_buffer[26+3], rx_buffer[26+2]), DXL_MAKEWORD(rx_buffer[26+1], rx_buffer[26]));
    Calibration_ONE.CountsPerTorque = (int)(uint32_t)DXL_MAKEDWORD(DXL_MAKEWORD(rx_buffer[30+3], rx_buffer[30+2]), DXL_MAKEWORD(rx_buffer[30+1], rx_buffer[30]));


    for(uint16_t i = 0;i<6;i++)
    {
      Calibration_ONE.GageGain[i] = (uint16_t)DXL_MAKEWORD(rx_buffer[i*2+34+1], rx_buffer[i*2+34]);
    }
    for(uint16_t i = 0;i<6;i++)
    {
      Calibration_ONE.GageOffset[i] = (uint16_t)DXL_MAKEWORD(rx_buffer[i*2+46+1], rx_buffer[i*2+46]);
    }
}

void Ati_Mini45::Print_Cali()
{
	printf("ATI CaliSerialNumber: %s\n",Calibration_ONE.CalibSerialNumber);
    printf("ATI CaliPartNumber: %s\n",Calibration_ONE.CalibPartNumber);
    printf("ATI CalibFamilyId: %s\n",Calibration_ONE.CalibFamilyId);
    printf("ATI CalibTime: %s\n",Calibration_ONE.CalibTime);
 
    printf("ATI BasicMatrix:\n");
    for(int16_t i = 0;i<6;i++)
    {
      for(int16_t j = 0;j<6;j++)
      {
        printf("%f\t",Calibration_ONE.BasicMatrix[i][j] );
      }
      printf("\n");
    }

    printf("ATI ForceUnits: ");
     switch (Calibration_ONE.ForceUnits) {
       case 1:
       printf("%s\n","Pound" );
       break;
       case 2:
       printf("%s\n","Newton(N)" );
       break;
       case 3:
       printf("%s\n","Kilopound" );
       break;
       case 4:
       printf("%s\n","KiloNewton" );
       break;
       case 5:
       printf("%s\n","Kilogram-equivalent force" );
       break;
       case 6:
       printf("%s\n","Gram-equivalent force" );
       break;
       default:
       printf("%s\n","Unkown Force Unit" );
       break;
     }

     printf("ATI TorqueUnits: ");
     switch (Calibration_ONE.TorqueUnits) {
       case 1:
       printf("%s\n","Pound-inch" );
       break;
       case 2:
       printf("%s\n","Pound-foot" );
       break;
       case 3:
       printf("%s\n","Newton-meter(N×m)" );
       break;
       case 4:
       printf("%s\n","Newton-milimeter" );
       break;
       case 5:
       printf("%s\n","Kilogram(-equivalent)-centimeter" );
       break;
       case 6:
       printf("%s\n","KiloNewton-meter" );
       break;
       default:
       printf("%s\n","Unkown Torque Unit" );
       break;
     }

     printf("ATI MaxRating: ");
     for(uint16_t i = 0;i<6;i++)
     {
      printf("%f\t",Calibration_ONE.MaxRating[i] );
     }

     printf("\n");
     printf("ATI: CountsPerForce: %d\n",Calibration_ONE.CountsPerForce );
     printf("ATI: CountsPerTorque:%d\n",Calibration_ONE.CountsPerTorque );

   printf("ATI GageGain: ");
     for(uint16_t i = 0;i<6;i++)
     {
       printf("%d\t",Calibration_ONE.GageGain[i] );
     }
   printf("\n");

   printf("ATI GageOffset: "); 
     for(uint16_t i = 0;i<6;i++)
     {
       printf("%d\t",Calibration_ONE.GageOffset[i] );
     }

    printf("\n");
}


void  Ati_Mini45::Check_Gain_offset(uint8_t* rx_buffer)
{
	uint16_t gage_gain[6],gage_offset[6];
    for(uint16_t i = 0;i<6;i++)
      {
        gage_gain[i] = (uint16_t)DXL_MAKEWORD(rx_buffer[i*2+3+1], rx_buffer[i*2+3]);
      }
    for(uint16_t i = 0;i<6;i++)
      {
        gage_offset[i] = (uint16_t)DXL_MAKEWORD(rx_buffer[i*2+15+1], rx_buffer[i*2+15]);
      }

    printf("ATI Read Gage Gain:");
    for(uint16_t i = 0;i<6;i++)
    {
    	printf(" %d ",gage_gain[i]);
    }

    printf("\n");
    printf("ATI Read Gage Offset:");
    for(uint16_t i = 0;i<6;i++)
    {
    	printf(" %d ",gage_offset[i]);
    }
    printf("\n");
}

void Ati_Mini45::update_to_gloable_var(gAti_Data *data)
{
	data->Fx = FT_data[0] - FT_bais[0];
	data->Fy = FT_data[1] - FT_bais[1];
	data->Fz = FT_data[2] - FT_bais[2];
	if( fabs(data->Fz) < 1e-6 ){
		printf("Absolute value is smaller than 1e-6: %.8f\n", data->Fz);
		printf("FT_bais[i] %f\n", FT_bais[2]);
		data->Fz = 1e-1;
	}
	data->Tx = FT_data[3] - FT_bais[3];
	data->Ty = FT_data[4] - FT_bais[4];
	data->Tz = FT_data[5] - FT_bais[5];
}

