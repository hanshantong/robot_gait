//##################################################
//# PROJECT: P1MC ATI.h
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
#ifndef _ATI_H_
#define _ATI_H_

#include <stdint.h>
#include <port/port.h>
#include <port/dxl_packet.h>

#define BIAS_NUM    7000


/* struct to store calibration information */
struct Calibration{
  uint8_t CalibSerialNumber[8]  ;
  uint8_t CalibPartNumber[32]   ;
  uint8_t CalibFamilyId[4]      ;
  uint8_t CalibTime[20]         ;
  float   BasicMatrix[6][6]     ;
  uint8_t ForceUnits            ;
  uint8_t TorqueUnits           ;
  float   MaxRating[6]          ;
  int     CountsPerForce        ;
  int     CountsPerTorque       ;
  uint16_t GageGain[6]          ;
  uint16_t GageOffset[6]        ;
};

struct gAti_Data
{
  float  Fx;
  float  Fy;
  float  Fz;
  float  Tx;  
  float  Ty;
  float  Tz;
};

class  Ati_Mini45
{
public:
	uint8_t     RX_BUFFER[255];
  uint16_t    print_counter;
  bool        check_error;
  bool        flag_SN_right;  //is CalibSerialNumber right      // added by li chunjing 2017-05-16
  bool        flag_PN_right;  //is CalibPartNumber right
  bool        flag_FID_right; //is CalibFamilyId right
  bool        flag_FU_right;  //is ForceUnits right
  bool        flag_TU_right;  //is TorqueUnits right
  int         SuccessRead;
  uint16_t    check_error_counter;

  float FT_data[6];
  float FT_bais[6];
  int   gages[6];  
  float FT_bais_matrix[BIAS_NUM][6];

  struct Calibration Calibration_info;
	struct Calibration Calibration_ONE;


public:
	   Ati_Mini45();
     Ati_Mini45(struct Calibration ati_cali);
	  ~Ati_Mini45(){};

    void       read_calibration(PortHandler *ati_portHandler);
    void       read_gage_offset(PortHandler *ati_portHandler);
    void       write_gage(PortHandler *ati_portHandler);
    bool       Unlock_storage(PortHandler *ati_portHandler);
    bool       Lock_storage(PortHandler *ati_portHandler);
    void       start_stream(PortHandler *ati_portHandler);
    void       stop_stream(PortHandler *ati_portHandler);
    void       update_ft_data(PortHandler *ati_portHandler);
    void       update_ft_data_restart_stream_mode(PortHandler *ati_portHandler);
    void       print_ft();
    void       update_to_gloable_var(gAti_Data *data);

    uint8_t    rx_nonblock(PortHandler *ati_portHandler, uint8_t* rx_buffer, uint16_t rx_length);
    void       Check_Cali_1(uint8_t* rx_buffer);
    void       Check_Cali_2(uint8_t* rx_buffer);
    void       Print_Cali();
    void       Check_Gain_offset(uint8_t* rx_buffer);


};




#endif