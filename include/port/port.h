//##################################################
//# PROJECT: P1MC PORT.h
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
#ifndef _PORT_H_
#define _PORT_H_

#include <stdint.h>

class PortHandler
{
  private:
    int     socket_fd_;
    int     baudrate_;
    char    port_name_[30];

    double  packet_start_time_;
    double  packet_timeout_;
    double  tx_time_per_byte;

  public:
    static const int DEFAULT_BAUDRATE_ = 3000000;

    bool   is_using_;

  public:
      PortHandler(const char *port_name);
     	~PortHandler() { }

     bool    openPort();
     bool    openPort_ati();

     void    closePort();
     void    clearPort();
     void    Port_init(const char *port_name, const int baudrate);

     bool    setBaudRate(const int baudrate);
     bool    setupPort(int cflag_baud);
     int     getCFlagBaud(int baudrate);

     int     readPort(uint8_t *packet, int length);
     int     writePort(uint8_t *packet, int length);

     void    setPacketTimeout(uint16_t packet_length);
     void    setPacketTimeout(double msec);
     bool    isPacketTimeout();
     double  getCurrentTime();
     double  getTimeSinceStart();


     bool    setupPort_ati(int cflag_baud);  //added by li chunjing 2017-03-03
     bool    setBaudRate_ati(const int baudrate);
     void    Port_init_ati(const char *port_name, const int baudrate);
     bool    setupPort_ati_continue_receive();  //added by li chunjing 2017-03-08


     void    Port_init_mti(const char *port_name, const int baudrate);
     bool    setupPort_mti_continue_receive();  //added by li chunjing 2017-03-08
};

#endif