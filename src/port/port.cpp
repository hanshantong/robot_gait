//##################################################
//# PROJECT: P1MC PORT.CPP
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
#include <string.h>
#include <fcntl.h>
#include <time.h>
#include <termios.h>
#include <unistd.h>
#include <port/port.h>

#define LATENCY_TIMER   4  // msec (USB latency timer)

PortHandler::PortHandler(const char *port_name)
  : socket_fd_(-1),
    baudrate_(DEFAULT_BAUDRATE_),
    packet_start_time_(0.0),
    packet_timeout_(0.0),
    tx_time_per_byte(0.0)
{
  is_using_ = false;
  strcpy(port_name_, port_name);
}

bool PortHandler::openPort()
{
  return setBaudRate(baudrate_);
}

void PortHandler::closePort()
{
  if(socket_fd_ != -1)
    close(socket_fd_);
  socket_fd_ = -1;
}

void PortHandler::clearPort()
{
  tcflush(socket_fd_, TCIOFLUSH);
}

void PortHandler::Port_init(const char *port_name, const int baudrate)
{
    // Open port
  if (openPort())
  {
    printf("Succeeded to open the port %s !\n",port_name);
  }
  else
  {
    printf("Failed to open the port!\n");
  }

// Set port baudrate
  if (setBaudRate(baudrate))
  {
    printf("Succeeded to change the baudrate %d !\n",baudrate);
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
  }
}

bool PortHandler::setBaudRate(const int baudrate)
{
  int baud = getCFlagBaud(baudrate);

  closePort();

  if(baud <= 0)   // custom baudrate
  {
    printf("Please check the baudrate!\n");
    return false;
  }
  else
  {
    baudrate_ = baudrate;
    return setupPort(baud);
  }
}

bool PortHandler::setupPort(int cflag_baud)
{
  struct termios newtio;

  socket_fd_ = open(port_name_, O_RDWR|O_NOCTTY|O_NONBLOCK);
  if(socket_fd_ < 0)
  {
    printf("[PortHandler::SetupPort] Error opening serial port!\n");
    return false;
  }

  bzero(&newtio, sizeof(newtio)); // clear struct for new port settings

  newtio.c_cflag = cflag_baud | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag      = 0;
  newtio.c_lflag      = 0;
  newtio.c_cc[VTIME]  = 0;
  newtio.c_cc[VMIN]   = 0;

  // clean the buffer and activate the settings for the port
  tcflush(socket_fd_, TCIFLUSH);
  tcsetattr(socket_fd_, TCSANOW, &newtio);

  tx_time_per_byte = (1000.0 / (double)baudrate_) * 10.0;
  return true;
}

int PortHandler::readPort(uint8_t *packet, int length)
{
  return read(socket_fd_, packet, length);
}

int PortHandler::writePort(uint8_t *packet, int length)
{
  return write(socket_fd_, packet, length);
}

void PortHandler::setPacketTimeout(uint16_t packet_length)
{
  packet_start_time_  = getCurrentTime();
  packet_timeout_     = (tx_time_per_byte * (double)packet_length) + (LATENCY_TIMER * 2.0) + 2.0;
}

void PortHandler::setPacketTimeout(double msec)
{
  packet_start_time_  = getCurrentTime();
  packet_timeout_     = msec;
}

bool PortHandler::isPacketTimeout()
{
  if(getTimeSinceStart() > packet_timeout_)
  {
    packet_timeout_ = 0;
    return true;
  }
  return false;
}

double PortHandler::getCurrentTime()
{
  struct timespec tv;
  clock_gettime( CLOCK_REALTIME, &tv);
  return ((double)tv.tv_sec*1000.0 + (double)tv.tv_nsec*0.001*0.001);
}

double PortHandler::getTimeSinceStart()
{
  double time;

  time = getCurrentTime() - packet_start_time_;
  if(time < 0.0)
    packet_start_time_ = getCurrentTime();

  return time;
}

int PortHandler::getCFlagBaud(int baudrate)
{
  switch(baudrate)
  {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 500000:
      return B500000;
    case 576000:
      return B576000;
    case 921600:
      return B921600;
    case 1000000:
      return B1000000;
    case 1152000:
      return B1152000;
    case 1250000:           //added by li chunjing 2017-03-04
      return B1152000;
    case 1500000:
      return B1500000;
    case 2000000:
      return B2000000;
    case 2500000:
      return B2500000;
    case 3000000:
      return B3000000;
    case 3500000:
      return B3500000;
    case 4000000:
      return B4000000;
    default:
      return -1;
  }
}


///////////////////////////////////////     ATI PORT CONFIG FUNCTION  ////////////////////////////////////////////

void PortHandler::Port_init_ati(const char *port_name, const int baudrate)
{
  if (openPort_ati())
  {
    printf("ATI: Succeeded to open the port %s !\n",port_name);
  }
  else
  {
    printf("ATI:Failed to open the port!\n");
  }

  //Set port baudrate
  if (setBaudRate_ati(baudrate))
  {
    printf("ATI: Succeeded to change the baudrate %d !\n",baudrate);
  }
  else
  {
    printf("ATI: Failed to change the baudrate!\n");
    printf("ATI: Press any key to terminate...\n");
  }
}

bool PortHandler::openPort_ati()
{
     return setBaudRate_ati(baudrate_);
}

bool PortHandler::setBaudRate_ati(const int baudrate)
{
  int baud = getCFlagBaud(baudrate);

  closePort();

  if(baud <= 0)   // custom baudrate
  {
    setupPort(B38400);
    baudrate_ = baudrate;
    return false;
  }
  else
  {
    baudrate_ = baudrate;
    return setupPort_ati(baud);
  }
}

bool PortHandler::setupPort_ati(int cflag_baud)  //added by li chunjing 2017-03-03
{
  struct termios newtio;

  socket_fd_ = open(port_name_, O_RDWR|O_NOCTTY|O_NONBLOCK);
  if(socket_fd_ < 0)
  {
    printf("[PortHandlerLinux::SetupPort] Error opening serial port!\n");
    return false;
  }

  bzero(&newtio, sizeof(newtio)); // clear struct for new port settings

  newtio.c_cflag = cflag_baud | CS8 | CLOCAL | CREAD;
//  newtio.c_iflag = IGNPAR;
  newtio.c_cflag |= PARENB;
  newtio.c_cflag &= ~PARODD;
  newtio.c_oflag      = 0;
  newtio.c_lflag      = 0;
  newtio.c_cc[VTIME]  = 0;
  newtio.c_cc[VMIN]   = 0;

  newtio.c_ispeed = 1250000;
  newtio.c_ospeed = 1250000;

  // clean the buffer and activate the settings for the port
  tcflush(socket_fd_, TCIFLUSH);
  tcsetattr(socket_fd_, TCSANOW, &newtio);

  tx_time_per_byte = (1000.0 / (double)baudrate_) * 10.0;
  return true;
}

bool PortHandler::setupPort_ati_continue_receive()  //added by li chunjing 2017-03-08
{
//  int cflag_baud = getCFlagBaud(baudrate);

   if(socket_fd_ != -1)     //before open file ,first close file
   {
     if(close(socket_fd_) == 0)
     {
      socket_fd_ = -1;
      printf("ATI: closed serial port!\n");
     }   
   }

   socket_fd_ = open(port_name_, O_RDWR|O_NOCTTY);   //block way to open port.    by lichunjing 2017-03-08
  if(socket_fd_ < 0)
  {
    printf("ATI: Error opening serial port!\n");
    return false;
  }
  else
  {
      printf("ATI: succeeded to open serial port!\n");
  }

  struct termios newtio;

  bzero(&newtio, sizeof(newtio)); // clear struct for new port settings

  newtio.c_cflag =CS8 | CLOCAL | CREAD;
  newtio.c_cflag |= PARENB;
  newtio.c_cflag &= ~PARODD;
  newtio.c_oflag      = 0;
  newtio.c_lflag      = 0;
  newtio.c_cc[VTIME]  = 0;
  newtio.c_cc[VMIN]   = 13;

  newtio.c_ispeed = 1250000;
  newtio.c_ospeed = 1250000;

  // clean the buffer and activate the settings for the port
  tcflush(socket_fd_, TCIFLUSH);
  tcsetattr(socket_fd_, TCSANOW, &newtio);

  return true;
}


///////////////////////////////////////     MTI PORT CONFIG FUNCTION  ////////////////////////////////////////////

void PortHandler::Port_init_mti(const char *port_name, const int baudrate)
{
  if (openPort())
  {
    printf("MTI300: Succeeded to open the port %s !\n",port_name);
  }
  else
  {
    printf("MTI300:Failed to open the port!\n");
  }

  //Set port baudrate
  if (setBaudRate(baudrate))
  {
    printf("MTI300: Succeeded to change the baudrate %d !\n",baudrate);
  }
  else
  {
    printf("MTI300: Failed to change the baudrate!\n");
    printf("MTI300: Press any key to terminate...\n");
  }
}

bool PortHandler::setupPort_mti_continue_receive()  //added by li chunjing 2017-03-08
{
//  int cflag_baud = getCFlagBaud(baudrate);

   if(socket_fd_ != -1)     //before open file ,first close file
   {
     if(close(socket_fd_) == 0)
     {
      socket_fd_ = -1;
      printf("[PortHandlerLinux::SetupPort]MTI: closed serial port!\n");
     }   
   }

   socket_fd_ = open(port_name_, O_RDWR|O_NOCTTY);   //block way to open port.    by lichunjing 2017-03-08
  if(socket_fd_ < 0)
  {
    printf("[PortHandlerLinux::SetupPort]MTI: Error opening serial port!\n");
    return false;
  }
  else
  {
      printf("[PortHandlerLinux::SetupPort]MTI: succeeded to open serial port!\n");
  }

  struct termios newtio;

  bzero(&newtio, sizeof(newtio)); // clear struct for new port settings

  newtio.c_cflag =CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  // newtio.c_cflag |= PARENB;
  // newtio.c_cflag &= ~PARODD;
  newtio.c_oflag      = 0;
  newtio.c_lflag      = 0;
  newtio.c_cc[VTIME]  = 0;
  newtio.c_cc[VMIN]   = 50;

  newtio.c_ispeed = 115200;
  newtio.c_ospeed = 115200;

  // clean the buffer and activate the settings for the port
  tcflush(socket_fd_, TCIFLUSH);
  tcsetattr(socket_fd_, TCSANOW, &newtio);

  return true;
}