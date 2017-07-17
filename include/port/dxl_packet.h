//##################################################
//# PROJECT: P1MC DXL_PACKET.h
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
#ifndef _DXL_PACKET_H_
#define _DXL_PACKET_H_

#include <map>
#include <vector>
#include <stdint.h>
#include <port/port.h>

#define BROADCAST_ID        0xFE    // 254
#define MAX_ID              0xFC    // 252

// Control table address
#define ADDR_TORQUE_ENABLE          562                 // Control table address is different in Dynamixel model
#define ADDR_GOAL_POSITION          596
#define ADDR_GOAL_TORQUE            604
#define ADDR_PRESENT_POSITION       611
#define ADDR_PRESENT_VELOCITY       615

// Data Byte Length
#define LEN_GOAL_TORQUE                       2
#define LEN_PRESENT_POSITION                  4
#define LEN_PRESENT_POSITION_VELOCITY         8

#define ADDR_MODE                   11

#define TORQUE_ENABLE               1                   // Value for enabling the torque
#define TORQUE_DISABLE              0                   // Value for disabling the torque

#define POSITION_MODE               3                   //
#define TORQUE_MODE                 0                   //

/* Macro for Control Table Value */
#define DXL_MAKEWORD(a, b)  ((unsigned short)(((unsigned char)(((unsigned long)(a)) & 0xff)) | ((unsigned short)((unsigned char)(((unsigned long)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b) ((unsigned int)(((unsigned short)(((unsigned long)(a)) & 0xffff)) | ((unsigned int)((unsigned short)(((unsigned long)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l)       ((unsigned short)(((unsigned long)(l)) & 0xffff))
#define DXL_HIWORD(l)       ((unsigned short)((((unsigned long)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)       ((unsigned char)(((unsigned long)(w)) & 0xff))
#define DXL_HIBYTE(w)       ((unsigned char)((((unsigned long)(w)) >> 8) & 0xff))

/* Instruction for DXL Protocol */
#define INST_PING               1
#define INST_READ               2
#define INST_WRITE              3
#define INST_REG_WRITE          4
#define INST_ACTION             5
#define INST_FACTORY_RESET      6
#define INST_SYNC_WRITE         131     // 0x83
#define INST_BULK_READ          146     // 0x92
// --- Only for 2.0 --- //
#define INST_REBOOT             8
#define INST_STATUS             85      // 0x55
#define INST_SYNC_READ          130     // 0x82
#define INST_BULK_WRITE         147     // 0x93

// Communication Result
#define COMM_SUCCESS        0       // tx or rx packet communication success
#define COMM_PORT_BUSY      -1000   // Port is busy (in use)
#define COMM_TX_FAIL        -1001   // Failed transmit instruction packet
#define COMM_RX_FAIL        -1002   // Failed get status packet
#define COMM_TX_ERROR       -2000   // Incorrect instruction packet
#define COMM_RX_WAITING     -3000   // Now recieving status packet
#define COMM_RX_TIMEOUT     -3001   // There is no status packet
#define COMM_RX_CORRUPT     -3002   // Incorrect status packet
#define COMM_NOT_AVAILABLE  -9000   //


class  dxl_PacketHandler
{
	private:
		unsigned short updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);

	public:
		dxl_PacketHandler();

		void addStuffing(uint8_t *packet);
		void removeStuffing(uint8_t *packet);

		void printTxRxResult(int result);
		void printRxPacketError(uint8_t error);

		int  write1ByteTxRx(PortHandler *port, uint8_t id, uint16_t address, uint8_t data, uint8_t *error);
		int  write4ByteTxRx(PortHandler *port, uint8_t id, uint16_t address, uint32_t data, uint8_t *error);
		int  writeTxRx(PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error);
	    int  txRxPacket(PortHandler *port, uint8_t *txpacket, uint8_t *rxpacket, uint8_t *error);
	    int  txPacket(PortHandler *port, uint8_t *txpacket);
	    int  rxPacket(PortHandler *port, uint8_t *rxpacket);

	    int  syncReadTx(PortHandler *port, uint16_t start_address, uint16_t data_length, uint8_t *param, uint16_t param_length);
	    int  readRx(PortHandler *port, uint16_t length, uint8_t *data, uint8_t *error);
	    int  syncWriteTxOnly(PortHandler *port, uint16_t start_address, uint16_t data_length, uint8_t *param, uint16_t param_length);
};


class dxl_GroupSyncRead
{
	private:
	  PortHandler    *port_;
	  dxl_PacketHandler  *ph_;

	  std::vector<uint8_t>            id_list_;
	  std::map<uint8_t, uint8_t* >    data_list_; // <id, data>

	  bool            last_result_;
	  bool            is_param_changed_;

	  uint8_t        *param_;
	  uint16_t        start_address_;
	  uint16_t        data_length_;

	  void    makeParam();

	public:
	  dxl_GroupSyncRead(PortHandler *port, dxl_PacketHandler *ph, uint16_t start_address, uint16_t data_length);
	  dxl_GroupSyncRead(PortHandler *port, dxl_PacketHandler *ph);
	  ~dxl_GroupSyncRead() { clearParam(); }

	  PortHandler         *getPortHandler()   { return port_; }
	  dxl_PacketHandler   *getPacketHandler() { return ph_; }

	  bool    			  addParam    (uint8_t id);
	  void                removeParam (uint8_t id);
	  void                clearParam  ();

	   int                txPacket();
	   int                rxPacket();
	   int                txRxPacket();

	   bool               isAvailable (uint8_t id, uint16_t address, uint16_t data_length);
	   uint32_t           getData     (uint8_t id, uint16_t address, uint16_t data_length);
};


class dxl_GroupSyncWrite
{
	private:
	  PortHandler    *port_;
	  dxl_PacketHandler  *ph_;

	  std::vector<uint8_t>            id_list_;
	  std::map<uint8_t, uint8_t* >    data_list_; // <id, data>

	  bool            is_param_changed_;

	  uint8_t        *param_;
	  uint16_t        start_address_;
	  uint16_t        data_length_;

	  void    makeParam();

	public:
	  dxl_GroupSyncWrite(PortHandler *port, dxl_PacketHandler *ph, uint16_t start_address, uint16_t data_length);
	  dxl_GroupSyncWrite(PortHandler *port, dxl_PacketHandler *ph);
	  ~dxl_GroupSyncWrite() { clearParam(); }

	  PortHandler         *getPortHandler()   { return port_; }
	  dxl_PacketHandler   *getPacketHandler() { return ph_; }

	  bool    addParam    (uint8_t id, uint8_t *data);
	  void    removeParam (uint8_t id);
	  bool    changeParam (uint8_t id, uint8_t *data);
	  void    clearParam  ();

	  int     txPacket();
};


#endif