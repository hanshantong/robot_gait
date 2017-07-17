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
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <unistd.h>
#include <port/dxl_packet.h>

#define TXPACKET_MAX_LEN    (4*1024)
#define RXPACKET_MAX_LEN    (4*1024)

///////////////// for Protocol 2.0 Packet /////////////////
#define PKT_HEADER0             0
#define PKT_HEADER1             1
#define PKT_HEADER2             2
#define PKT_RESERVED            3
#define PKT_ID                  4
#define PKT_LENGTH_L            5
#define PKT_LENGTH_H            6
#define PKT_INSTRUCTION         7
#define PKT_ERROR               8
#define PKT_PARAMETER0          8

///////////////// Protocol 2.0 Error bit /////////////////
#define ERRNUM_RESULT_FAIL      1       // Failed to process the instruction packet.
#define ERRNUM_INSTRUCTION      2       // Instruction error
#define ERRNUM_CRC              3       // CRC check error
#define ERRNUM_DATA_RANGE       4       // Data range error
#define ERRNUM_DATA_LENGTH      5       // Data length error
#define ERRNUM_DATA_LIMIT       6       // Data limit error
#define ERRNUM_ACCESS           7       // Access error

#define ERRBIT_ALERT            128     //When the device has a problem, this bit is set to 1. Check "Device Status Check" value.



/////////////////////////////////////  CLASS: DEFINITION OF DXL_PACKETHANDLER  ////////////////////////////////////////

dxl_PacketHandler::dxl_PacketHandler(){}

unsigned short dxl_PacketHandler::updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
  uint16_t i;
  uint16_t crc_table[256] = {0x0000,
  0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
  0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
  0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
  0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
  0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
  0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
  0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
  0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
  0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
  0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
  0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
  0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
  0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
  0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
  0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
  0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
  0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
  0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
  0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
  0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
  0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
  0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
  0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
  0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
  0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
  0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
  0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
  0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
  0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
  0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
  0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
  0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
  0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
  0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
  0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
  0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
  0x820D, 0x8207, 0x0202 };

  for (uint16_t j = 0; j < data_blk_size; j++)
  {
    i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }

  return crc_accum;
}

void dxl_PacketHandler::printTxRxResult(int result)
{
  switch(result)
  {
    case COMM_SUCCESS:
      printf("[TxRxResult] Communication success.\n");
      break;

    case COMM_PORT_BUSY:
      printf("[TxRxResult] Port is in use!\n");
      break;

    case COMM_TX_FAIL:
      printf("[TxRxResult] Failed transmit instruction packet!\n");
      break;

    case COMM_RX_FAIL:
      printf("[TxRxResult] Failed get status packet from device!\n");
      break;

    case COMM_TX_ERROR:
      printf("[TxRxResult] Incorrect instruction packet!\n");
      break;

    case COMM_RX_WAITING:
      printf("[TxRxResult] Now recieving status packet!\n");
      break;

    case COMM_RX_TIMEOUT:
      printf("[TxRxResult] There is no status packet!\n");
      break;

    case COMM_RX_CORRUPT:
      printf("[TxRxResult] Incorrect status packet!\n");
      break;

    case COMM_NOT_AVAILABLE:
      printf("[TxRxResult] Protocol does not support This function!\n");
      break;

    default:
      break;
  }
}

void dxl_PacketHandler::printRxPacketError(uint8_t error)
{
  if (error & ERRBIT_ALERT)
    printf("[RxPacketError] Hardware error occurred. Check the error at Control Table (Hardware Error Status)!\n");

  int not_alert_error = error & ~ERRBIT_ALERT;

  switch(not_alert_error)
  {
    case 0:
      break;

    case ERRNUM_RESULT_FAIL:
      printf("[RxPacketError] Failed to process the instruction packet!\n");
      break;

    case ERRNUM_INSTRUCTION:
      printf("[RxPacketError] Undefined instruction or incorrect instruction!\n");
      break;

    case ERRNUM_CRC:
      printf("[RxPacketError] CRC doesn't match!\n");
      break;

    case ERRNUM_DATA_RANGE:
      printf("[RxPacketError] The data value is out of range!\n");
      break;

    case ERRNUM_DATA_LENGTH:
      printf("[RxPacketError] The data length does not match as expected!\n");
      break;

    case ERRNUM_DATA_LIMIT:
      printf("[RxPacketError] The data value exceeds the limit value!\n");
      break;

    case ERRNUM_ACCESS:
      printf("[RxPacketError] Writing or Reading is not available to target address!\n");
      break;

    default:
      printf("[RxPacketError] Unknown error code!\n");
      break;
  }
}


void dxl_PacketHandler::addStuffing(uint8_t *packet)
{
  int i = 0, index = 0;
  int packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
  int packet_length_out = packet_length_in;
  uint8_t temp[TXPACKET_MAX_LEN] = {0};

  for (uint8_t s = PKT_HEADER0; s <= PKT_LENGTH_H; s++)
    temp[s] = packet[s]; // FF FF FD XX ID LEN_L LEN_H
  //memcpy(temp, packet, PKT_LENGTH_H+1);
  index = PKT_INSTRUCTION;
  for (i = 0; i < packet_length_in - 2; i++)  // except CRC
  {
    temp[index++] = packet[i+PKT_INSTRUCTION];
    if (packet[i+PKT_INSTRUCTION] == 0xFD && packet[i+PKT_INSTRUCTION-1] == 0xFF && packet[i+PKT_INSTRUCTION-2] == 0xFF)
    {   // FF FF FD
      temp[index++] = 0xFD;
      packet_length_out++;
    }
  }
  temp[index++] = packet[PKT_INSTRUCTION+packet_length_in-2];
  temp[index++] = packet[PKT_INSTRUCTION+packet_length_in-1];


  //////////////////////////
  if (packet_length_in != packet_length_out)
    packet = (uint8_t *)realloc(packet, index * sizeof(uint8_t));

  ///////////////////////////

  for (uint8_t s = 0; s < index; s++)
    packet[s] = temp[s];
  //memcpy(packet, temp, index);
  packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
  packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);
}

void dxl_PacketHandler::removeStuffing(uint8_t *packet)
{
  int i = 0, index = 0;
  int packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
  int packet_length_out = packet_length_in;

  index = PKT_INSTRUCTION;
  for (i = 0; i < packet_length_in - 2; i++)  // except CRC
  {
    if (packet[i+PKT_INSTRUCTION] == 0xFD && packet[i+PKT_INSTRUCTION+1] == 0xFD && packet[i+PKT_INSTRUCTION-1] == 0xFF && packet[i+PKT_INSTRUCTION-2] == 0xFF)
    {   // FF FF FD FD
      packet_length_out--;
      i++;
    }
    packet[index++] = packet[i+PKT_INSTRUCTION];
  }
  packet[index++] = packet[PKT_INSTRUCTION+packet_length_in-2];
  packet[index++] = packet[PKT_INSTRUCTION+packet_length_in-1];

  packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
  packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);
}

int dxl_PacketHandler::write1ByteTxRx(PortHandler *port, uint8_t id, uint16_t address, uint8_t data, uint8_t *error)
{
  uint8_t data_write[1] = { data };
  return writeTxRx(port, id, address, 1, data_write, error);
}

int dxl_PacketHandler::write4ByteTxRx(PortHandler *port, uint8_t id, uint16_t address, uint32_t data, uint8_t *error)
{
  uint8_t data_write[4] = { DXL_LOBYTE(DXL_LOWORD(data)), DXL_HIBYTE(DXL_LOWORD(data)), DXL_LOBYTE(DXL_HIWORD(data)), DXL_HIBYTE(DXL_HIWORD(data)) };
  return writeTxRx(port, id, address, 4, data_write, error);
}

int dxl_PacketHandler::writeTxRx(PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error)
{
  int result                  = COMM_TX_FAIL;

  uint8_t *txpacket           = (uint8_t *)malloc(length + 12);
  //uint8_t *txpacket           = new uint8_t[length+12];
  uint8_t rxpacket[11]        = {0};

  txpacket[PKT_ID]            = id;
  txpacket[PKT_LENGTH_L]      = DXL_LOBYTE(length+5);
  txpacket[PKT_LENGTH_H]      = DXL_HIBYTE(length+5);
  txpacket[PKT_INSTRUCTION]   = INST_WRITE;
  txpacket[PKT_PARAMETER0+0]  = (uint8_t)DXL_LOBYTE(address);
  txpacket[PKT_PARAMETER0+1]  = (uint8_t)DXL_HIBYTE(address);

  for (uint8_t s = 0; s < length; s++)
    txpacket[PKT_PARAMETER0+2+s] = data[s];
  //memcpy(&txpacket[PKT_PARAMETER0+2], data, length);

  result = txRxPacket(port, txpacket, rxpacket, error);

  free(txpacket);
  //delete[] txpacket;
  return result;
}

// NOT for BulkRead / SyncRead instruction
int dxl_PacketHandler::txRxPacket(PortHandler *port, uint8_t *txpacket, uint8_t *rxpacket, uint8_t *error)
{
  int result = COMM_TX_FAIL;

  // tx packet
  result = txPacket(port, txpacket);
  if (result != COMM_SUCCESS)
    return result;

  // (ID == Broadcast ID && NOT BulkRead) == no need to wait for status packet
  // (Instruction == action) == no need to wait for status packet
  if ((txpacket[PKT_ID] == BROADCAST_ID && txpacket[PKT_INSTRUCTION] != INST_BULK_READ) ||
     (txpacket[PKT_ID] == BROADCAST_ID && txpacket[PKT_INSTRUCTION] != INST_SYNC_READ) ||
     (txpacket[PKT_INSTRUCTION] == INST_ACTION))
  {
    port->is_using_ = false;
    return result;
  }

  // set packet timeout
  if (txpacket[PKT_INSTRUCTION] == INST_READ)
  {
    port->setPacketTimeout((uint16_t)(DXL_MAKEWORD(txpacket[PKT_PARAMETER0+2], txpacket[PKT_PARAMETER0+3]) + 11));
  }
  else
  {
    port->setPacketTimeout((uint16_t)11);
    // HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H INST ERROR CRC16_L CRC16_H
  }

  // rx packet
  result = rxPacket(port, rxpacket);
  // check txpacket ID == rxpacket ID
  if (txpacket[PKT_ID] != rxpacket[PKT_ID])
    result = rxPacket(port, rxpacket);

  if (result == COMM_SUCCESS && txpacket[PKT_ID] != BROADCAST_ID)
  {
    if (error != 0)
      *error = (uint8_t)rxpacket[PKT_ERROR];
  }

  return result;
}

int dxl_PacketHandler::txPacket(PortHandler *port, uint8_t *txpacket)
{
  uint16_t total_packet_length   = 0;
  uint16_t written_packet_length = 0;

  if (port->is_using_)
    return COMM_PORT_BUSY;
  port->is_using_ = true;

  // byte stuffing for header
  addStuffing(txpacket);

  // check max packet length
  total_packet_length = DXL_MAKEWORD(txpacket[PKT_LENGTH_L], txpacket[PKT_LENGTH_H]) + 7;
  // 7: HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H
  if (total_packet_length > TXPACKET_MAX_LEN)
  {
    port->is_using_ = false;
    return COMM_TX_ERROR;
  }

  // make packet header
  txpacket[PKT_HEADER0]   = 0xFF;
  txpacket[PKT_HEADER1]   = 0xFF;
  txpacket[PKT_HEADER2]   = 0xFD;
  txpacket[PKT_RESERVED]  = 0x00;

  // add CRC16
  uint16_t crc = updateCRC(0, txpacket, total_packet_length - 2);    // 2: CRC16
  txpacket[total_packet_length - 2] = DXL_LOBYTE(crc);
  txpacket[total_packet_length - 1] = DXL_HIBYTE(crc);

  // tx packet
  port->clearPort();
  written_packet_length = port->writePort(txpacket, total_packet_length);
  if (total_packet_length != written_packet_length)
  {
    port->is_using_ = false;
    return COMM_TX_FAIL;
  }

  return COMM_SUCCESS;
}

int dxl_PacketHandler::rxPacket(PortHandler *port, uint8_t *rxpacket)
{
  int     result         = COMM_TX_FAIL;

  uint16_t rx_length     = 0;
  uint16_t wait_length   = 11; // minimum length (HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H INST ERROR CRC16_L CRC16_H)

  while(true)
  {
    usleep(10);
    rx_length += port->readPort(&rxpacket[rx_length], wait_length - rx_length);
    
    if (rx_length >= wait_length)
    {
      uint16_t idx = 0;
      // find packet header
      for (idx = 0; idx < (rx_length - 3); idx++)
      {
        if ((rxpacket[idx] == 0xFF) && (rxpacket[idx+1] == 0xFF) && (rxpacket[idx+2] == 0xFD) && (rxpacket[idx+3] != 0xFD))
          break;
      }

      if (idx == 0)   // found at the beginning of the packet
      {
        if (rxpacket[PKT_RESERVED] != 0x00 ||
           rxpacket[PKT_ID] > 0xFC ||
           DXL_MAKEWORD(rxpacket[PKT_LENGTH_L], rxpacket[PKT_LENGTH_H]) > RXPACKET_MAX_LEN ||
           rxpacket[PKT_INSTRUCTION] != 0x55)
        {
          // remove the first byte in the packet
          for (uint8_t s = 0; s < rx_length - 1; s++)
            rxpacket[s] = rxpacket[1 + s];
          //memcpy(&rxpacket[0], &rxpacket[idx], rx_length - idx);
          rx_length -= 1;
          continue;
        }

        // re-calculate the exact length of the rx packet
        if (wait_length != DXL_MAKEWORD(rxpacket[PKT_LENGTH_L], rxpacket[PKT_LENGTH_H]) + PKT_LENGTH_H + 1)
        {
          wait_length = DXL_MAKEWORD(rxpacket[PKT_LENGTH_L], rxpacket[PKT_LENGTH_H]) + PKT_LENGTH_H + 1;
          continue;
        }

        if (rx_length < wait_length)
        {
          // check timeout
          if (port->isPacketTimeout() == true)
          {
            if (rx_length == 0)
            {
              result = COMM_RX_TIMEOUT;
            }
            else
            {
              result = COMM_RX_CORRUPT;
            }
            break;
          }
          else
          {
            continue;
          }
        }

        // verify CRC16
        uint16_t crc = DXL_MAKEWORD(rxpacket[wait_length-2], rxpacket[wait_length-1]);
        if (updateCRC(0, rxpacket, wait_length - 2) == crc)
        {
          result = COMM_SUCCESS;
        }
        else
        {
          result = COMM_RX_CORRUPT;
        }
        break;
      }
      else
      {
        // remove unnecessary packets
        for (uint8_t s = 0; s < rx_length - idx; s++)
          rxpacket[s] = rxpacket[idx + s];
        //memcpy(&rxpacket[0], &rxpacket[idx], rx_length - idx);
        rx_length -= idx;
      }
    }
    else
    {
      // check timeout
      if (port->isPacketTimeout() == true)
      {
        if (rx_length == 0)
        {
          result = COMM_RX_TIMEOUT;
        }
        else
        {
          result = COMM_RX_CORRUPT;
        }
        break;
      }
    }
  }
  port->is_using_ = false;

  if (result == COMM_SUCCESS)
    removeStuffing(rxpacket);

  return result;
}

int dxl_PacketHandler::syncReadTx(PortHandler *port, uint16_t start_address, uint16_t data_length, uint8_t *param, uint16_t param_length)
{
  int result                 = COMM_TX_FAIL;

  uint8_t *txpacket           = (uint8_t *)malloc(param_length + 14);
  // 14: HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H

  txpacket[PKT_ID]            = BROADCAST_ID;
  txpacket[PKT_LENGTH_L]      = DXL_LOBYTE(param_length + 7); // 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
  txpacket[PKT_LENGTH_H]      = DXL_HIBYTE(param_length + 7); // 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
  txpacket[PKT_INSTRUCTION]   = INST_SYNC_READ;
  txpacket[PKT_PARAMETER0+0]  = DXL_LOBYTE(start_address);
  txpacket[PKT_PARAMETER0+1]  = DXL_HIBYTE(start_address);
  txpacket[PKT_PARAMETER0+2]  = DXL_LOBYTE(data_length);
  txpacket[PKT_PARAMETER0+3]  = DXL_HIBYTE(data_length);

  for (uint8_t s = 0; s < param_length; s++)
    txpacket[PKT_PARAMETER0+4+s] = param[s];
  //memcpy(&txpacket[PKT_PARAMETER0+4], param, param_length);

  result = txPacket(port, txpacket);
  if (result == COMM_SUCCESS)
    port->setPacketTimeout((uint16_t)((11 + data_length) * param_length));

  free(txpacket);
  return result;
}

int dxl_PacketHandler::readRx(PortHandler *port, uint16_t length, uint8_t *data, uint8_t *error)
{
  int result                 = COMM_TX_FAIL;
  uint8_t *rxpacket           = (uint8_t *)malloc(RXPACKET_MAX_LEN);
  //(length + 11 + (length/3));  // (length/3): consider stuffing
  //uint8_t *rxpacket           = new uint8_t[length + 11 + (length/3)];    // (length/3): consider stuffing

  result = rxPacket(port, rxpacket);
  if (result == COMM_SUCCESS)
  {
    if (error != 0)
      *error = (uint8_t)rxpacket[PKT_ERROR];
    for (uint8_t s = 0; s < length; s++)
      data[s] = rxpacket[PKT_PARAMETER0 + 1 + s];
    //memcpy(data, &rxpacket[PKT_PARAMETER0+1], length);
  }

  free(rxpacket);
  //delete[] rxpacket;
  return result;
}

int dxl_PacketHandler::syncWriteTxOnly(PortHandler *port, uint16_t start_address, uint16_t data_length, uint8_t *param, uint16_t param_length)
{
  int result                 = COMM_TX_FAIL;

  uint8_t *txpacket           = (uint8_t *)malloc(param_length + 14);
  //uint8_t *txpacket           = new uint8_t[param_length + 14];
  // 14: HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H

  txpacket[PKT_ID]            = BROADCAST_ID;
  txpacket[PKT_LENGTH_L]      = DXL_LOBYTE(param_length + 7); // 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
  txpacket[PKT_LENGTH_H]      = DXL_HIBYTE(param_length + 7); // 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
  txpacket[PKT_INSTRUCTION]   = INST_SYNC_WRITE;
  txpacket[PKT_PARAMETER0+0]  = DXL_LOBYTE(start_address);
  txpacket[PKT_PARAMETER0+1]  = DXL_HIBYTE(start_address);
  txpacket[PKT_PARAMETER0+2]  = DXL_LOBYTE(data_length);
  txpacket[PKT_PARAMETER0+3]  = DXL_HIBYTE(data_length);

  for (uint8_t s = 0; s < param_length; s++)
    txpacket[PKT_PARAMETER0+4+s] = param[s];
  //memcpy(&txpacket[PKT_PARAMETER0+4], param, param_length);

  result = txRxPacket(port, txpacket, 0, 0);

  free(txpacket);
  //delete[] txpacket;
  return result;
}

///////////////////////////////////  END OF CLASS: DEFINITION OF DXL_PACKETHANDLER  ////////////////////////////////////


/////////////////////////////////////  CLASS: DEFINITION OF DXL_GROUPSYNCREAD    ////////////////////////////////////////

dxl_GroupSyncRead::dxl_GroupSyncRead(PortHandler *port, dxl_PacketHandler *ph, uint16_t start_address, uint16_t data_length)
  : port_(port),
    ph_(ph),
    last_result_(false),
    is_param_changed_(false),
    param_(0),
    start_address_(start_address),
    data_length_(data_length)
{
  clearParam();
}

dxl_GroupSyncRead::dxl_GroupSyncRead(PortHandler *port, dxl_PacketHandler *ph)
  : port_(port),
    ph_(ph),
    last_result_(false),
    is_param_changed_(false),
    param_(0),
    start_address_(ADDR_PRESENT_POSITION),
    data_length_(LEN_PRESENT_POSITION_VELOCITY)
{
  clearParam();
}

void dxl_GroupSyncRead::clearParam()
{
    if (id_list_.size() == 0)
    return;

  for (unsigned int i = 0; i < id_list_.size(); i++)
    delete[] data_list_[id_list_[i]];

  id_list_.clear();
  data_list_.clear();
  if (param_ != 0)
    delete[] param_;
  param_ = 0;
}

bool dxl_GroupSyncRead::addParam(uint8_t id)
{
  if (std::find(id_list_.begin(), id_list_.end(), id) != id_list_.end())   // id already exist
    return false;

  id_list_.push_back(id);
  data_list_[id] = new uint8_t[data_length_];

  is_param_changed_   = true;
  return true;
}

void dxl_GroupSyncRead::removeParam(uint8_t id)
{
  std::vector<uint8_t>::iterator it = std::find(id_list_.begin(), id_list_.end(), id);
  if (it == id_list_.end())    // NOT exist
    return;

  id_list_.erase(it);
  delete[] data_list_[id];
  data_list_.erase(id);

  is_param_changed_   = true;
}

void dxl_GroupSyncRead::makeParam()
{
  if (id_list_.size() == 0)
    return;

  if (param_ != 0)
    delete[] param_;
  param_ = 0;

  param_ = new uint8_t[id_list_.size() * 1];  // ID(1)

  int idx = 0;
  for (unsigned int i = 0; i < id_list_.size(); i++)
    param_[idx++] = id_list_[i];
}

int dxl_GroupSyncRead::txPacket()
{
  if (id_list_.size() == 0)
    return COMM_NOT_AVAILABLE;

  if (is_param_changed_ == true)
    makeParam();

  return ph_->syncReadTx(port_, start_address_, data_length_, param_, (uint16_t)id_list_.size() * 1);
}

int dxl_GroupSyncRead::rxPacket()
{
  last_result_ = false;

  int cnt            = id_list_.size();
  int result         = COMM_RX_FAIL;

  if (cnt == 0)
    return COMM_NOT_AVAILABLE;

  for (int i = 0; i < cnt; i++)
  {
    uint8_t id = id_list_[i];

    result = ph_->readRx(port_, data_length_, data_list_[id], 0);   //????????
    if (result != COMM_SUCCESS){
      printf("i: %d\n", i);
      return result;
    }
  }



  if (result == COMM_SUCCESS)
    last_result_ = true;

  return result;
}

int dxl_GroupSyncRead::txRxPacket()
{
  int result         = COMM_TX_FAIL;

  result = txPacket();
  if (result != COMM_SUCCESS)
    return result;

  return rxPacket();
}

bool dxl_GroupSyncRead::isAvailable(uint8_t id, uint16_t address, uint16_t data_length)
{
  if (last_result_ == false || data_list_.find(id) == data_list_.end())
    return false;

  if (address < start_address_ || start_address_ + data_length_ - data_length < address)
    return false;

  return true;
}

uint32_t dxl_GroupSyncRead::getData(uint8_t id, uint16_t address, uint16_t data_length)
{
  if (isAvailable(id, address, data_length) == false)
    return 0;

  switch(data_length)
  {
    case 1:
      return data_list_[id][address - start_address_];

    case 2:
      return DXL_MAKEWORD(data_list_[id][address - start_address_], data_list_[id][address - start_address_ + 1]);

    case 4:
      return DXL_MAKEDWORD(DXL_MAKEWORD(data_list_[id][address - start_address_ + 0], data_list_[id][address - start_address_ + 1]),
                 DXL_MAKEWORD(data_list_[id][address - start_address_ + 2], data_list_[id][address - start_address_ + 3]));

    default:
      return 0;
  }
}
////////////////////////////////// END OF  CLASS: DEFINITION OF DXL_GROUPSYNCREAD    ////////////////////////////////////

//////////////////////////////////  CLASS: DEFINITION OF DXL_GROUPSYNCWRTIE    ////////////////////////////////////

dxl_GroupSyncWrite::dxl_GroupSyncWrite(PortHandler *port, dxl_PacketHandler *ph, uint16_t start_address, uint16_t data_length)
  : port_(port),
    ph_(ph),
    is_param_changed_(false),
    param_(0),
    start_address_(start_address),
    data_length_(data_length)
{
  clearParam();
}

dxl_GroupSyncWrite::dxl_GroupSyncWrite(PortHandler *port, dxl_PacketHandler *ph)
  : port_(port),
    ph_(ph),
    is_param_changed_(false),
    param_(0),
    start_address_(ADDR_GOAL_TORQUE),
    data_length_(LEN_GOAL_TORQUE)
{
  clearParam();
}

void dxl_GroupSyncWrite::makeParam()
{
  if (id_list_.size() == 0) return;

  if (param_ != 0)
    delete[] param_;
  param_ = 0;

  param_ = new uint8_t[id_list_.size() * (1 + data_length_)]; // ID(1) + DATA(data_length)

  int idx = 0;
  for (unsigned int i = 0; i < id_list_.size(); i++)
  {
    uint8_t id = id_list_[i];
    if (data_list_[id] == 0)
      return;

    param_[idx++] = id;
    for (int c = 0; c < data_length_; c++)
      param_[idx++] = (data_list_[id])[c];
  }
}

void dxl_GroupSyncWrite::clearParam()
{
  if (id_list_.size() == 0)
    return;

  for (unsigned int i = 0; i < id_list_.size(); i++)
    delete[] data_list_[id_list_[i]];

  id_list_.clear();
  data_list_.clear();
  if (param_ != 0)
    delete[] param_;
  param_ = 0;
}

bool dxl_GroupSyncWrite::addParam(uint8_t id, uint8_t *data)
{
  if (std::find(id_list_.begin(), id_list_.end(), id) != id_list_.end())   // id already exist
    return false;

  id_list_.push_back(id);
  data_list_[id]    = new uint8_t[data_length_];
  for (int c = 0; c < data_length_; c++)
    data_list_[id][c] = data[c];

  is_param_changed_   = true;
  return true;
}

void dxl_GroupSyncWrite::removeParam(uint8_t id)
{
  std::vector<uint8_t>::iterator it = std::find(id_list_.begin(), id_list_.end(), id);
  if (it == id_list_.end())    // NOT exist
    return;

  id_list_.erase(it);
  delete[] data_list_[id];
  data_list_.erase(id);

  is_param_changed_   = true;
}

bool dxl_GroupSyncWrite::changeParam(uint8_t id, uint8_t *data)
{
  std::vector<uint8_t>::iterator it = std::find(id_list_.begin(), id_list_.end(), id);
  if (it == id_list_.end())    // NOT exist
    return false;

  delete[] data_list_[id];
  data_list_[id]    = new uint8_t[data_length_];
  for (int c = 0; c < data_length_; c++)
    data_list_[id][c] = data[c];

  is_param_changed_   = true;
  return true;
}

int dxl_GroupSyncWrite::txPacket()
{
  if (id_list_.size() == 0)
    return COMM_NOT_AVAILABLE;

  if (is_param_changed_ == true)
    makeParam();

  return ph_->syncWriteTxOnly(port_, start_address_, data_length_, param_, id_list_.size() * (1 + data_length_));
}
////////////////////////////////// END OF  CLASS: DEFINITION OF DXL_GROUPSYNCWRTIE    /////////////////////////////

