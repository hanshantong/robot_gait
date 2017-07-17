//##################################################
//# PROJECT: P1MC MTI.CPP
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
#include <pthread.h>
#include <string.h>
#include <sensor/mti.h>
#include <port/dxl_packet.h>


uint8_t MTI300_GOTO_CONFIG[] = {0xFA,0xFF,0x30,0x00,0xD1};
uint8_t MTI300_GOTO_MEASUREMENT[] = {0xFA,0xFF,0x10,0x00,0xF1};
uint8_t MTI300_REQ_PRODUCTCODE[] = {0xFA,0xFF,0x1C,0x00,0xE5};
uint8_t MTI300_REQ_DEVICE_SN[] = {0xFA,0xFF,0x00,0x00,0x01};
uint8_t MTI300_REQ_DEVICE_FIRMWARE[] = {0xFA,0xFF,0x12,0x00,0xEF};
uint8_t MTI300_REQ_DEVICE_BAUDRATE[] = {0xFA,0xFF,0x18,0x00,0xE9};
uint8_t MTI300_REQ_OUTPUT_CONFIGRATION[] = {0xFA,0xFF,0xC0,0x00,0x41};

void Mti300::gotoConfig(PortHandler *mti300_portHandler)
{
	int16_t mti_connect_counter=100;
	while(mti_connect_counter--)
	{
		mti300_portHandler->clearPort();
	    mti300_portHandler->writePort(MTI300_GOTO_CONFIG,sizeof(MTI300_GOTO_CONFIG));
	    rx_nonblock(mti300_portHandler, MTI300_RX_BUFFER, 5);

	    if((MTI300_RX_BUFFER[0] ==0xFA)&&(MTI300_RX_BUFFER[1] == 0xFF)&&(MTI300_RX_BUFFER[2] ==0x31)&&(MTI300_RX_BUFFER[3] ==0x00)&&(MTI300_RX_BUFFER[4] ==0xD0))
	    {
	        printf("MTI-300 Connect succeeded after %d times!\n",100-mti_connect_counter);
	        printf("MTI-300 is in config status!\n");
	        break;
	    }
	}
	if(mti_connect_counter <= 0)
	{
		printf("MTI-300 Connect failed after %d times!\n",100-mti_connect_counter);
		printf("try again!\n");
        mti300_portHandler->closePort();
		pthread_exit(0);   
	}
}

uint8_t Mti300::rx_nonblock(PortHandler *mti300_portHandler, uint8_t* rx_buffer, uint16_t rx_length)
{
	uint8_t  Rx_result = 0;
	uint16_t rx_counter = 0;
	while(1)
	{
		rx_counter += mti300_portHandler->readPort(&rx_buffer[rx_counter], rx_length - rx_counter);
		if (rx_counter >= rx_length)
			{
				Rx_result = 1;
				break;
			}
	}

	for(uint16_t i=0;i<rx_counter;i++)
	printf(" <%.2x> ",rx_buffer[i]);
	printf("\n MTI Recieve <%d> Bytes data! \n",rx_counter);

	return Rx_result;
}

uint8_t Mti300::rx_nonblock(PortHandler *mti300_portHandler, uint16_t rx_length)
{
	uint8_t  rx_buffer[255];
	uint8_t  Rx_result = 0;
	uint16_t rx_counter = 0;
	while(1)
	{
		rx_counter += mti300_portHandler->readPort(&rx_buffer[rx_counter], rx_length - rx_counter);
		if (rx_counter >= rx_length)
			{
				Rx_result = 1;
				break;
			}
	}

	for(uint16_t i=0;i<rx_counter;i++)
	printf(" <%.2x> ",rx_buffer[i]);
	printf("\n MTI Recieve <%d> Bytes data! \n",rx_counter);

	return Rx_result;
}

void Mti300::get_dev_code(PortHandler *mti300_portHandler)
{
	mti300_portHandler->clearPort();
	mti300_portHandler->writePort(MTI300_REQ_PRODUCTCODE,sizeof(MTI300_REQ_PRODUCTCODE));
	rx_nonblock(mti300_portHandler, MTI300_RX_BUFFER, 25);

	for(uint16_t i=0;i<20;i++)
	{
		product_info.Product_Code[i] = MTI300_RX_BUFFER[i+4];
	}
	product_info.Product_Code[20] = 0x00;
	printf("IMU CODE: %s\n",product_info.Product_Code);
}

void Mti300::get_dev_SN(PortHandler *mti300_portHandler)
{
	mti300_portHandler->clearPort();
	mti300_portHandler->writePort(MTI300_REQ_DEVICE_SN,sizeof(MTI300_REQ_DEVICE_SN));
	rx_nonblock(mti300_portHandler, MTI300_RX_BUFFER, 9);

	for(uint16_t i=0;i<4;i++)
	{
		product_info.Product_SN[i] = MTI300_RX_BUFFER[i+4];
	}
	product_info.Product_SN[4] = 0x00;
	printf("IMU SN: %.2x%.2x%.2x%.2x\n",product_info.Product_SN[0],product_info.Product_SN[1],product_info.Product_SN[2],product_info.Product_SN[3]);
}

void Mti300::get_dev_baud(PortHandler *mti300_portHandler)
{
	mti300_portHandler->clearPort();
	mti300_portHandler->writePort(MTI300_REQ_DEVICE_BAUDRATE,sizeof(MTI300_REQ_DEVICE_BAUDRATE));
	rx_nonblock(mti300_portHandler, MTI300_RX_BUFFER, 6);
	BaudRate = MTI300_RX_BUFFER[4];

	switch(BaudRate)
	{
		case 0x80:printf("MTI300 BaudRate: 921600\n");
		case 0x0A:printf("MTI300 BaudRate: 921600\n");break;
		case 0x00:printf("MTI300 BaudRate: 460800\n");break;
		case 0x01:printf("MTI300 BaudRate: 230400\n");break;
		case 0x02:printf("MTI300 BaudRate: 115200\n");break;
		case 0x03:printf("MTI300 BaudRate: 76600\n");break;
		case 0x04:printf("MTI300 BaudRate: 57600\n");break;
		case 0x05:printf("MTI300 BaudRate: 38400\n");break;
		case 0x06:printf("MTI300 BaudRate: 28800\n");break;
		case 0x07:printf("MTI300 BaudRate: 19200\n");break;
		case 0x08:printf("MTI300 BaudRate: 14400\n");break;
		case 0x09:printf("MTI300 BaudRate: 9600\n");break;
		case 0x0B:printf("MTI300 BaudRate: 4800\n");break;
		default: printf("MTI300 error BaudRate!\n");break;
	}
}

void Mti300::get_dev_output_config(PortHandler *mti300_portHandler)
{
	mti300_portHandler->clearPort();
	mti300_portHandler->writePort(MTI300_REQ_OUTPUT_CONFIGRATION,sizeof(MTI300_REQ_OUTPUT_CONFIGRATION));
	rx_nonblock(mti300_portHandler, MTI300_RX_BUFFER, 17);

	OutputConfigration[0].Data_ID[0] = MTI300_RX_BUFFER[4];
	OutputConfigration[0].Data_ID[1] = MTI300_RX_BUFFER[5];
	OutputConfigration[0].SampleRate = (uint16_t)DXL_MAKEWORD(MTI300_RX_BUFFER[7],MTI300_RX_BUFFER[6]);
	if((OutputConfigration[0].Data_ID[0]==0x20)&&(OutputConfigration[0].Data_ID[1]==0x30))
	{
		printf("Data ID:%x%x\t",OutputConfigration[0].Data_ID[0], OutputConfigration[0].Data_ID[1]);
		printf("Data Type: EulerAngles\t");
		printf("Data Format: float\t");
		printf("Coordinate system: ENU\t");
		printf("Sample Rate: %d HZ\n",OutputConfigration[0].SampleRate);
	}

	OutputConfigration[1].Data_ID[0] = MTI300_RX_BUFFER[8];
	OutputConfigration[1].Data_ID[1] = MTI300_RX_BUFFER[9];
	OutputConfigration[1].SampleRate = (uint16_t)DXL_MAKEWORD(MTI300_RX_BUFFER[11],MTI300_RX_BUFFER[10]);
	if((OutputConfigration[1].Data_ID[0]==0x40)&&(OutputConfigration[1].Data_ID[1]==0x20))
	{
		printf("Data ID:%x%x\t",OutputConfigration[1].Data_ID[0], OutputConfigration[1].Data_ID[1]);
		printf("Data Type: Acceleration\t");
		printf("Data Format: float\t");
		printf("Coordinate system: ENU\t");
		printf("Sample Rate: %d HZ\n",OutputConfigration[1].SampleRate);
	}

	OutputConfigration[2].Data_ID[0] = MTI300_RX_BUFFER[12];
	OutputConfigration[2].Data_ID[1] = MTI300_RX_BUFFER[13];
	OutputConfigration[2].SampleRate = (uint16_t)DXL_MAKEWORD(MTI300_RX_BUFFER[15],MTI300_RX_BUFFER[14]);
	if((OutputConfigration[2].Data_ID[0]==0x80)&&(OutputConfigration[2].Data_ID[1]==0x20))
	{
		printf("Data ID:%x%x\t",OutputConfigration[2].Data_ID[0], OutputConfigration[2].Data_ID[1]);
		printf("Data Type: RateOfTurn\t");
		printf("Data Format: float\t");
		printf("Coordinate system: ENU\t");
		printf("Sample Rate: %d HZ\n",OutputConfigration[2].SampleRate);
	}

}

void Mti300::gotoMeasurement(PortHandler *mti300_portHandler)
{
	mti300_portHandler->writePort(MTI300_GOTO_MEASUREMENT,sizeof(MTI300_GOTO_MEASUREMENT));
	rx_nonblock(mti300_portHandler, MTI300_RX_BUFFER, 5);
}

float Mti300::uint32_to_float(uint8_t* rx_buffer)
{
	uint32_t temp;
	float result;

    temp = (uint32_t)DXL_MAKEDWORD(DXL_MAKEWORD(rx_buffer[3], rx_buffer[2]), DXL_MAKEWORD(rx_buffer[1], rx_buffer[0]));
    memcpy(&result, &temp, sizeof(float));
    return result;
}

void  Mti300::recv_data(PortHandler *mti300_portHandler)
{
	mti300_portHandler->readPort(MTI300_RX_BUFFER, 50);

	uint32_t checksum = 0;
	uint8_t  checsum_result;

	for(uint16_t i=1;i<50;i++)
	   checksum += MTI300_RX_BUFFER[i];

	checsum_result = checksum &(0x00ff);
	if(checsum_result == 0)    //checksum OK!
		{
			if((MTI300_RX_BUFFER[0]==0xFA)&&(MTI300_RX_BUFFER[1]==0xFF)&&(MTI300_RX_BUFFER[2]==0x36))
			{
				if((MTI300_RX_BUFFER[4]==0x20)&&(MTI300_RX_BUFFER[5]==0x30)&&(MTI300_RX_BUFFER[6]==0x0C))
				{
					eulerangles.Roll = uint32_to_float(&MTI300_RX_BUFFER[7]);
			        eulerangles.Pitch = uint32_to_float(&MTI300_RX_BUFFER[11]);
	                eulerangles.Yaw = uint32_to_float(&MTI300_RX_BUFFER[15]); 

				}
				
				if((MTI300_RX_BUFFER[19]==0x40)&&(MTI300_RX_BUFFER[20]==0x20)&&(MTI300_RX_BUFFER[21]==0x0C))
				{
					acceleration.accX = uint32_to_float(&MTI300_RX_BUFFER[22]);
	                acceleration.accY = uint32_to_float(&MTI300_RX_BUFFER[26]);
	                acceleration.accZ = uint32_to_float(&MTI300_RX_BUFFER[30]);
				}

				if((MTI300_RX_BUFFER[34]==0x80)&&(MTI300_RX_BUFFER[35]==0x20)&&(MTI300_RX_BUFFER[36]==0x0C))
				{
					 rateofturn.gyrX = uint32_to_float(&MTI300_RX_BUFFER[37]);
	                 rateofturn.gyrY = uint32_to_float(&MTI300_RX_BUFFER[41]);
	                 rateofturn.gyrZ = uint32_to_float(&MTI300_RX_BUFFER[45]);
				}
			}
			else
			{
				printf("incorrect data!\n");
			}
		}
	else
		printf("checksum failed!  \n");
}

void  Mti300::recv_data_auto_recovery(PortHandler *mti300_portHandler)  //add by lichunjing 2017-04-14
{
	uint32_t checksum = 0;
	uint8_t  checsum_result;

	while(1)
	{
		mti300_portHandler->readPort(MTI300_RX_BUFFER, 1);
		if(MTI300_RX_BUFFER[0] == 0xFA)
		{
			mti300_portHandler->readPort(&MTI300_RX_BUFFER[1], 1);
			if(MTI300_RX_BUFFER[1] == 0xFF)
			{
				mti300_portHandler->readPort(&MTI300_RX_BUFFER[2], 1);
				if(MTI300_RX_BUFFER[2] == 0x36)
				{
					mti300_portHandler->readPort(&MTI300_RX_BUFFER[3], 2);
					if(MTI300_RX_BUFFER[4] == 0x20)
					{
						mti300_portHandler->readPort(&MTI300_RX_BUFFER[5], 1);
						if(MTI300_RX_BUFFER[5] == 0x30)
						{
							mti300_portHandler->readPort(&MTI300_RX_BUFFER[6], 1);
							if(MTI300_RX_BUFFER[6] == 0x0C)
							{
								mti300_portHandler->readPort(&MTI300_RX_BUFFER[7], 12);  //attitude data
								mti300_portHandler->readPort(&MTI300_RX_BUFFER[19], 1);  
								if(MTI300_RX_BUFFER[19] == 0x40)
								{
									mti300_portHandler->readPort(&MTI300_RX_BUFFER[20], 1);
									if(MTI300_RX_BUFFER[20] == 0x20)
									{
										mti300_portHandler->readPort(&MTI300_RX_BUFFER[21], 1);
										if(MTI300_RX_BUFFER[21] == 0x0C)
										{
											mti300_portHandler->readPort(&MTI300_RX_BUFFER[22], 12); //acc data
											mti300_portHandler->readPort(&MTI300_RX_BUFFER[34], 1);
											if(MTI300_RX_BUFFER[34] == 0x80)
											{
												mti300_portHandler->readPort(&MTI300_RX_BUFFER[35], 1);
												if(MTI300_RX_BUFFER[35] == 0x20)
												{
													mti300_portHandler->readPort(&MTI300_RX_BUFFER[36], 1);
													if(MTI300_RX_BUFFER[36] == 0x0C)
													{
														mti300_portHandler->readPort(&MTI300_RX_BUFFER[37], 12); //gyr data
														mti300_portHandler->readPort(&MTI300_RX_BUFFER[49], 1);  //checksum

															for(uint16_t i=1;i<50;i++)
															   checksum += MTI300_RX_BUFFER[i];

															checsum_result = checksum &(0x00ff);
															if(checsum_result == 0)    //checksum OK!
															{
																eulerangles.Roll = uint32_to_float(&MTI300_RX_BUFFER[7]);
												                eulerangles.Pitch = uint32_to_float(&MTI300_RX_BUFFER[11]);
												                eulerangles.Yaw = uint32_to_float(&MTI300_RX_BUFFER[15]); 

												                acceleration.accX = uint32_to_float(&MTI300_RX_BUFFER[22]);
												                acceleration.accY = uint32_to_float(&MTI300_RX_BUFFER[26]);
												                acceleration.accZ = uint32_to_float(&MTI300_RX_BUFFER[30]);

							                					rateofturn.gyrX = uint32_to_float(&MTI300_RX_BUFFER[37]);
												                rateofturn.gyrY = uint32_to_float(&MTI300_RX_BUFFER[41]);
												                rateofturn.gyrZ = uint32_to_float(&MTI300_RX_BUFFER[45]);

												                break;
															}
															else
															{
																printf("checksum failed! \n");
																break;
															}
													}
													else
													{
														break;
													}
												}
												else
												{
													break;
												}
											}
											else
											{
												break;
											}
										}
										else
										{
											break;
										}
									}
									else
									{
										break;
									}
								}
								else
								{
									break;
								}

							}
							else
							{
								break;
							}
						}
						else
						{
							break;
						}
					}
					else
					{
						break;
					}
				}
				else
				{
					break;
				}
			}
			else
			{
				break;
			}
		}
		else
		{
			break;
		}
	}

}

void  Mti300::print_data(PortHandler *mti300_portHandler)
{
	printf("{Roll:%.4f\tPitch:%.4f\tYaw:%.4f}",eulerangles.Roll, eulerangles.Pitch, eulerangles.Yaw);
	printf("{accX:%.4f\taccY:%.4f\taccZ:%.4f}",acceleration.accX, acceleration.accY, acceleration.accZ);
	printf("{gyrX:%.4f\tgyrY:%.4f\tgyrZ:%.4f}",rateofturn.gyrX, rateofturn.gyrY, rateofturn.gyrZ);
	printf("\n");
}

void Mti300::update_to_gloable_var(gMti_Data *data)
{
	float offset = ((0 < eulerangles.Roll) - (eulerangles.Roll < 0))*180;
    if (offset == 0)
    {
            offset = 180;
	}
	eulerangles.Roll -= offset;
	data->eulerangles.Roll = eulerangles.Roll;
	data->eulerangles.Pitch = eulerangles.Pitch;
	data->eulerangles.Yaw = eulerangles.Yaw;

	data->acceleration.accX = acceleration.accX;
	data->acceleration.accY = acceleration.accY;
	data->acceleration.accZ = acceleration.accZ;

	data->rateofturn.gyrX = rateofturn.gyrX;
	data->rateofturn.gyrY = rateofturn.gyrY;
	data->rateofturn.gyrZ = rateofturn.gyrZ;
}