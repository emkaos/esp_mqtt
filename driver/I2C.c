/*********************************************************************************
Copyright (c) 2016, Cosmin Plasoianu
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 ********************************************************************************/

#include "ets_sys.h"
#include "osapi.h"
#include "driver/I2C.h"

uint8 ICACHE_FLASH_ATTR I2C_WriteData(uint8 device_addr, uint8 register_addr, uint8* data, uint8 length)
{
	uint8 cnt = 0;

	i2c_master_start();						//Start

	device_addr &= 0xFE;					//Device address
	i2c_master_writeByte(device_addr);
	if(!i2c_master_checkAck())
	{
		i2c_master_stop();
		return 1;
	}

	i2c_master_writeByte(register_addr);	//Register address
	if(!i2c_master_checkAck())
	{
		i2c_master_stop();
		return 2;
	}

	while(cnt != length)					//Data
	{
		i2c_master_writeByte(data[cnt++]);

		if(!i2c_master_checkAck())
		{
			i2c_master_stop();
			return 3;
		}
	}

	i2c_master_stop();						//Stop

	return 0;
}

uint8 ICACHE_FLASH_ATTR I2C_ReadData(uint8 device_addr, uint8 register_addr, uint8* data, uint8 length)
{
	uint8 cnt;

	i2c_master_start();						//Start

	device_addr &= 0xFE;					//Device address (W)
	i2c_master_writeByte(device_addr);
	if(!i2c_master_checkAck())
	{
		i2c_master_stop();
		INFO("no ack 1");
		return 1;
	}

	i2c_master_writeByte(register_addr);	//Register address
	if(!i2c_master_checkAck())
	{
		i2c_master_stop();
		INFO("no ack 2");
		return 2;
	}

	i2c_master_start();						//Start

	device_addr |= 0x01;					//Device address (R)
	i2c_master_writeByte(device_addr);
	if(!i2c_master_checkAck())
	{
		i2c_master_stop();
		INFO("no ack 3");
		return 3;
	}

	if(length)
	{
		if(length > 1)							//Data
		{
			for(cnt = 0; cnt < length - 1; cnt++)
			{
				data[cnt] = i2c_master_readByte();
				i2c_master_send_ack();
			}
		}

		data[length - 1] = i2c_master_readByte();
		i2c_master_send_nack();
	}

	i2c_master_stop();						//Stop

	return 0;
}
