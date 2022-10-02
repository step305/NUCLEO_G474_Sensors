/*
 * packet.c
 *
 *  Created on: 30 дек. 2020 г.
 *      Author: driver
 */

#include "packet.h"
uint16_t pack_ADIS_data(ADIS_DATA_Type* data, unsigned char* buffer)
{
	Packet_union pack;
	uint16_t len;
	uint16_t i;
	uint16_t buf_len = sizeof(ReportPacketStructure);

	pack.data.e3[0] = data->rate[0];
	pack.data.e3[1] = data->rate[1];
	pack.data.e3[2] = data->rate[2];

	pack.data.e2[0] = data->adc[0];
	pack.data.e2[1] = data->adc[1];
	pack.data.e2[2] = data->adc[2];
	len = 0;

	buffer[len] = DLE;
	len++;
	buffer[len] = PACKET_CODE;
	len++;

	for(i=0; i< buf_len; i++)
	{
		if(pack.buff[i] == DLE)
		{
			buffer[len] = DLE;
			len++;
		}
		buffer[len] = pack.buff[i];
		len++;
	}

	buffer[len] = DLE;
	len++;
	buffer[len] = ETX;
	len++;
	return len;
}
