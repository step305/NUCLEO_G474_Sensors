#ifndef __PACKET_H_
#define __PACKET_H_
#include "main.h"


#define DLE 	(0x10)
#define ETX 	(0x03)
#define MAX_BUF (512)
#define PACKET_CODE	0xAA

#pragma pack(1)
typedef struct{
	float e1[3];
	float e2[3];
    float e3[3];
    float e4[3];
    float e5[3];
    unsigned char check_sum;
} ReportPacketStructure;
#pragma pack()

#pragma pack(1)
	typedef union Packet_union_t
	{
		ReportPacketStructure data;
		char buff[sizeof(ReportPacketStructure)];
	}Packet_union;
#pragma pack()

uint16_t pack_ADIS_data(ADIS_DATA_Type* data, unsigned char* buffer);

#endif
