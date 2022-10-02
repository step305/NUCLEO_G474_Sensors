#ifndef __ADIS_16505_H__
#define __ADIS_16505_H__
uint16_t send_ADIS(uint8_t adr, uint8_t dat);
void write_ADIS(uint8_t adr, uint16_t dat);
void burst_read_ADIS(ADIS_DATA_Type* dat);

#endif

