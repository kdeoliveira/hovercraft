#ifndef I2C_H
#define I2C_H   1

#include <util/twi.h>

struct i2cflags{
	uint8_t TX_new_data:1;
	uint8_t TX_finished:1;
	uint8_t TX_buffer1_empty:1;
	uint8_t TX_buffer2_empty:1;
	uint8_t RX_flag:3;
	uint8_t TWI_ACK:1;
};




uint8_t TWI_start(uint8_t twi_addr, uint8_t read_write);

void TWI_stop(void);

uint8_t TWI_write(uint8_t tx_data); //write byte to the started device

uint8_t TWI_ack_read(void); // continuous read

uint8_t TWI_nack_read(void); //read and stop condition

uint8_t Read_Reg(uint8_t TWI_addr, uint8_t reg_addr);

uint8_t Read_Reg_N(uint8_t TWI_addr, uint8_t reg_addr, uint8_t bytes, int16_t* data);

uint8_t Write_Reg(uint8_t TWI_addr, uint8_t reg_addr, uint8_t value);


#endif