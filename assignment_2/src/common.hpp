#pragma once
#include <Arduino.h>
#include "i2c.h"





#define G_ADDR 0x68
#define G_START_ADDR 29
#define G_INT_ADDR 26
#define G_INT_MASK 1

#define A_ADDR 0x53
#define A_START_ADDR 50
#define A_INT_ADDR 48
#define A_INT_MASK 128
#define M_ADDR 0x1E

#define CCW_THRESHOLD 100
#define CW_THRESHOLD -100

#define GYRO_SAMPLES 7
#define ACC_SAMPLES 7

volatile struct
{
    uint8_t TX_finished : 1;
    uint8_t TWI_ACK : 1;
    uint8_t sample : 1;
} twi_flags;

static volatile uint8_t RX_buff, servo_idx;
volatile uint8_t TWI_status, TWI_byte;
static volatile uint16_t time, delay_ms;
static const char CRLF[3] = {13, 10, 0};

static int16_t total_yaw, g_offset[3], a_offset[3] = {0, 0, 0}, acc[3];

static int16_t *gyro = new int16_t[3];


void little_endian(int16_t *data)
{
    uint8_t *p_data = (uint8_t *)data;
    uint8_t temp;
    for (uint8_t i = 0; i < 6; i += 2)
    {
        temp = *p_data;
        *p_data = *(p_data + 1);
        *(p_data + 1) = temp;
        p_data += 2;
    }
}

uint8_t gyro_init()
{
    Write_Reg(G_ADDR, 22, 0x18); // set DPLF to full-scale. For ITG-3200 ONLY!!!!
    Write_Reg(G_ADDR, 23, 1);    // Enable RAW data INT. For ITG-3200 ONLY!!!!
    Write_Reg(G_ADDR, 62, 3);    // set clock to Z. For ITG-3200 ONLY!!!!
    uint8_t ret_status;
    // Gyro warm-up=======================
    for (uint8_t i = 100; i > 0; i--)
    {
        do
        {
            Read_Reg(G_ADDR, G_INT_ADDR);
        } while (!(TWI_byte & 1)); // waiting for data to be ready
        ret_status = Read_Reg_N(G_ADDR, G_START_ADDR, 6, gyro);
        if (ret_status)
        {
#ifdef DEBUG
            char err[] = "ERROR\n\r";
            send_string((uint8_t *)err);
#endif
            return (ret_status);
        }
    } // end warm-up

    // Gyro offset values
    int16_t x_acc = 0, y_acc = 0, z_acc = 0;

    for (uint8_t i = (1 << GYRO_SAMPLES); i > 0; i--)
    {

        do
        {
            Read_Reg(G_ADDR, G_INT_ADDR);
        } while (!(TWI_byte & 1)); // waiting for data to be ready

        ret_status = Read_Reg_N(G_ADDR, G_START_ADDR, 6, gyro);
        if (ret_status)
        {
#ifdef DEBUG
            char err[] = "ERROR\n\r";
            send_string((uint8_t *)err);
#endif
            return (ret_status);
        } // end if
        little_endian((int16_t *)gyro);
        x_acc += gyro[0];
        y_acc += gyro[1];
        z_acc += gyro[2];
    } // end for
    *g_offset = (int16_t)(x_acc >> GYRO_SAMPLES);
    *(g_offset + 1) = (int16_t)(y_acc >> GYRO_SAMPLES);
    *(g_offset + 2) = (int16_t)(z_acc >> GYRO_SAMPLES);
#ifdef DEBUG
    char offs[] = "Offsets:\r\n";
    send_string((uint8_t *)offs);
    send_reading(*g_offset, "Xoff=");
    send_reading(*(g_offset + 1), "Yoff=");
    send_reading(*(g_offset + 2), "Zoff=");
#endif
    return (0);
}

uint8_t read_avg_sensor(uint8_t sensor_addr,
                        uint8_t start_register,
                        int16_t *dest_array,
                        volatile int16_t *offset,
                        uint8_t samples2,
                        uint8_t big_endian,
                        uint8_t irq_reg,
                        uint8_t irq_mask)
{
    int16_t x_acc = 0, y_acc = 0, z_acc = 0;
    uint8_t ret_status;

    for (uint8_t i = (1 << samples2); i > 0; i--)
    {
        do
        {
            Read_Reg(sensor_addr, irq_reg);
        } while (!(TWI_byte & irq_mask)); // waiting for data to be ready
        ret_status = Read_Reg_N(sensor_addr, start_register, 6, dest_array);
        if (ret_status)
        {
#ifdef DEBUG
            char err[] = "ERROR\n\r";
            send_string((uint8_t *)err);
#endif
            return (ret_status);
        } // end if
        if (big_endian)
            little_endian((int16_t *)dest_array);
        x_acc += *dest_array;
        y_acc += *(dest_array + 1);
        z_acc += *(dest_array + 2);
    } // end for
    *dest_array = (int16_t)(x_acc >> samples2) - *offset++;
    *(dest_array + 1) = (int16_t)(y_acc >> samples2) - *offset++;
    *(dest_array + 2) = (int16_t)(z_acc >> samples2) - *offset;
#ifdef DEBUG
    char offs[] = "----:\r\n";
    send_string((uint8_t *)offs);
    send_reading(*(dest_array), "X=");
    send_reading(*(dest_array + 1), "Y=");
    send_reading(*(dest_array + 2), "Z=");
#endif
    return (0);
} // end filter



int16_t compl_filter (int16_t old_yaw){
	int16_t yaw;
	yaw=(int16_t)(0.98*((float)old_yaw+(float)gyro[2]*0.02)+0.02*(float)acc[2]);
	return (yaw);
}