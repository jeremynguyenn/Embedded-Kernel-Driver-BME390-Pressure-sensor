/*
 * bmp390.h - Header for BMP390 kernel driver
 * Based on LibDriver user-space code.
 * Copyright (c) 2025, Your Name. MIT License.
 */

#ifndef __BMP390_H__
#define __BMP390_H__

#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/interrupt.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/mutex.h>
#include <linux/delay.h>

/* Registers from driver_bmp390.c */
#define BMP390_REG_CHIP_ID          0x00
#define BMP390_REG_REV_ID           0x01
#define BMP390_REG_ERR              0x02
#define BMP390_REG_STATUS           0x03
#define BMP390_REG_EVENT            0x04
#define BMP390_REG_INT_STATUS       0x05
#define BMP390_REG_FIFO_STATUS_1    0x07
#define BMP390_REG_FIFO_STATUS_2    0x08
#define BMP390_REG_SENSOR_TIME_0    0x0C
#define BMP390_REG_SENSOR_TIME_1    0x0D
#define BMP390_REG_SENSOR_TIME_2    0x0E
#define BMP390_REG_SENSOR_TIME_3    0x0F
#define BMP390_REG_PRESSURE_DATA_0  0x04  /* In forced mode */
#define BMP390_REG_PRESSURE_DATA_1  0x05
#define BMP390_REG_PRESSURE_DATA_2  0x06
#define BMP390_REG_PRESSURE_DATA_3  0x07
#define BMP390_REG_TEMP_DATA_0      0x08
#define BMP390_REG_TEMP_DATA_1      0x09
#define BMP390_REG_TEMP_DATA_2      0x0A
#define BMP390_REG_TEMP_DATA_3      0x0B
#define BMP390_REG_CMD              0x7E
#define BMP390_REG_CONFIG           0x1F
#define BMP390_REG_ODR              0x1D
#define BMP390_REG_OSR              0x1C  /* Oversampling */
#define BMP390_REG_PWR_CTRL         0x1B
#define BMP390_REG_FIFO_CONFIG_0    0x44
#define BMP390_REG_FIFO_CONFIG_1    0x45
#define BMP390_REG_INT_CTRL         0x19
#define BMP390_REG_NVM_PAR_T1_L     0x31
#define BMP390_REG_NVM_PAR_T1_H     0x32
#define BMP390_REG_NVM_PAR_T2_L     0x33
#define BMP390_REG_NVM_PAR_T2_H     0x34
#define BMP390_REG_NVM_PAR_T3       0x35
#define BMP390_REG_NVM_PAR_P1_L     0x36
#define BMP390_REG_NVM_PAR_P1_H     0x37
#define BMP390_REG_NVM_PAR_P2_L     0x38
#define BMP390_REG_NVM_PAR_P2_H     0x39
#define BMP390_REG_NVM_PAR_P3       0x3A
#define BMP390_REG_NVM_PAR_P4       0x3B
#define BMP390_REG_NVM_PAR_P5_L     0x3C
#define BMP390_REG_NVM_PAR_P5_H     0x3D
#define BMP390_REG_NVM_PAR_P6_L     0x3E
#define BMP390_REG_NVM_PAR_P6_H     0x3F
#define BMP390_REG_NVM_PAR_P7       0x40
#define BMP390_REG_NVM_PAR_P8       0x41
#define BMP390_REG_NVM_PAR_P9_L     0x42
#define BMP390_REG_NVM_PAR_P9_H     0x43
#define BMP390_REG_NVM_PAR_P10      0x44
#define BMP390_REG_NVM_PAR_P11      0x45

#define BMP390_CHIP_ID              0x50
#define BMP390_CMD_SOFT_RESET       0xB6
#define BMP390_MODE_SLEEP           0x00
#define BMP390_MODE_FORCED          0x01
#define BMP390_MODE_NORMAL          0x03

/* Enums from driver_bmp390.h */
enum bmp390_oversampling {
    BMP390_OVERSAMPLING_x1 = 0,
    BMP390_OVERSAMPLING_x2,
    BMP390_OVERSAMPLING_x4,
    BMP390_OVERSAMPLING_x8,
    BMP390_OVERSAMPLING_x16,
    BMP390_OVERSAMPLING_x32,
};

enum bmp390_odr {
    BMP390_ODR_0P78_HZ = 0,
    BMP390_ODR_1P5_HZ,
    BMP390_ODR_3_HZ,
    BMP390_ODR_6_HZ,
    BMP390_ODR_12P5_HZ,
    BMP390_ODR_25_HZ,
    BMP390_ODR_50_HZ,
    BMP390_ODR_100_HZ,
};

enum bmp390_filter_coeff {
    BMP390_FILTER_COEFFICIENT_0 = 0,
    BMP390_FILTER_COEFFICIENT_1,
    BMP390_FILTER_COEFFICIENT_3,
    BMP390_FILTER_COEFFICIENT_7,
    BMP390_FILTER_COEFFICIENT_15,
    BMP390_FILTER_COEFFICIENT_31,
    BMP390_FILTER_COEFFICIENT_63,
    BMP390_FILTER_COEFFICIENT_127,
};

enum bmp390_fifo_data_source {
    BMP390_FIFO_DATA_SOURCE_FILTERED = 0,
    BMP390_FIFO_DATA_SOURCE_UNFILTERED,
};

enum bmp390_interrupt_pin_type {
    BMP390_INTERRUPT_PIN_TYPE_PUSH_PULL = 0,
    BMP390_INTERRUPT_PIN_TYPE_OPEN_DRAIN,
};

enum bmp390_interrupt_active_level {
    BMP390_INTERRUPT_ACTIVE_LEVEL_LOW = 0,
    BMP390_INTERRUPT_ACTIVE_LEVEL_HIGH,
};

enum bmp390_iic_watchdog_period {
    BMP390_IIC_WATCHDOG_PERIOD_1P25_MS = 0,
    BMP390_IIC_WATCHDOG_PERIOD_40_MS,
};

enum bmp390_spi_wire {
    BMP390_SPI_WIRE_4 = 0,
    BMP390_SPI_WIRE_3,
};

/* Calibration params struct (from compensation in user-space) */
struct bmp390_calib_param {
    int16_t t1, t2;
    uint16_t t3;
    int16_t p1, p2;
    uint16_t p3, p4;
    int16_t p5, p6;
    uint16_t p7, p8, p9, p10, p11;
};

/* Device private data */
struct bmp390_data {
    struct i2c_client *client;  /* For I2C */
    struct spi_device *spi;     /* For SPI */
    struct mutex lock;
    struct regulator *vdd;
    struct bmp390_calib_param calib;
    u8 chip_id;
    enum bmp390_oversampling osr_p, osr_t;
    enum bmp390_odr odr;
    enum bmp390_filter_coeff filter;
    u8 mode;
    u8 irq_enabled;
    struct work_struct data_work;  /* For bottom-half IRQ */
    struct iio_dev *indio_dev;
    /* FIFO */
    u16 fifo_length;
    u8 fifo_watermark;
    enum bmp390_fifo_data_source fifo_source;
    /* Interrupt */
    enum bmp390_interrupt_pin_type int_pin_type;
    enum bmp390_interrupt_active_level int_level;
    enum bmp390_spi_wire spi_wire;
    enum bmp390_iic_watchdog_period watchdog_period;
};

/* Function prototypes */
int bmp390_read_reg(struct bmp390_data *data, u8 reg, u8 *val, u32 len);
int bmp390_write_reg(struct bmp390_data *data, u8 reg, const u8 *val, u32 len);
int bmp390_soft_reset(struct bmp390_data *data);
int bmp390_set_mode(struct bmp390_data *data, u8 mode);
int bmp390_read_temperature_pressure(struct bmp390_data *data, s32 *temp_raw, s32 *press_raw, s32 *temp, s32 *press);
int bmp390_load_calib(struct bmp390_data *data);
void bmp390_compensate_temp_press(struct bmp390_data *data, s32 adc_temp, s32 adc_press, s32 *temp, s32 *press);
int bmp390_set_oversampling(struct bmp390_data *data, enum bmp390_oversampling osr_p, enum bmp390_oversampling osr_t);
int bmp390_set_odr(struct bmp390_data *data, enum bmp390_odr odr);
int bmp390_set_filter_coefficient(struct bmp390_data *data, enum bmp390_filter_coeff coeff);
int bmp390_set_fifo_watermark(struct bmp390_data *data, u8 watermark);
int bmp390_set_fifo_data_source(struct bmp390_data *data, enum bmp390_fifo_data_source source);
int bmp390_flush_fifo(struct bmp390_data *data);
int bmp390_read_fifo(struct bmp390_data *data, u8 *buf, u16 *len);
int bmp390_set_interrupt_data_ready(struct bmp390_data *data, bool enable);
int bmp390_set_interrupt_pin_type(struct bmp390_data *data, enum bmp390_interrupt_pin_type type);
int bmp390_set_interrupt_active_level(struct bmp390_data *data, enum bmp390_interrupt_active_level level);
int bmp390_set_spi_wire(struct bmp390_data *data, enum bmp390_spi_wire wire);
int bmp390_set_iic_watchdog_period(struct bmp390_data *data, enum bmp390_iic_watchdog_period period);

/* IIO channels */
enum bmp390_channel {
    BMP390_CHANNEL_TEMP,
    BMP390_CHANNEL_PRESS,
};

#endif /* __BMP390_H__ */