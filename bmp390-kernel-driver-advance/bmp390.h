/*
 * bmp390.h - Header for BMP390 kernel driver
 * Author: Nguyen Nhan
 * Copyright (c) 2025, Nguyen Nhan. MIT License.
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
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/sched/signal.h>
#include <linux/uaccess.h>
#include <linux/pm_runtime.h>
#include <linux/wait.h>

/* Registers */
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
#define BMP390_REG_PRESSURE_DATA_0  0x04
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
#define BMP390_REG_OSR              0x1C
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
#define BMP390_REG_FIFO_DATA        0x14

#define BMP390_CHIP_ID              0x50
#define BMP390_CMD_SOFT_RESET       0xB6
#define BMP390_MODE_SLEEP           0x00
#define BMP390_MODE_FORCED          0x01
#define BMP390_MODE_NORMAL          0x03
#define BMP390_INTERRUPT_STATUS_DATA_READY 0x01
#define BMP390_INTERRUPT_STATUS_FIFO_WATERMARK 0x02
#define BMP390_INTERRUPT_STATUS_FIFO_FULL 0x04

/* Enums */
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

enum bmp390_operation_mode {
    BMP390_OP_MODE_DEFAULT,
    BMP390_OP_MODE_PERIODIC,
    BMP390_OP_MODE_ASYNC,
};

/* Calibration parameters */
struct bmp390_calib_param {
    int16_t t1, t2;
    uint16_t t3;
    int16_t p1, p2;
    uint16_t p3, p4;
    int16_t p5, p6;
    uint16_t p7, p8, p9, p10, p11;
};

/* Shared memory structure for IPC */
struct bmp390_shared_data {
    s32 temperature;
    s32 pressure;
    u64 timestamp;
    u8 status;
};

/* IOCTL commands */
#define BMP390_IOC_MAGIC 'b'
#define BMP390_IOC_SET_MODE _IOW(BMP390_IOC_MAGIC, 1, int)
#define BMP390_IOC_GET_SHARED_DATA _IOR(BMP390_IOC_MAGIC, 2, struct bmp390_shared_data)
#define BMP390_IOC_FLUSH_FIFO _IO(BMP390_IOC_MAGIC, 3)
#define BMP390_IOC_SET_ODR _IOW(BMP390_IOC_MAGIC, 4, int)
#define BMP390_IOC_GET_CALIB _IOR(BMP390_IOC_MAGIC, 5, struct bmp390_calib_param)

/* Device private data */
struct bmp390_data {
    struct i2c_client *client;
    struct spi_device *spi;
    struct mutex lock;
    struct regulator *vdd;
    struct bmp390_calib_param calib;
    u8 chip_id;
    enum bmp390_oversampling osr_p, osr_t;
    enum bmp390_odr odr;
    enum bmp390_filter_coeff filter;
    enum bmp390_operation_mode op_mode;
    u8 mode;
    u8 irq_enabled;
    struct work_struct data_work;
    struct iio_dev *indio_dev;
    u16 fifo_length;
    u8 fifo_watermark;
    enum bmp390_fifo_data_source fifo_source;
    enum bmp390_interrupt_pin_type int_pin_type;
    enum bmp390_interrupt_active_level int_level;
    enum bmp390_spi_wire spi_wire;
    enum bmp390_iic_watchdog_period watchdog_period;
    struct task_struct *kthread;
    struct completion data_ready;
    wait_queue_head_t wait_queue;
    struct bmp390_shared_data *shared_mem;
    struct file *sigio_file;
    bool pm_enabled;
};

/* IIO channels */
static const struct iio_chan_spec bmp390_channels[] = {
    {
        .type = IIO_TEMP,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_PROCESSED) |
                              BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),
        .scan_index = BMP390_CHANNEL_TEMP,
        .scan_type = {
            .sign = 's',
            .realbits = 32,
            .storagebits = 32,
            .shift = 0,
        },
    },
    {
        .type = IIO_PRESSURE,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_PROCESSED) |
                              BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),
        .scan_index = BMP390_CHANNEL_PRESS,
        .scan_type = {
            .sign = 's',
            .realbits = 32,
            .storagebits = 32,
            .shift = 0,
        },
    },
    IIO_CHAN_SOFT_TIMESTAMP(2),
};

/* IIO info structure */
static const struct iio_info bmp390_info = {
    .read_raw = bmp390_read_raw,
    .write_raw = bmp390_write_raw,
    .attrs = &bmp390_attribute_group,
};

/* Function prototypes with kernel-doc */

/**
 * bmp390_read_reg - Read from BMP390 register
 * @data: Device data structure
 * @reg: Register address
 * @val: Buffer to store read data
 * @len: Number of bytes to read
 * Return: 0 on success, negative error code on failure
 */
int bmp390_read_reg(struct bmp390_data *data, u8 reg, u8 *val, u32 len);

/**
 * bmp390_write_reg - Write to BMP390 register
 * @data: Device data structure
 * @reg: Register address
 * @val: Data to write
 * @len: Number of bytes to write
 * Return: 0 on success, negative error code on failure
 */
int bmp390_write_reg(struct bmp390_data *data, u8 reg, const u8 *val, u32 len);

/**
 * bmp390_soft_reset - Perform soft reset
 * @data: Device data structure
 * Return: 0 on success, negative error code on failure
 */
int bmp390_soft_reset(struct bmp390_data *data);

/**
 * bmp390_set_mode - Set operating mode
 * @data: Device data structure
 * @mode: Mode (BMP390_MODE_SLEEP, BMP390_MODE_FORCED, BMP390_MODE_NORMAL)
 * Return: 0 on success, negative error code on failure
 */
int bmp390_set_mode(struct bmp390_data *data, u8 mode);

/**
 * bmp390_read_temperature_pressure - Read temperature and pressure
 * @data: Device data structure
 * @temp_raw: Raw temperature output
 * @press_raw: Raw pressure output
 * @temp: Compensated temperature output
 * @press: Compensated pressure output
 * Return: 0 on success, negative error code on failure
 */
int bmp390_read_temperature_pressure(struct bmp390_data *data, s32 *temp_raw, s32 *press_raw, s32 *temp, s32 *press);

/**
 * bmp390_load_calib - Load calibration parameters
 * @data: Device data structure
 * Return: 0 on success, negative error code on failure
 */
int bmp390_load_calib(struct bmp390_data *data);

/**
 * bmp390_compensate_temp_press - Compensate raw temperature and pressure
 * @data: Device data structure
 * @adc_temp: Raw temperature
 * @adc_press: Raw pressure
 * @temp: Compensated temperature output
 * @press: Compensated pressure output
 */
void bmp390_compensate_temp_press(struct bmp390_data *data, s32 adc_temp, s32 adc_press, s32 *temp, s32 *press);

/**
 * bmp390_set_oversampling - Set oversampling ratios
 * @data: Device data structure
 * @osr_p: Pressure oversampling
 * @osr_t: Temperature oversampling
 * Return: 0 on success, negative error code on failure
 */
int bmp390_set_oversampling(struct bmp390_data *data, enum bmp390_oversampling osr_p, enum bmp390_oversampling osr_t);

/**
 * bmp390_set_odr - Set output data rate
 * @data: Device data structure
 * @odr: Output data rate
 * Return: 0 on success, negative error code on failure
 */
int bmp390_set_odr(struct bmp390_data *data, enum bmp390_odr odr);

/**
 * bmp390_set_filter_coefficient - Set IIR filter coefficient
 * @data: Device data structure
 * @coeff: Filter coefficient
 * Return: 0 on success, negative error code on failure
 */
int bmp390_set_filter_coefficient(struct bmp390_data *data, enum bmp390_filter_coeff coeff);

/**
 * bmp390_set_fifo_watermark - Set FIFO watermark
 * @data: Device data structure
 * @watermark: FIFO watermark value
 * Return: 0 on success, negative error code on failure
 */
int bmp390_set_fifo_watermark(struct bmp390_data *data, u8 watermark);

/**
 * bmp390_set_fifo_data_source - Set FIFO data source
 * @data: Device data structure
 * @source: FIFO data source
 * Return: 0 on success, negative error code on failure
 */
int bmp390_set_fifo_data_source(struct bmp390_data *data, enum bmp390_fifo_data_source source);

/**
 * bmp390_flush_fifo - Flush FIFO
 * @data: Device data structure
 * Return: 0 on success, negative error code on failure
 */
int bmp390_flush_fifo(struct bmp390_data *data);

/**
 * bmp390_read_fifo - Read FIFO data
 * @data: Device data structure
 * @buf: Buffer for FIFO data
 * @len: Length of data read
 * Return: 0 on success, negative error code on failure
 */
int bmp390_read_fifo(struct bmp390_data *data, u8 *buf, u16 *len);

/**
 * bmp390_set_interrupt_data_ready - Enable/disable data ready interrupt
 * @data: Device data structure
 * @enable: Enable flag
 * Return: 0 on success, negative error code on failure
 */
int bmp390_set_interrupt_data_ready(struct bmp390_data *data, bool enable);

/**
 * bmp390_set_interrupt_pin_type - Set interrupt pin type
 * @data: Device data structure
 * @type: Pin type
 * Return: 0 on success, negative error code on failure
 */
int bmp390_set_interrupt_pin_type(struct bmp390_data *data, enum bmp390_interrupt_pin_type type);

/**
 * bmp390_set_interrupt_active_level - Set interrupt active level
 * @data: Device data structure
 * @level: Active level
 * Return: 0 on success, negative error code on failure
 */
int bmp390_set_interrupt_active_level(struct bmp390_data *data, enum bmp390_interrupt_active_level level);

/**
 * bmp390_set_spi_wire - Set SPI wire mode
 * @data: Device data structure
 * @wire: SPI wire mode
 * Return: 0 on success, negative error code on failure
 */
int bmp390_set_spi_wire(struct bmp390_data *data, enum bmp390_spi_wire wire);

/**
 * bmp390_set_iic_watchdog_period - Set I2C watchdog period
 * @data: Device data structure
 * @period: Watchdog period
 * Return: 0 on success, negative error code on failure
 */
int bmp390_set_iic_watchdog_period(struct bmp390_data *data, enum bmp390_iic_watchdog_period period);

/**
 * bmp390_init_kthread - Initialize kernel thread
 * @data: Device data structure
 * Return: 0 on success, negative error code on failure
 */
int bmp390_init_kthread(struct bmp390_data *data);

/**
 * bmp390_stop_kthread - Stop kernel thread
 * @data: Device data structure
 */
void bmp390_stop_kthread(struct bmp390_data *data);

/**
 * bmp390_setup_shared_memory - Setup shared memory for IPC
 * @data: Device data structure
 * Return: 0 on success, negative error code on failure
 */
int bmp390_setup_shared_memory(struct bmp390_data *data);

/**
 * bmp390_cleanup_shared_memory - Cleanup shared memory
 * @data: Device data structure
 */
void bmp390_cleanup_shared_memory(struct bmp390_data *data);

/**
 * bmp390_ioctl - IOCTL handler
 * @filp: File pointer
 * @cmd: IOCTL command
 * @arg: IOCTL argument
 * Return: 0 on success, negative error code on failure
 */
long bmp390_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

#endif /* __BMP390_H__ */