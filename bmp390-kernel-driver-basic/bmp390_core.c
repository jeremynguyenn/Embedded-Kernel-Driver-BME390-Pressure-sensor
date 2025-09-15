/*
 * bmp390_core.c - Core functions for BMP390 driver
 * Adapted from LibDriver user-space code.
 */

#include "bmp390.h"
#include <linux/math64.h>

static int bmp390_read_reg(struct bmp390_data *data, u8 reg, u8 *val, u32 len)
{
    int ret;
    struct device *dev = data->indio_dev->dev.parent;

    mutex_lock(&data->lock);

    if (data->client) {  /* I2C */
        ret = i2c_smbus_read_i2c_block_data(data->client, reg, len, val);
    } else {  /* SPI */
        u8 txbuf[1] = { reg | 0x80 };  /* SPI read flag from user-space spi.c */
        ret = spi_write_then_read(data->spi, txbuf, 1, val, len);
    }

    mutex_unlock(&data->lock);
    if (ret < 0) {
        dev_err(dev, "Failed to read reg 0x%02x\n", reg);
        return ret;
    }
    return 0;
}

static int bmp390_write_reg(struct bmp390_data *data, u8 reg, const u8 *val, u32 len)
{
    int ret;
    struct device *dev = data->indio_dev->dev.parent;
    u8 buf[32];

    mutex_lock(&data->lock);

    if (len > sizeof(buf) - 1) {
        ret = -EINVAL;
        goto out;
    }

    if (data->client) {  /* I2C */
        buf[0] = reg;
        memcpy(&buf[1], val, len);
        ret = i2c_master_send(data->client, buf, len + 1);
    } else {  /* SPI */
        buf[0] = reg & 0x7F;  /* SPI write flag */
        memcpy(&buf[1], val, len);
        ret = spi_write(data->spi, buf, len + 1);
    }

    if (ret < 0)
        dev_err(dev, "Failed to write reg 0x%02x\n", reg);
out:
    mutex_unlock(&data->lock);
    return ret < 0 ? ret : 0;
}

int bmp390_soft_reset(struct bmp390_data *data)
{
    u8 cmd = BMP390_CMD_SOFT_RESET;
    int ret = bmp390_write_reg(data, BMP390_REG_CMD, &cmd, 1);
    if (ret)
        return ret;
    msleep(2);  /* Delay from user-space */
    return 0;
}

int bmp390_set_mode(struct bmp390_data *data, u8 mode)
{
    u8 val;
    int ret = bmp390_read_reg(data, BMP390_REG_PWR_CTRL, &val, 1);
    if (ret)
        return ret;
    val = (val & ~0x30) | (mode << 4);  /* Bits 4-5 for mode from user-space */
    ret = bmp390_write_reg(data, BMP390_REG_PWR_CTRL, &val, 1);
    if (ret == 0)
        data->mode = mode;
    return ret;
}

int bmp390_load_calib(struct bmp390_data *data)
{
    u8 buf[21];  /* Full 21 bytes for NVM_PAR from driver_bmp390.c */
    int ret = bmp390_read_reg(data, BMP390_REG_NVM_PAR_T1_L, buf, 21);
    if (ret)
        return ret;

    data->calib.t1 = (buf[1] << 8) | buf[0];
    data->calib.t2 = (buf[3] << 8) | buf[2];
    data->calib.t3 = buf[4];
    data->calib.p1 = (buf[6] << 8) | buf[5];
    data->calib.p2 = (buf[8] << 8) | buf[7];
    data->calib.p3 = buf[9];
    data->calib.p4 = buf[10];
    data->calib.p5 = (buf[12] << 8) | buf[11];
    data->calib.p6 = (buf[14] << 8) | buf[13];
    data->calib.p7 = buf[15];
    data->calib.p8 = buf[16];
    data->calib.p9 = (buf[18] << 8) | buf[17];
    data->calib.p10 = buf[19];
    data->calib.p11 = buf[20];

    return 0;
}

void bmp390_compensate_temp_press(struct bmp390_data *data, s32 adc_temp, s32 adc_press, s32 *temp, s32 *press)
{
    s64 temp_var1, temp_var2, temp_var3;
    s64 press_var1, press_var2, press_var3, press_var4, press_var5, press_var6;
    s64 t_lin, p_lin;

    /* Temperature compensation from driver_bmp390.c */
    temp_var1 = ((s64)adc_temp) - ((s64)data->calib.t1 << 8);
    temp_var2 = temp_var1 * ((s64)data->calib.t2);
    temp_var3 = (temp_var1 * temp_var1) >> 1;
    temp_var3 = (temp_var3 * ((s64)data->calib.t3)) >> 12;
    t_lin = (temp_var2 + temp_var3) >> 14;
    *temp = (s32)t_lin;

    /* Pressure compensation - full poly from user-space */
    press_var1 = (((s64)t_lin) * ((s64)t_lin)) >> 1;
    press_var1 = (press_var1 * ((s64)data->calib.p3)) >> 13;
    press_var2 = ((s64)data->calib.p2 * (s64)t_lin) >> 1;
    press_var3 = ((s64)data->calib.p1 << 17);
    press_var4 = press_var1 + press_var2 + press_var3;
    press_var5 = ((s64)adc_press * press_var4) >> 6;
    press_var6 = press_var5 >> 2;
    press_var1 = ((s64)data->calib.p6 * press_var6) >> 31;
    press_var2 = ((s64)data->calib.p5 * press_var5) >> 1;
    press_var3 = ((s64)data->calib.p4 << 35);
    press_var4 = press_var1 + press_var2 + press_var3;
    press_var5 = ((s64)data->calib.p8 * (s64)adc_press) >> 3;
    press_var6 = ((s64)data->calib.p7 << 4);
    press_var1 = press_var5 + press_var6;
    press_var2 = (press_var1 * (s64)adc_press) >> 5;
    press_var3 = ((s64)data->calib.p10 * (s64)adc_press) >> 1;
    press_var1 = press_var2 + press_var3;
    press_var2 = ((s64)data->calib.p9 << 6);
    press_var3 = press_var1 + press_var2;
    press_var1 = (press_var3 * (s64)adc_press) >> 13;
    press_var2 = ((s64)data->calib.p11 << 15);
    p_lin = press_var4 + press_var1 + press_var2;
    *press = (s32)p_lin;
}

int bmp390_read_temperature_pressure(struct bmp390_data *data, s32 *temp_raw, s32 *press_raw, s32 *temp, s32 *press)
{
    u8 buf[6];
    int ret;

    ret = bmp390_read_reg(data, BMP390_REG_PRESSURE_DATA_0, buf, 3);
    if (ret)
        return ret;
    *press_raw = (buf[2] << 16) | (buf[1] << 8) | buf[0];

    ret = bmp390_read_reg(data, BMP390_REG_TEMP_DATA_0, buf, 3);
    if (ret)
        return ret;
    *temp_raw = (buf[2] << 16) | (buf[1] << 8) | buf[0];

    bmp390_compensate_temp_press(data, *temp_raw, *press_raw, temp, press);

    return 0;
}

int bmp390_set_oversampling(struct bmp390_data *data, enum bmp390_oversampling osr_p, enum bmp390_oversampling osr_t)
{
    u8 val = (osr_p & 0x07) | ((osr_t & 0x07) << 3);
    int ret = bmp390_write_reg(data, BMP390_REG_OSR, &val, 1);
    if (ret == 0) {
        data->osr_p = osr_p;
        data->osr_t = osr_t;
    }
    return ret;
}

int bmp390_set_odr(struct bmp390_data *data, enum bmp390_odr odr)
{
    u8 val = odr & 0x0F;  /* Bits 0-3 */
    int ret = bmp390_write_reg(data, BMP390_REG_ODR, &val, 1);
    if (ret == 0)
        data->odr = odr;
    return ret;
}

int bmp390_set_filter_coefficient(struct bmp390_data *data, enum bmp390_filter_coeff coeff)
{
    u8 val;
    int ret = bmp390_read_reg(data, BMP390_REG_CONFIG, &val, 1);
    if (ret)
        return ret;
    val = (val & ~0x07) | (coeff & 0x07);
    ret = bmp390_write_reg(data, BMP390_REG_CONFIG, &val, 1);
    if (ret == 0)
        data->filter = coeff;
    return ret;
}

int bmp390_set_fifo_watermark(struct bmp390_data *data, u8 watermark)
{
    u8 val = watermark;
    int ret = bmp390_write_reg(data, BMP390_REG_FIFO_CONFIG_0, &val, 1);
    if (ret == 0)
        data->fifo_watermark = watermark;
    return ret;
}

int bmp390_set_fifo_data_source(struct bmp390_data *data, enum bmp390_fifo_data_source source)
{
    u8 val;
    int ret = bmp390_read_reg(data, BMP390_REG_FIFO_CONFIG_1, &val, 1);
    if (ret)
        return ret;
    val = (val & ~0x01) | (source & 0x01);
    ret = bmp390_write_reg(data, BMP390_REG_FIFO_CONFIG_1, &val, 1);
    if (ret == 0)
        data->fifo_source = source;
    return ret;
}

int bmp390_flush_fifo(struct bmp390_data *data)
{
    u8 val = 0xB0;  /* Flush command from user-space */
    return bmp390_write_reg(data, BMP390_REG_CMD, &val, 1);
}

int bmp390_read_fifo(struct bmp390_data *data, u8 *buf, u16 *len)
{
    u8 status[2];
    int ret = bmp390_read_reg(data, BMP390_REG_FIFO_STATUS_1, status, 2);
    if (ret)
        return ret;
    *len = (status[1] << 8) | status[0];

    return bmp390_read_reg(data, BMP390_REG_FIFO_DATA, buf, *len);
}

int bmp390_set_interrupt_data_ready(struct bmp390_data *data, bool enable)
{
    u8 val;
    int ret = bmp390_read_reg(data, BMP390_REG_INT_CTRL, &val, 1);
    if (ret)
        return ret;
    val = (val & ~0x01) | (enable ? 0x01 : 0x00);
    return bmp390_write_reg(data, BMP390_REG_INT_CTRL, &val, 1);
}

int bmp390_set_interrupt_pin_type(struct bmp390_data *data, enum bmp390_interrupt_pin_type type)
{
    u8 val;
    int ret = bmp390_read_reg(data, BMP390_REG_INT_CTRL, &val, 1);
    if (ret)
        return ret;
    val = (val & ~0x04) | ((type << 2) & 0x04);
    ret = bmp390_write_reg(data, BMP390_REG_INT_CTRL, &val, 1);
    if (ret == 0)
        data->int_pin_type = type;
    return ret;
}

int bmp390_set_interrupt_active_level(struct bmp390_data *data, enum bmp390_interrupt_active_level level)
{
    u8 val;
    int ret = bmp390_read_reg(data, BMP390_REG_INT_CTRL, &val, 1);
    if (ret)
        return ret;
    val = (val & ~0x02) | ((level << 1) & 0x02);
    ret = bmp390_write_reg(data, BMP390_REG_INT_CTRL, &val, 1);
    if (ret == 0)
        data->int_level = level;
    return ret;
}

int bmp390_set_spi_wire(struct bmp390_data *data, enum bmp390_spi_wire wire)
{
    u8 val;
    int ret = bmp390_read_reg(data, BMP390_REG_CONFIG, &val, 1);
    if (ret)
        return ret;
    val = (val & ~0x08) | ((wire << 3) & 0x08);
    ret = bmp390_write_reg(data, BMP390_REG_CONFIG, &val, 1);
    if (ret == 0)
        data->spi_wire = wire;
    return ret;
}

int bmp390_set_iic_watchdog_period(struct bmp390_data *data, enum bmp390_iic_watchdog_period period)
{
    u8 val;
    int ret = bmp390_read_reg(data, BMP390_REG_CONFIG, &val, 1);
    if (ret)
        return ret;
    val = (val & ~0x10) | ((period << 4) & 0x10);
    ret = bmp390_write_reg(data, BMP390_REG_CONFIG, &val, 1);
    if (ret == 0)
        data->watchdog_period = period;
    return ret;
}