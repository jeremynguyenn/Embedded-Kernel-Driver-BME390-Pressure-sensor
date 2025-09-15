/*
 * bmp390_core.c - Core functions for BMP390 driver
 * Adapted from LibDriver user-space code.
 */
#include "bmp390.h"
#include <linux/math64.h>
#include <linux/slab.h>

#define MAX_RETRIES 3
#define RETRY_DELAY_MS 10

/**
 * bmp390_read_reg - Read from BMP390 register with retries
 * @data: Device data structure
 * @reg: Register address
 * @val: Buffer to store read data
 * @len: Number of bytes to read
 * Return: 0 on success, negative error code on failure
 */
int bmp390_read_reg(struct bmp390_data *data, u8 reg, u8 *val, u32 len)
{
    int ret, retries = 0;
    struct device *dev = data->indio_dev->dev.parent;

    mutex_lock(&data->lock);
    while (retries < MAX_RETRIES) {
        if (data->client) {
            ret = i2c_smbus_read_i2c_block_data(data->client, reg, len, val);
        } else {
            u8 txbuf[1] = { reg | 0x80 };
            ret = spi_write_then_read(data->spi, txbuf, 1, val, len);
        }
        if (ret >= 0)
            break;
        retries++;
        msleep(RETRY_DELAY_MS);
        dev_dbg(dev, "Retry %d reading reg 0x%02x\n", retries, reg);
    }
    mutex_unlock(&data->lock);

    if (ret < 0) {
        dev_err(dev, "Failed to read reg 0x%02x after %d retries\n", reg, MAX_RETRIES);
        return ret;
    }
    dev_dbg(dev, "Read reg 0x%02x, len=%d\n", reg, len);
    return 0;
}

/**
 * bmp390_write_reg - Write to BMP390 register with retries
 * @data: Device data structure
 * @reg: Register address
 * @val: Data to write
 * @len: Number of bytes to write
 * Return: 0 on success, negative error code on failure
 */
int bmp390_write_reg(struct bmp390_data *data, u8 reg, const u8 *val, u32 len)
{
    int ret, retries = 0;
    struct device *dev = data->indio_dev->dev.parent;
    u8 *buf;

    buf = kzalloc(len + 1, GFP_KERNEL);
    if (!buf)
        return -ENOMEM;

    mutex_lock(&data->lock);
    buf[0] = reg & (data->client ? 0xFF : 0x7F);
    memcpy(&buf[1], val, len);
    while (retries < MAX_RETRIES) {
        if (data->client) {
            ret = i2c_master_send(data->client, buf, len + 1);
        } else {
            ret = spi_write(data->spi, buf, len + 1);
        }
        if (ret >= 0)
            break;
        retries++;
        msleep(RETRY_DELAY_MS);
        dev_dbg(dev, "Retry %d writing reg 0x%02x\n", retries, reg);
    }
    mutex_unlock(&data->lock);

    kfree(buf);
    if (ret < 0) {
        dev_err(dev, "Failed to write reg 0x%02x after %d retries\n", reg, MAX_RETRIES);
        return ret;
    }
    dev_dbg(dev, "Wrote reg 0x%02x, len=%d\n", reg, len);
    return 0;
}

/**
 * bmp390_soft_reset - Perform soft reset
 * @data: Device data structure
 * Return: 0 on success, negative error code on failure
 */
int bmp390_soft_reset(struct bmp390_data *data)
{
    u8 cmd = BMP390_CMD_SOFT_RESET;
    int ret = bmp390_write_reg(data, BMP390_REG_CMD, &cmd, 1);
    if (ret)
        return ret;
    msleep(2);
    dev_info(data->indio_dev->dev.parent, "Soft reset completed\n");
    return 0;
}

/**
 * bmp390_set_mode - Set operating mode with power management
 * @data: Device data structure
 * @mode: Mode (BMP390_MODE_SLEEP, BMP390_MODE_FORCED, BMP390_MODE_NORMAL)
 * Return: 0 on success, negative error code on failure
 */
int bmp390_set_mode(struct bmp390_data *data, u8 mode)
{
    u8 val;
    int ret;

    ret = pm_runtime_get_sync(data->indio_dev->dev.parent);
    if (ret < 0)
        return ret;

    ret = bmp390_read_reg(data, BMP390_REG_PWR_CTRL, &val, 1);
    if (ret)
        goto out_pm;

    val = (val & ~0x30) | (mode << 4);
    ret = bmp390_write_reg(data, BMP390_REG_PWR_CTRL, &val, 1);
    if (ret == 0) {
        data->mode = mode;
        dev_info(data->indio_dev->dev.parent, "Set mode to %d\n", mode);
    }

out_pm:
    pm_runtime_put(data->indio_dev->dev.parent);
    return ret;
}

/**
 * bmp390_load_calib - Load calibration parameters
 * @data: Device data structure
 * Return: 0 on success, negative error code on failure
 */
int bmp390_load_calib(struct bmp390_data *data)
{
    u8 *buf;
    int ret;

    buf = kzalloc(21, GFP_KERNEL);
    if (!buf)
        return -ENOMEM;

    ret = pm_runtime_get_sync(data->indio_dev->dev.parent);
    if (ret < 0) {
        kfree(buf);
        return ret;
    }

    ret = bmp390_read_reg(data, BMP390_REG_NVM_PAR_T1_L, buf, 21);
    if (ret) {
        kfree(buf);
        pm_runtime_put(data->indio_dev->dev.parent);
        return ret;
    }

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

    kfree(buf);
    pm_runtime_put(data->indio_dev->dev.parent);
    dev_info(data->indio_dev->dev.parent, "Calibration data loaded\n");
    return 0;
}

/**
 * bmp390_compensate_temp_press - Compensate raw temperature and pressure
 * @data: Device data structure
 * @adc_temp: Raw temperature
 * @adc_press: Raw pressure
 * @temp: Compensated temperature output
 * @press: Compensated pressure output
 */
void bmp390_compensate_temp_press(struct bmp390_data *data, s32 adc_temp, s32 adc_press, s32 *temp, s32 *press)
{
    s64 temp_var1, temp_var2, temp_var3;
    s64 press_var1, press_var2, press_var3, press_var4, press_var5, press_var6;
    s64 t_lin, p_lin;

    /* Temperature compensation */
    temp_var1 = ((s64)adc_temp) - ((s64)data->calib.t1 << 8);
    temp_var2 = temp_var1 * ((s64)data->calib.t2);
    temp_var3 = (temp_var1 * temp_var1) >> 1;
    temp_var3 = (temp_var3 * ((s64)data->calib.t3)) >> 12;
    t_lin = (temp_var2 + temp_var3) >> 14;
    *temp = (s32)t_lin;

    /* Pressure compensation */
    press_var1 = (((s64)t_lin) * ((s64)t_lin)) >> 1;
    press_var1 = (press_var1 * ((s64)data->calib.p3)) >> 13;
    press_var2 = ((s64)data->calib.p2 * (s64)t_lin) >> 1;
    press_var3 = ((s64)data->calib.p1 << 17);
    press_var4 = press_var1 + press_var2 + press_var3;
    press_var5 = ((s64)adc_press * press_var4) >> 6;
    press_var6 = (s64)data->calib.p5 << 17;
    press_var1 = ((s64)adc_press * (s64)adc_press) >> 1;
    press_var1 = (press_var1 * ((s64)data->calib.p6)) >> 10;
    press_var2 = ((s64)data->calib.p4 * (s64)adc_press) >> 1;
    press_var3 = ((s64)data->calib.p7 << 13);
    press_var4 = (press_var1 + press_var2 + press_var3) >> 5;
    press_var1 = ((s64)adc_press * (s64)adc_press * (s64)adc_press) >> 3;
    press_var1 = (press_var1 * ((s64)data->calib.p9)) >> 15;
    press_var2 = ((s64)data->calib.p8 * (s64)adc_press * (s64)adc_press) >> 13;
    press_var3 = ((s64)data->calib.p10 * (s64)adc_press) >> 3;
    p_lin = press_var5 + press_var6 + press_var4 + press_var1 + press_var2 + press_var3 + ((s64)data->calib.p11 << 13);
    *press = (s32)(p_lin >> 8);
}

/**
 * bmp390_read_temperature_pressure - Read temperature and pressure
 * @data: Device data structure
 * @temp_raw: Raw temperature output
 * @press_raw: Raw pressure output
 * @temp: Compensated temperature output
 * @press: Compensated pressure output
 * Return: 0 on success, negative error code on failure
 */
int bmp390_read_temperature_pressure(struct bmp390_data *data, s32 *temp_raw, s32 *press_raw, s32 *temp, s32 *press)
{
    u8 buf[6];
    int ret;

    ret = pm_runtime_get_sync(data->indio_dev->dev.parent);
    if (ret < 0)
        return ret;

    ret = bmp390_read_reg(data, BMP390_REG_PRESSURE_DATA_0, buf, 6);
    if (ret)
        goto out_pm;

    if (press_raw)
        *press_raw = (buf[2] << 16) | (buf[1] << 8) | buf[0];
    if (temp_raw)
        *temp_raw = (buf[5] << 16) | (buf[4] << 8) | buf[3];
    if (temp || press) {
        s32 adc_temp = (buf[5] << 16) | (buf[4] << 8) | buf[3];
        s32 adc_press = (buf[2] << 16) | (buf[1] << 8) | buf[0];
        bmp390_compensate_temp_press(data, adc_temp, adc_press, temp, press);
    }

out_pm:
    pm_runtime_put(data->indio_dev->dev.parent);
    return ret;
}

/**
 * bmp390_set_oversampling - Set oversampling ratios
 * @data: Device data structure
 * @osr_p: Pressure oversampling
 * @osr_t: Temperature oversampling
 * Return: 0 on success, negative error code on failure
 */
int bmp390_set_oversampling(struct bmp390_data *data, enum bmp390_oversampling osr_p, enum bmp390_oversampling osr_t)
{
    u8 val = (osr_p & 0x07) | ((osr_t & 0x07) << 3);
    int ret = bmp390_write_reg(data, BMP390_REG_OSR, &val, 1);
    if (ret == 0) {
        data->osr_p = osr_p;
        data->osr_t = osr_t;
        dev_info(data->indio_dev->dev.parent, "Set oversampling: pressure=%d, temp=%d\n", 1 << osr_p, 1 << osr_t);
    }
    return ret;
}

/**
 * bmp390_set_odr - Set output data rate
 * @data: Device data structure
 * @odr: Output data rate
 * Return: 0 on success, negative error code on failure
 */
int bmp390_set_odr(struct bmp390_data *data, enum bmp390_odr odr)
{
    u8 val = odr & 0x0F;
    int ret = bmp390_write_reg(data, BMP390_REG_ODR, &val, 1);
    if (ret == 0) {
        data->odr = odr;
        dev_info(data->indio_dev->dev.parent, "Set ODR: %d\n", odr);
    }
    return ret;
}

/**
 * bmp390_set_filter_coefficient - Set IIR filter coefficient
 * @data: Device data structure
 * @coeff: Filter coefficient
 * Return: 0 on success, negative error code on failure
 */
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

/**
 * bmp390_set_fifo_watermark - Set FIFO watermark
 * @data: Device data structure
 * @watermark: FIFO watermark value
 * Return: 0 on success, negative error code on failure
 */
int bmp390_set_fifo_watermark(struct bmp390_data *data, u8 watermark)
{
    u8 val = watermark;
    int ret = bmp390_write_reg(data, BMP390_REG_FIFO_CONFIG_0, &val, 1);
    if (ret == 0) {
        data->fifo_watermark = watermark;
        dev_info(data->indio_dev->dev.parent, "Set FIFO watermark: %d\n", watermark);
    }
    return ret;
}

/**
 * bmp390_set_fifo_data_source - Set FIFO data source
 * @data: Device data structure
 * @source: FIFO data source
 * Return: 0 on success, negative error code on failure
 */
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

/**
 * bmp390_flush_fifo - Flush FIFO
 * @data: Device data structure
 * Return: 0 on success, negative error code on failure
 */
int bmp390_flush_fifo(struct bmp390_data *data)
{
    u8 val = 0xB0;
    int ret = bmp390_write_reg(data, BMP390_REG_CMD, &val, 1);
    if (ret == 0)
        dev_info(data->indio_dev->dev.parent, "FIFO flushed\n");
    return ret;
}

/**
 * bmp390_read_fifo - Read FIFO data
 * @data: Device data structure
 * @buf: Buffer for FIFO data
 * @len: Length of data read
 * Return: 0 on success, negative error code on failure
 */
int bmp390_read_fifo(struct bmp390_data *data, u8 *buf, u16 *len)
{
    u8 status[2];
    int ret = bmp390_read_reg(data, BMP390_REG_FIFO_STATUS_1, status, 2);
    if (ret)
        return ret;
    *len = (status[1] << 8) | status[0];
    ret = bmp390_read_reg(data, BMP390_REG_FIFO_DATA, buf, *len);
    if (ret == 0)
        dev_dbg(data->indio_dev->dev.parent, "Read FIFO: %d bytes\n", *len);
    return ret;
}

/**
 * bmp390_set_interrupt_data_ready - Enable/disable data ready interrupt
 * @data: Device data structure
 * @enable: Enable flag
 * Return: 0 on success, negative error code on failure
 */
int bmp390_set_interrupt_data_ready(struct bmp390_data *data, bool enable)
{
    u8 val;
    int ret = bmp390_read_reg(data, BMP390_REG_INT_CTRL, &val, 1);
    if (ret)
        return ret;
    val = (val & ~0x01) | (enable ? 0x01 : 0x00);
    ret = bmp390_write_reg(data, BMP390_REG_INT_CTRL, &val, 1);
    if (ret == 0) {
        data->irq_enabled = enable;
        dev_info(data->indio_dev->dev.parent, "Data ready interrupt %s\n", enable ? "enabled" : "disabled");
    }
    return ret;
}

/**
 * bmp390_set_interrupt_pin_type - Set interrupt pin type
 * @data: Device data structure
 * @type: Pin type
 * Return: 0 on success, negative error code on failure
 */
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

/**
 * bmp390_set_interrupt_active_level - Set interrupt active level
 * @data: Device data structure
 * @level: Active level
 * Return: 0 on success, negative error code on failure
 */
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

/**
 * bmp390_set_spi_wire - Set SPI wire mode
 * @data: Device data structure
 * @wire: SPI wire mode
 * Return: 0 on success, negative error code on failure
 */
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

/**
 * bmp390_set_iic_watchdog_period - Set I2C watchdog period
 * @data: Device data structure
 * @period: Watchdog period
 * Return: 0 on success, negative error code on failure
 */
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

/**
 * bmp390_kthread_fn - Kernel thread function for periodic polling
 * @arg: Device data structure
 * Return: 0 on success
 */
static int bmp390_kthread_fn(void *arg)
{
    struct bmp390_data *data = arg;
    s32 temp, press;

    while (!kthread_should_stop()) {
        if (data->op_mode == BMP390_OP_MODE_PERIODIC) {
            if (bmp390_read_temperature_pressure(data, NULL, NULL, &temp, &press) == 0) {
                mutex_lock(&data->lock);
                data->shared_mem->temperature = temp;
                data->shared_mem->pressure = press;
                data->shared_mem->timestamp = ktime_get_ns();
                data->shared_mem->status = 1;
                mutex_unlock(&data->lock);
                complete(&data->data_ready);
                wake_up(&data->wait_queue);

                if (data->sigio_file && data->sigio_file->f_owner.pid)
                    send_sig(SIGIO, data->sigio_file->f_owner.pid, 0);
            }
        }
        msleep_interruptible(1000 / (data->odr + 1));
    }
    return 0;
}

/**
 * bmp390_init_kthread - Initialize kernel thread
 * @data: Device data structure
 * Return: 0 on success, negative error code on failure
 */
int bmp390_init_kthread(struct bmp390_data *data)
{
    init_completion(&data->data_ready);
    init_waitqueue_head(&data->wait_queue);
    data->kthread = kthread_run(bmp390_kthread_fn, data, "bmp390-kthread");
    if (IS_ERR(data->kthread)) {
        dev_err(data->indio_dev->dev.parent, "Failed to start kthread\n");
        return PTR_ERR(data->kthread);
    }
    dev_info(data->indio_dev->dev.parent, "Kernel thread started\n");
    return 0;
}

/**
 * bmp390_stop_kthread - Stop kernel thread
 * @data: Device data structure
 */
void bmp390_stop_kthread(struct bmp390_data *data)
{
    if (data->kthread) {
        kthread_stop(data->kthread);
        data->kthread = NULL;
        dev_info(data->indio_dev->dev.parent, "Kernel thread stopped\n");
    }
}

/**
 * bmp390_setup_shared_memory - Setup shared memory for IPC
 * @data: Device data structure
 * Return: 0 on success, negative error code on failure
 */
int bmp390_setup_shared_memory(struct bmp390_data *data)
{
    data->shared_mem = kzalloc(sizeof(struct bmp390_shared_data), GFP_KERNEL);
    if (!data->shared_mem)
        return -ENOMEM;
    dev_info(data->indio_dev->dev.parent, "Shared memory allocated\n");
    return 0;
}

/**
 * bmp390_cleanup_shared_memory - Cleanup shared memory
 * @data: Device data structure
 */
void bmp390_cleanup_shared_memory(struct bmp390_data *data)
{
    if (data->shared_mem) {
        kfree(data->shared_mem);
        data->shared_mem = NULL;
        dev_info(data->indio_dev->dev.parent, "Shared memory freed\n");
    }
}