/*
 * bmp390_iio.c - IIO subsystem integration for BMP390 pressure and temperature sensor
 * Author: Nguyen Nhan
 * Copyright (c) 2025, Nguyen Nhan. MIT License.
 */
#include "bmp390.h"
#include <linux/iio/buffer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/pm_runtime.h>

/* Channel indices */
#define BMP390_CHANNEL_TEMP  0
#define BMP390_CHANNEL_PRESS 1

/**
 * bmp390_read_raw - Read raw or processed data from the BMP390 sensor
 * @indio_dev: IIO device structure
 * @chan: Channel specification
 * @val: First value (integer part or raw value)
 * @val2: Second value (micro part for processed data)
 * @mask: Information mask specifying what to read
 * Return: IIO value type (IIO_VAL_INT, IIO_VAL_INT_PLUS_MICRO) or negative error code
 */
static int bmp390_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int *val, int *val2, long mask)
{
    struct bmp390_data *data = iio_priv(indio_dev);
    s32 temp_raw, press_raw, temp, press;
    int ret;

    ret = pm_runtime_get_sync(indio_dev->dev.parent);
    if (ret < 0) {
        dev_err(indio_dev->dev.parent, "Failed to enable runtime PM: %d\n", ret);
        return ret;
    }

    switch (mask) {
    case IIO_CHAN_INFO_RAW:
        if (data->op_mode == BMP390_OP_MODE_PERIODIC) {
            wait_event_timeout(data->wait_queue, data->shared_mem->status, msecs_to_jiffies(1000));
            mutex_lock(&data->lock);
            temp = data->shared_mem->temperature;
            press = data->shared_mem->pressure;
            mutex_unlock(&data->lock);
        } else {
            ret = bmp390_set_mode(data, BMP390_MODE_FORCED);
            if (ret) {
                dev_err(indio_dev->dev.parent, "Failed to set forced mode: %d\n", ret);
                goto out_pm;
            }
            msleep(10); /* Wait for measurement */
            ret = bmp390_read_temperature_pressure(data, &temp_raw, &press_raw, &temp, &press);
            if (ret) {
                dev_err(indio_dev->dev.parent, "Failed to read data: %d\n", ret);
                goto out_pm;
            }
        }
        switch (chan->type) {
        case IIO_TEMP:
            *val = temp;
            return IIO_VAL_INT;
        case IIO_PRESSURE:
            *val = press;
            return IIO_VAL_INT;
        default:
            ret = -EINVAL;
            goto out_pm;
        }
    case IIO_CHAN_INFO_PROCESSED:
        if (data->op_mode == BMP390_OP_MODE_PERIODIC) {
            wait_event_timeout(data->wait_queue, data->shared_mem->status, msecs_to_jiffies(1000));
            mutex_lock(&data->lock);
            temp = data->shared_mem->temperature;
            press = data->shared_mem->pressure;
            mutex_unlock(&data->lock);
        } else {
            ret = bmp390_set_mode(data, BMP390_MODE_FORCED);
            if (ret) {
                dev_err(indio_dev->dev.parent, "Failed to set forced mode: %d\n", ret);
                goto out_pm;
            }
            msleep(10);
            ret = bmp390_read_temperature_pressure(data, &temp_raw, &press_raw, &temp, &press);
            if (ret) {
                dev_err(indio_dev->dev.parent, "Failed to read data: %d\n", ret);
                goto out_pm;
            }
        }
        switch (chan->type) {
        case IIO_TEMP:
            *val = temp / 1000; /* Convert to degrees Celsius */
            *val2 = (temp % 1000) * 1000; /* Micro degrees */
            return IIO_VAL_INT_PLUS_MICRO;
        case IIO_PRESSURE:
            *val = press / 256; /* Convert to kPa */
            *val2 = (press % 256) * (1000000 / 256); /* Micro kPa */
            return IIO_VAL_INT_PLUS_MICRO;
        default:
            ret = -EINVAL;
            goto out_pm;
        }
    case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
        switch (chan->type) {
        case IIO_TEMP:
            *val = 1 << data->osr_t;
            return IIO_VAL_INT;
        case IIO_PRESSURE:
            *val = 1 << data->osr_p;
            return IIO_VAL_INT;
        default:
            ret = -EINVAL;
            goto out_pm;
        }
    case IIO_CHAN_INFO_SAMP_FREQ:
        *val = data->odr;
        return IIO_VAL_INT;
    default:
        ret = -EINVAL;
        goto out_pm;
    }

out_pm:
    pm_runtime_put(indio_dev->dev.parent);
    return ret;
}

/**
 * bmp390_write_raw - Write configuration data to the BMP390 sensor
 * @indio_dev: IIO device structure
 * @chan: Channel specification
 * @val: First value
 * @val2: Second value (unused)
 * @mask: Information mask specifying what to write
 * Return: 0 on success, negative error code on failure
 */
static int bmp390_write_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int val, int val2, long mask)
{
    struct bmp390_data *data = iio_priv(indio_dev);
    enum bmp390_oversampling osr;
    int ret;

    ret = pm_runtime_get_sync(indio_dev->dev.parent);
    if (ret < 0) {
        dev_err(indio_dev->dev.parent, "Failed to enable runtime PM: %d\n", ret);
        return ret;
    }

    switch (mask) {
    case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
        osr = ilog2(val);
        if (osr < BMP390_OVERSAMPLING_x1 || osr > BMP390_OVERSAMPLING_x32) {
            dev_err(indio_dev->dev.parent, "Invalid oversampling ratio: %d\n", val);
            ret = -EINVAL;
            goto out_pm;
        }
        switch (chan->type) {
        case IIO_TEMP:
            data->osr_t = osr;
            break;
        case IIO_PRESSURE:
            data->osr_p = osr;
            break;
        default:
            ret = -EINVAL;
            goto out_pm;
        }
        ret = bmp390_set_oversampling(data, data->osr_p, data->osr_t);
        if (ret)
            dev_err(indio_dev->dev.parent, "Failed to set oversampling: %d\n", ret);
        break;
    case IIO_CHAN_INFO_SAMP_FREQ:
        if (val < BMP390_ODR_0P78_HZ || val > BMP390_ODR_100_HZ) {
            dev_err(indio_dev->dev.parent, "Invalid sampling frequency: %d\n", val);
            ret = -EINVAL;
            goto out_pm;
        }
        ret = bmp390_set_odr(data, val);
        if (ret)
            dev_err(indio_dev->dev.parent, "Failed to set ODR: %d\n", ret);
        break;
    default:
        ret = -EINVAL;
        goto out_pm;
    }

out_pm:
    pm_runtime_put(indio_dev->dev.parent);
    return ret;
}

/**
 * bmp390_irq_handler - Interrupt handler for BMP390
 * @irq: IRQ number
 * @private: Private data (IIO device)
 * Return: IRQ_HANDLED
 */
irqreturn_t bmp390_irq_handler(int irq, void *private)
{
    struct iio_dev *indio_dev = private;
    struct bmp390_data *data = iio_priv(indio_dev);
    u8 status;
    int ret;

    ret = pm_runtime_get_sync(indio_dev->dev.parent);
    if (ret < 0)
        return IRQ_HANDLED;

    ret = bmp390_read_reg(data, BMP390_REG_INT_STATUS, &status, 1);
    if (ret)
        goto out_pm;

    if (status & BMP390_INTERRUPT_STATUS_DATA_READY) {
        schedule_work(&data->data_work);
        dev_dbg(indio_dev->dev.parent, "Data ready interrupt triggered\n");
    }

    if (status & BMP390_INTERRUPT_STATUS_FIFO_WATERMARK) {
        u8 *fifo_buf;
        u16 fifo_len;

        fifo_buf = kmalloc(data->fifo_length, GFP_KERNEL);
        if (!fifo_buf)
            goto out_pm;

        ret = bmp390_read_fifo(data, fifo_buf, &fifo_len);
        if (!ret) {
            iio_push_to_buffers(indio_dev, fifo_buf);
            dev_dbg(indio_dev->dev.parent, "Pushed %d bytes to IIO buffer\n", fifo_len);
        }
        kfree(fifo_buf);
    }

out_pm:
    pm_runtime_put(indio_dev->dev.parent);
    return IRQ_HANDLED;
}

/**
 * bmp390_data_work - Work function to process data on interrupt
 * @work: Work structure
 */
static void bmp390_data_work(struct work_struct *work)
{
    struct bmp390_data *data = container_of(work, struct bmp390_data, data_work);
    struct iio_dev *indio_dev = data->indio_dev;
    s32 temp, press;
    u64 timestamp;
    int ret;

    ret = pm_runtime_get_sync(indio_dev->dev.parent);
    if (ret < 0)
        return;

    ret = bmp390_read_temperature_pressure(data, NULL, NULL, &temp, &press);
    if (ret) {
        dev_err(indio_dev->dev.parent, "Failed to read data in work: %d\n", ret);
        goto out_pm;
    }

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

    timestamp = ktime_get_ns();
    iio_push_to_buffers_with_timestamp(indio_dev, (u8 *)&(struct { s32 temp; s32 press; }) { temp, press }, timestamp);

out_pm:
    pm_runtime_put(indio_dev->dev.parent);
}

/**
 * bmp390_iio_buffer_setup - Setup IIO triggered buffer
 * @indio_dev: IIO device structure
 * @data: Device data structure
 * Return: 0 on success, negative error code on failure
 */
static int bmp390_iio_buffer_setup(struct iio_dev *indio_dev, struct bmp390_data *data)
{
    int ret;

    ret = iio_triggered_buffer_setup(indio_dev, NULL, NULL, NULL);
    if (ret) {
        dev_err(indio_dev->dev.parent, "Failed to setup triggered buffer: %d\n", ret);
        return ret;
    }

    data->fifo_length = 256; /* Example FIFO length */
    indio_dev->channels = bmp390_channels;
    indio_dev->num_channels = ARRAY_SIZE(bmp390_channels);
    indio_dev->modes |= INDIO_BUFFER_TRIGGERED;
    INIT_WORK(&data->data_work, bmp390_data_work);

    return 0;
}

/**
 * bmp390_iio_register - Register IIO device with buffer
 * @indio_dev: IIO device structure
 * @data: Device data structure
 * Return: 0 on success, negative error code on failure
 */
int bmp390_iio_register(struct iio_dev *indio_dev, struct bmp390_data *data)
{
    int ret;

    data->indio_dev = indio_dev;
    ret = bmp390_iio_buffer_setup(indio_dev, data);
    if (ret) {
        dev_err(indio_dev->dev.parent, "Failed to setup IIO buffer: %d\n", ret);
        return ret;
    }

    dev_info(indio_dev->dev.parent, "IIO device registered\n");
    return 0;
}

/**
 * bmp390_iio_unregister - Unregister IIO device and cleanup buffer
 * @indio_dev: IIO device structure
 */
void bmp390_iio_unregister(struct iio_dev *indio_dev)
{
    cancel_work_sync(&iio_priv(indio_dev)->data_work);
    iio_triggered_buffer_cleanup(indio_dev);
    dev_info(indio_dev->dev.parent, "IIO device unregistered\n");
}

/* Sysfs attributes for oversampling and sampling frequency */
static IIO_DEVICE_ATTR(in_temp_oversampling_ratio, 0644,
                       bmp390_read_raw, bmp390_write_raw, IIO_CHAN_INFO_OVERSAMPLING_RATIO);
static IIO_DEVICE_ATTR(in_pressure_oversampling_ratio, 0644,
                       bmp390_read_raw, bmp390_write_raw, IIO_CHAN_INFO_OVERSAMPLING_RATIO);
static IIO_DEVICE_ATTR(in_sampling_frequency, 0644,
                       bmp390_read_raw, bmp390_write_raw, IIO_CHAN_INFO_SAMP_FREQ);
static IIO_DEVICE_ATTR(in_fifo_watermark, 0644,
                       NULL,
                       NULL,
                       0); /* Placeholder for watermark attribute */

static struct attribute *bmp390_attributes[] = {
    &iio_dev_attr_in_temp_oversampling_ratio.dev_attr.attr,
    &iio_dev_attr_in_pressure_oversampling_ratio.dev_attr.attr,
    &iio_dev_attr_in_sampling_frequency.dev_attr.attr,
    &iio_dev_attr_in_fifo_watermark.dev_attr.attr,
    NULL
};

static const struct attribute_group bmp390_attribute_group = {
    .attrs = bmp390_attributes,
};

MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("BMP390 IIO subsystem driver");
MODULE_LICENSE("GPL v2");