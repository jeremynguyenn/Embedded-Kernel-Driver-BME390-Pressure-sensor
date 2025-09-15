/*
 * bmp390_iio.c - IIO subsystem integration
 * Based on read/interrupt/fifo tests.
 */

#include "bmp390.h"

static int bmp390_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int *val, int *val2, long mask)
{
    struct bmp390_data *data = iio_priv(indio_dev);
    s32 temp_raw, press_raw, temp, press;
    int ret;

    switch (mask) {
    case IIO_CHAN_INFO_RAW:
        ret = bmp390_set_mode(data, BMP390_MODE_FORCED);
        if (ret)
            return ret;
        msleep(10);
        ret = bmp390_read_temperature_pressure(data, &temp_raw, &press_raw, &temp, &press);
        if (ret)
            return ret;
        switch (chan->type) {
        case IIO_TEMP:
            *val = temp_raw;
            return IIO_VAL_INT;
        case IIO_PRESSURE:
            *val = press_raw;
            return IIO_VAL_INT;
        default:
            return -EINVAL;
        }
    case IIO_CHAN_INFO_PROCESSED:
        ret = bmp390_set_mode(data, BMP390_MODE_FORCED);
        if (ret)
            return ret;
        msleep(10);
        ret = bmp390_read_temperature_pressure(data, &temp_raw, &press_raw, &temp, &press);
        if (ret)
            return ret;
        switch (chan->type) {
        case IIO_TEMP:
            *val = temp / 1000;  /* mdeg C */
            *val2 = (temp % 1000) * 1000;
            return IIO_VAL_INT_PLUS_MICRO;
        case IIO_PRESSURE:
            *val = press / 256;
            *val2 = (press % 256) * (1000000 / 256);
            return IIO_VAL_INT_PLUS_MICRO;
        default:
            return -EINVAL;
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
            return -EINVAL;
        }
    default:
        return -EINVAL;
    }
}

static int bmp390_write_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int val, int val2, long mask)
{
    struct bmp390_data *data = iio_priv(indio_dev);
    enum bmp390_oversampling osr;

    switch (mask) {
    case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
        osr = ilog2(val);
        if (osr < BMP390_OVERSAMPLING_x1 || osr > BMP390_OVERSAMPLING_x32)
            return -EINVAL;
        switch (chan->type) {
        case IIO_TEMP:
            data->osr_t = osr;
            break;
        case IIO_PRESSURE:
            data->osr_p = osr;
            break;
        default:
            return -EINVAL;
        }
        return bmp390_set_oversampling(data, data->osr_p, data->osr_t);
    default:
        return -EINVAL;
    }
}

static int bmp390_show_osr_avail(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "1 2 4 8 16 32\n");
}

static int bmp390_show_odr_avail(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "0.78 1.5 3 6 12.5 25 50 100\n");
}

static int bmp390_show_filter_avail(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "0 1 3 7 15 31 63 127\n");
}

static ssize_t bmp390_set_odr_attr(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    struct iio_dev *indio_dev = dev_to_iio_dev(dev);
    struct bmp390_data *data = iio_priv(indio_dev);
    enum bmp390_odr odr;
    double val;

    if (kstrtod(buf, 10, &val) < 0)
        return -EINVAL;

    if (val == 0.78) odr = BMP390_ODR_0P78_HZ;
    else if (val == 1.5) odr = BMP390_ODR_1P5_HZ;
    else if (val == 3) odr = BMP390_ODR_3_HZ;
    else if (val == 6) odr = BMP390_ODR_6_HZ;
    else if (val == 12.5) odr = BMP390_ODR_12P5_HZ;
    else if (val == 25) odr = BMP390_ODR_25_HZ;
    else if (val == 50) odr = BMP390_ODR_50_HZ;
    else if (val == 100) odr = BMP390_ODR_100_HZ;
    else return -EINVAL;

    return bmp390_set_odr(data, odr) ? -EINVAL : len;
}

static ssize_t bmp390_get_odr_attr(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct iio_dev *indio_dev = dev_to_iio_dev(dev);
    struct bmp390_data *data = iio_priv(indio_dev);
    double val;

    switch (data->odr) {
    case BMP390_ODR_0P78_HZ: val = 0.78; break;
    case BMP390_ODR_1P5_HZ: val = 1.5; break;
    case BMP390_ODR_3_HZ: val = 3; break;
    case BMP390_ODR_6_HZ: val = 6; break;
    case BMP390_ODR_12P5_HZ: val = 12.5; break;
    case BMP390_ODR_25_HZ: val = 25; break;
    case BMP390_ODR_50_HZ: val = 50; break;
    case BMP390_ODR_100_HZ: val = 100; break;
    }

    return sprintf(buf, "%.2f\n", val);
}

static IIO_DEVICE_ATTR(sampling_frequency_available, 0444, bmp390_show_odr_avail, NULL, 0);
static IIO_DEVICE_ATTR(sampling_frequency, 0644, bmp390_get_odr_attr, bmp390_set_odr_attr, 0);
static IIO_DEVICE_ATTR(in_pressure_oversampling_ratio_available, 0444, bmp390_show_osr_avail, NULL, 0);
static IIO_DEVICE_ATTR(in_temp_oversampling_ratio_available, 0444, bmp390_show_osr_avail, NULL, 0);
static IIO_DEVICE_ATTR(filter_coeff_available, 0444, bmp390_show_filter_avail, NULL, 0);

static struct attribute *bmp390_attrs[] = {
    &iio_dev_attr_sampling_frequency_available.dev_attr.attr,
    &iio_dev_attr_sampling_frequency.dev_attr.attr,
    &iio_dev_attr_in_pressure_oversampling_ratio_available.dev_attr.attr,
    &iio_dev_attr_in_temp_oversampling_ratio_available.dev_attr.attr,
    &iio_dev_attr_filter_coeff_available.dev_attr.attr,
    NULL,
};

static const struct attribute_group bmp390_attr_group = {
    .attrs = bmp390_attrs,
};

static const struct iio_chan_spec bmp390_channels[] = {
    {
        .type = IIO_TEMP,
        .address = BMP390_CHANNEL_TEMP,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_PROCESSED) | BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),
        .info_mask_separate_available = BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),
        .scan_index = 0,
        .scan_type = {
            .sign = 's',
            .realbits = 24,
            .storagebits = 32,
            .shift = 0,
        },
    },
    {
        .type = IIO_PRESSURE,
        .address = BMP390_CHANNEL_PRESS,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_PROCESSED) | BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),
        .info_mask_separate_available = BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),
        .scan_index = 1,
        .scan_type = {
            .sign = 'u',
            .realbits = 24,
            .storagebits = 32,
            .shift = 0,
        },
    },
    IIO_CHAN_SOFT_TIMESTAMP(2),
};

static const struct iio_info bmp390_info = {
    .read_raw = bmp390_read_raw,
    .write_raw = bmp390_write_raw,
    .attrs = &bmp390_attr_group,
    .driver_module = THIS_MODULE,
};

static int bmp390_buffer_postenable(struct iio_dev *indio_dev)
{
    struct bmp390_data *data = iio_priv(indio_dev);
    bmp390_set_fifo_watermark(data, 1);  /* Minimal */
    return bmp390_set_mode(data, BMP390_MODE_NORMAL);
}

static int bmp390_buffer_predisable(struct iio_dev *indio_dev)
{
    struct bmp390_data *data = iio_priv(indio_dev);
    bmp390_flush_fifo(data);
    return bmp390_set_mode(data, BMP390_MODE_SLEEP);
}

static const struct iio_buffer_setup_ops bmp390_buffer_ops = {
    .postenable = bmp390_buffer_postenable,
    .predisable = bmp390_buffer_predisable,
};

int bmp390_iio_register(struct iio_dev *indio_dev, struct bmp390_data *data)
{
    indio_dev->channels = bmp390_channels;
    indio_dev->num_channels = ARRAY_SIZE(bmp390_channels);
    indio_dev->info = &bmp390_info;
    indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_TRIGGERED;
    indio_dev->setup_ops = &bmp390_buffer_ops;

    INIT_WORK(&data->data_work, bmp390_data_work_fn);

    return iio_triggered_buffer_setup(indio_dev, NULL, bmp390_trigger_handler, NULL);
}

void bmp390_iio_unregister(struct iio_dev *indio_dev)
{
    iio_triggered_buffer_cleanup(indio_dev);
}

static irqreturn_t bmp390_irq_handler(int irq, void *private)
{
    struct iio_dev *indio_dev = private;
    struct bmp390_data *data = iio_priv(indio_dev);
    u8 status;

    bmp390_read_reg(data, BMP390_REG_INT_STATUS, &status, 1);
    if (status & BMP390_INTERRUPT_STATUS_DATA_READY) {
        schedule_work(&data->data_work);
    } else if (status & BMP390_INTERRUPT_STATUS_FIFO_WATERMARK) {
        /* Handle FIFO watermark */
        iio_trigger_poll(indio_dev->trig);
    } else if (status & BMP390_INTERRUPT_STATUS_FIFO_FULL) {
        /* Handle FIFO full */
        iio_trigger_poll(indio_dev->trig);
    }

    return IRQ_HANDLED;
}

static void bmp390_data_work_fn(struct work_struct *work)
{
    struct bmp390_data *data = container_of(work, struct bmp390_data, data_work);
    s32 temp, press;
    bmp390_read_temperature_pressure(data, NULL, NULL, &temp, &press);
    /* Push event or data */
    iio_push_event(indio_dev, IIO_EVENT_CODE(IIO_PRESSURE, 0, IIO_NO_MOD, IIO_EV_DIR_NONE, IIO_EV_TYPE_THRESH, 0, 0, 0), 0);
}

static irqreturn_t bmp390_trigger_handler(int irq, void *p)
{
    struct iio_poll_func *pf = p;
    struct iio_dev *indio_dev = pf->indio_dev;
    struct bmp390_data *data = iio_priv(indio_dev);
    s64 buffer[4];  /* Temp, press, timestamp */
    s32 temp_raw, press_raw;

    bmp390_read_temperature_pressure(data, &temp_raw, &press_raw, NULL, NULL);
    buffer[0] = temp_raw;
    buffer[1] = press_raw;
    buffer[2] = 0;  /* Padding */
    buffer[3] = iio_get_time_ns(indio_dev);

    iio_push_to_buffers(indio_dev, buffer);
    iio_trigger_notify_done(indio_dev->trig);

    return IRQ_HANDLED;
}