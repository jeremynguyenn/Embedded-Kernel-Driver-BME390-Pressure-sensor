/*
 * bmp390_i2c.c - I2C specific for BMP390
 */

#include "bmp390.h"

static int bmp390_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct iio_dev *indio_dev;
    struct bmp390_data *data;
    int ret;

    indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
    if (!indio_dev)
        return -ENOMEM;

    data = iio_priv(indio_dev);
    data->client = client;
    data->spi = NULL;
    i2c_set_clientdata(client, indio_dev);
    mutex_init(&data->lock);

    data->vdd = devm_regulator_get(&client->dev, "vdd");
    if (IS_ERR(data->vdd))
        return PTR_ERR(data->vdd);
    ret = regulator_enable(data->vdd);
    if (ret)
        return ret;

    ret = bmp390_soft_reset(data);
    if (ret)
        goto err_reg;

    ret = bmp390_read_reg(data, BMP390_REG_CHIP_ID, &data->chip_id, 1);
    if (ret || data->chip_id != BMP390_CHIP_ID) {
        dev_err(&client->dev, "Invalid chip ID 0x%02x\n", data->chip_id);
        ret = -ENODEV;
        goto err_reg;
    }

    ret = bmp390_load_calib(data);
    if (ret)
        goto err_reg;

    bmp390_set_oversampling(data, BMP390_OVERSAMPLING_x16, BMP390_OVERSAMPLING_x2);
    bmp390_set_odr(data, BMP390_ODR_25_HZ);
    bmp390_set_filter_coefficient(data, BMP390_FILTER_COEFFICIENT_15);
    bmp390_set_mode(data, BMP390_MODE_NORMAL);
    bmp390_set_fifo_data_source(data, BMP390_FIFO_DATA_SOURCE_FILTERED);
    bmp390_set_interrupt_data_ready(data, true);
    bmp390_set_interrupt_pin_type(data, BMP390_INTERRUPT_PIN_TYPE_PUSH_PULL);
    bmp390_set_interrupt_active_level(data, BMP390_INTERRUPT_ACTIVE_LEVEL_HIGH);
    bmp390_set_spi_wire(data, BMP390_SPI_WIRE_4);
    bmp390_set_iic_watchdog_period(data, BMP390_IIC_WATCHDOG_PERIOD_40_MS);

    indio_dev->dev.parent = &client->dev;
    indio_dev->name = "bmp390";
    indio_dev->modes = INDIO_DIRECT_MODE;
    indio_dev->info = &bmp390_info;

    ret = bmp390_iio_register(indio_dev, data);
    if (ret)
        goto err_reg;

    ret = iio_device_register(indio_dev);
    if (ret)
        goto err_iio;

    if (client->irq) {
        ret = devm_request_threaded_irq(&client->dev, client->irq, NULL, bmp390_irq_handler,
                                        IRQF_TRIGGER_RISING | IRQF_ONESHOT, "bmp390", indio_dev);
        if (ret)
            goto err_iio_reg;
        data->irq_enabled = 1;
    }

    return 0;

err_iio_reg:
    iio_device_unregister(indio_dev);
err_iio:
    bmp390_iio_unregister(indio_dev);
err_reg:
    regulator_disable(data->vdd);
    return ret;
}

static int bmp390_i2c_remove(struct i2c_client *client)
{
    struct iio_dev *indio_dev = i2c_get_clientdata(client);
    struct bmp390_data *data = iio_priv(indio_dev);

    iio_device_unregister(indio_dev);
    bmp390_iio_unregister(indio_dev);
    bmp390_set_mode(data, BMP390_MODE_SLEEP);
    regulator_disable(data->vdd);
    return 0;
}

static const struct i2c_device_id bmp390_id[] = {
    { "bmp390", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, bmp390_id);

static const struct of_device_id bmp390_of_match[] = {
    { .compatible = "bosch,bmp390" },
    { }
};
MODULE_DEVICE_TABLE(of, bmp390_of_match);

static struct i2c_driver bmp390_i2c_driver = {
    .driver = {
        .name = "bmp390-i2c",
        .of_match_table = bmp390_of_match,
    },
    .probe = bmp390_i2c_probe,
    .remove = bmp390_i2c_remove,
    .id_table = bmp390_id,
};

module_i2c_driver(bmp390_i2c_driver);

MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("BMP390 I2C driver");
MODULE_LICENSE("GPL v2");