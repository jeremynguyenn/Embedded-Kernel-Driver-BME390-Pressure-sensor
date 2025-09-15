/*
 * bmp390_spi.c - SPI specific for BMP390
 */
#include "bmp390.h"
#include <linux/of.h>
#include <linux/pm_runtime.h>

/**
 * bmp390_spi_probe - Probe function for SPI
 * @spi: SPI device
 * Return: 0 on success, negative error code on failure
 */
static int bmp390_spi_probe(struct spi_device *spi)
{
    struct iio_dev *indio_dev;
    struct bmp390_data *data;
    int ret;
    const char *op_mode_str;
    u32 fifo_watermark, odr;

    indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*data));
    if (!indio_dev)
        return -ENOMEM;

    data = iio_priv(indio_dev);
    data->spi = spi;
    data->client = NULL;
    data->op_mode = BMP390_OP_MODE_DEFAULT;
    spi_set_drvdata(spi, indio_dev);
    mutex_init(&data->lock);
    init_waitqueue_head(&data->wait_queue);

    ret = of_property_read_string(spi->dev.of_node, "operation-mode", &op_mode_str);
    if (!ret) {
        if (!strcmp(op_mode_str, "periodic"))
            data->op_mode = BMP390_OP_MODE_PERIODIC;
        else if (!strcmp(op_mode_str, "async"))
            data->op_mode = BMP390_OP_MODE_ASYNC;
        else
            dev_warn(&spi->dev, "Invalid operation-mode: %s, using default\n", op_mode_str);
    }

    ret = of_property_read_u32(spi->dev.of_node, "fifo-watermark", &fifo_watermark);
    if (!ret && fifo_watermark <= 255)
        data->fifo_watermark = fifo_watermark;
    else
        data->fifo_watermark = 1;

    ret = of_property_read_u32(spi->dev.of_node, "output-data-rate", &odr);
    if (!ret && odr <= BMP390_ODR_100_HZ)
        data->odr = odr;
    else
        data->odr = BMP390_ODR_25_HZ;

    data->vdd = devm_regulator_get(&spi->dev, "vdd");
    if (IS_ERR(data->vdd))
        return PTR_ERR(data->vdd);
    ret = regulator_enable(data->vdd);
    if (ret)
        return ret;

    pm_runtime_enable(&spi->dev);
    data->pm_enabled = true;

    spi->mode = SPI_MODE_3;
    spi->max_speed_hz = 1000000;
    spi->bits_per_word = 8;
    ret = spi_setup(spi);
    if (ret)
        goto err_reg;

    ret = bmp390_setup_shared_memory(data);
    if (ret)
        goto err_reg;

    ret = bmp390_soft_reset(data);
    if (ret)
        goto err_shared_mem;

    ret = bmp390_read_reg(data, BMP390_REG_CHIP_ID, &data->chip_id, 1);
    if (ret || data->chip_id != BMP390_CHIP_ID) {
        dev_err(&spi->dev, "Invalid chip ID 0x%02x\n", data->chip_id);
        ret = -ENODEV;
        goto err_shared_mem;
    }

    ret = bmp390_load_calib(data);
    if (ret)
        goto err_shared_mem;

    ret = bmp390_set_oversampling(data, BMP390_OVERSAMPLING_x16, BMP390_OVERSAMPLING_x2);
    if (ret)
        goto err_shared_mem;

    ret = bmp390_set_odr(data, data->odr);
    if (ret)
        goto err_shared_mem;

    ret = bmp390_set_filter_coefficient(data, BMP390_FILTER_COEFFICIENT_15);
    if (ret)
        goto err_shared_mem;

    ret = bmp390_set_mode(data, BMP390_MODE_NORMAL);
    if (ret)
        goto err_shared_mem;

    ret = bmp390_set_fifo_data_source(data, BMP390_FIFO_DATA_SOURCE_FILTERED);
    if (ret)
        goto err_shared_mem;

    ret = bmp390_set_interrupt_data_ready(data, data->op_mode == BMP390_OP_MODE_ASYNC);
    if (ret)
        goto err_shared_mem;

    ret = bmp390_set_interrupt_pin_type(data, BMP390_INTERRUPT_PIN_TYPE_PUSH_PULL);
    if (ret)
        goto err_shared_mem;

    ret = bmp390_set_interrupt_active_level(data, BMP390_INTERRUPT_ACTIVE_LEVEL_HIGH);
    if (ret)
        goto err_shared_mem;

    ret = bmp390_set_spi_wire(data, BMP390_SPI_WIRE_4);
    if (ret)
        goto err_shared_mem;

    ret = bmp390_set_fifo_watermark(data, data->fifo_watermark);
    if (ret)
        goto err_shared_mem;

    indio_dev->dev.parent = &spi->dev;
    indio_dev->name = "bmp390";
    indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_TRIGGERED;
    indio_dev->info = &bmp390_info;

    ret = bmp390_iio_register(indio_dev, data);
    if (ret)
        goto err_shared_mem;

    ret = iio_device_register(indio_dev);
    if (ret)
        goto err_iio;

    if (spi->irq) {
        ret = devm_request_threaded_irq(&spi->dev, spi->irq, NULL, bmp390_irq_handler,
                                        IRQF_TRIGGER_RISING | IRQF_ONESHOT, "bmp390", indio_dev);
        if (ret)
            goto err_iio_reg;
        data->irq_enabled = true;
    }

    if (data->op_mode == BMP390_OP_MODE_PERIODIC) {
        ret = bmp390_init_kthread(data);
        if (ret)
            goto err_iio_reg;
    }

    return 0;

err_iio_reg:
    iio_device_unregister(indio_dev);
err_iio:
    bmp390_iio_unregister(indio_dev);
err_shared_mem:
    bmp390_cleanup_shared_memory(data);
err_reg:
    regulator_disable(data->vdd);
    pm_runtime_disable(&spi->dev);
    return ret;
}

/**
 * bmp390_spi_remove - Remove function for SPI
 * @spi: SPI device
 * Return: 0 on success
 */
static int bmp390_spi_remove(struct spi_device *spi)
{
    struct iio_dev *indio_dev = spi_get_drvdata(spi);
    struct bmp390_data *data = iio_priv(indio_dev);

    bmp390_stop_kthread(data);
    iio_device_unregister(indio_dev);
    bmp390_iio_unregister(indio_dev);
    bmp390_set_mode(data, BMP390_MODE_SLEEP);
    bmp390_cleanup_shared_memory(data);
    regulator_disable(data->vdd);
    pm_runtime_disable(&spi->dev);
    return 0;
}

/**
 * bmp390_spi_suspend - Suspend callback for power management
 * @dev: Device structure
 * Return: 0 on success
 */
static int __maybe_unused bmp390_spi_suspend(struct device *dev)
{
    struct iio_dev *indio_dev = dev_get_drvdata(dev);
    struct bmp390_data *data = iio_priv(indio_dev);

    bmp390_stop_kthread(data);
    bmp390_set_mode(data, BMP390_MODE_SLEEP);
    regulator_disable(data->vdd);
    return 0;
}

/**
 * bmp390_spi_resume - Resume callback for power management
 * @dev: Device structure
 * Return: 0 on success
 */
static int __maybe_unused bmp390_spi_resume(struct device *dev)
{
    struct iio_dev *indio_dev = dev_get_drvdata(dev);
    struct bmp390_data *data = iio_priv(indio_dev);
    int ret;

    ret = regulator_enable(data->vdd);
    if (ret)
        return ret;
    return bmp390_set_mode(data, BMP390_MODE_NORMAL);
}

static const struct dev_pm_ops bmp390_spi_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(bmp390_spi_suspend, bmp390_spi_resume)
    SET_RUNTIME_PM_OPS(bmp390_spi_suspend, bmp390_spi_resume, NULL)
};

static const struct spi_device_id bmp390_spi_id[] = {
    { "bmp390", 0 },
    { }
};
MODULE_DEVICE_TABLE(spi, bmp390_spi_id);

static const struct of_device_id bmp390_of_match[] = {
    { .compatible = "bosch,bmp390" },
    { }
};
MODULE_DEVICE_TABLE(of, bmp390_of_match);

static struct spi_driver bmp390_spi_driver = {
    .driver = {
        .name = "bmp390-spi",
        .of_match_table = bmp390_of_match,
        .pm = &bmp390_spi_pm_ops,
    },
    .probe = bmp390_spi_probe,
    .remove = bmp390_spi_remove,
    .id_table = bmp390_spi_id,
};

module_spi_driver(bmp390_spi_driver);

MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("BMP390 SPI driver");
MODULE_LICENSE("GPL v2");