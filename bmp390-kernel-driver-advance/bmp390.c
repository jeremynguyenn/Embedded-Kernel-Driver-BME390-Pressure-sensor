/*
 * bmp390.c - Main BMP390 kernel driver
 * Integrates core, IIO, I2C/SPI.
 */
#include "bmp390.h"
#include <linux/miscdevice.h>

/**
 * bmp390_open - File open handler
 * @filp: File pointer
 * Return: 0 on success, negative error code on failure
 */
static int bmp390_open(struct file *filp)
{
    struct iio_dev *indio_dev = filp->private_data;
    struct bmp390_data *data = iio_priv(indio_dev);
    int ret;

    ret = pm_runtime_get_sync(indio_dev->dev.parent);
    if (ret < 0)
        return ret;

    data->sigio_file = filp;
    dev_dbg(indio_dev->dev.parent, "Device file opened\n");
    return 0;
}

/**
 * bmp390_release - File release handler
 * @filp: File pointer
 * Return: 0 on success, negative error code on failure
 */
static int bmp390_release(struct file *filp)
{
    struct iio_dev *indio_dev = filp->private_data;
    struct bmp390_data *data = iio_priv(indio_dev);

    data->sigio_file = NULL;
    pm_runtime_put(indio_dev->dev.parent);
    dev_dbg(indio_dev->dev.parent, "Device file released\n");
    return 0;
}

/**
 * bmp390_ioctl - IOCTL handler
 * @filp: File pointer
 * @cmd: IOCTL command
 * @arg: IOCTL argument
 * Return: 0 on success, negative error code on failure
 */
static long bmp390_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct iio_dev *indio_dev = filp->private_data;
    struct bmp390_data *data = iio_priv(indio_dev);
    int ret = 0;

    if (_IOC_TYPE(cmd) != BMP390_IOC_MAGIC)
        return -ENOTTY;

    ret = pm_runtime_get_sync(indio_dev->dev.parent);
    if (ret < 0)
        return ret;

    switch (cmd) {
    case BMP390_IOC_SET_MODE:
        {
            int mode;
            if (copy_from_user(&mode, (void __user *)arg, sizeof(mode))) {
                ret = -EFAULT;
                goto out_pm;
            }
            if (mode < BMP390_OP_MODE_DEFAULT || mode > BMP390_OP_MODE_ASYNC) {
                ret = -EINVAL;
                goto out_pm;
            }
            mutex_lock(&data->lock);
            data->op_mode = mode;
            if (mode == BMP390_OP_MODE_PERIODIC) {
                ret = bmp390_init_kthread(data);
            } else {
                bmp390_stop_kthread(data);
                if (mode == BMP390_OP_MODE_ASYNC)
                    ret = bmp390_set_interrupt_data_ready(data, true);
            }
            mutex_unlock(&data->lock);
            dev_info(indio_dev->dev.parent, "Set operation mode to %d\n", mode);
            break;
        }
    case BMP390_IOC_GET_SHARED_DATA:
        {
            mutex_lock(&data->lock);
            if (!data->shared_mem) {
                mutex_unlock(&data->lock);
                ret = -EINVAL;
                goto out_pm;
            }
            if (copy_to_user((void __user *)arg, data->shared_mem, sizeof(struct bmp390_shared_data))) {
                mutex_unlock(&data->lock);
                ret = -EFAULT;
                goto out_pm;
            }
            mutex_unlock(&data->lock);
            break;
        }
    case BMP390_IOC_FLUSH_FIFO:
        ret = bmp390_flush_fifo(data);
        break;
    case BMP390_IOC_SET_ODR:
        {
            int odr;
            if (copy_from_user(&odr, (void __user *)arg, sizeof(odr))) {
                ret = -EFAULT;
                goto out_pm;
            }
            if (odr < BMP390_ODR_0P78_HZ || odr > BMP390_ODR_100_HZ) {
                ret = -EINVAL;
                goto out_pm;
            }
            mutex_lock(&data->lock);
            ret = bmp390_set_odr(data, odr);
            mutex_unlock(&data->lock);
            break;
        }
    case BMP390_IOC_GET_CALIB:
        {
            mutex_lock(&data->lock);
            if (copy_to_user((void __user *)arg, &data->calib, sizeof(struct bmp390_calib_param))) {
                mutex_unlock(&data->lock);
                ret = -EFAULT;
                goto out_pm;
            }
            mutex_unlock(&data->lock);
            break;
        }
    default:
        ret = -ENOTTY;
        goto out_pm;
    }

out_pm:
    pm_runtime_put(indio_dev->dev.parent);
    return ret;
}

static const struct file_operations bmp390_fops = {
    .unlocked_ioctl = bmp390_ioctl,
    .open = bmp390_open,
    .release = bmp390_release,
};

static struct miscdevice bmp390_miscdev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "bmp390",
    .fops = &bmp390_fops,
};

/**
 * bmp390_init - Module initialization
 * Return: 0 on success, negative error code on failure
 */
static int __init bmp390_init(void)
{
    int ret;

    ret = i2c_add_driver(&bmp390_i2c_driver);
    if (ret < 0) {
        pr_err("BMP390: Failed to register I2C driver\n");
        return ret;
    }

    ret = spi_register_driver(&bmp390_spi_driver);
    if (ret < 0) {
        pr_err("BMP390: Failed to register SPI driver\n");
        i2c_del_driver(&bmp390_i2c_driver);
        return ret;
    }

    ret = misc_register(&bmp390_miscdev);
    if (ret < 0) {
        pr_err("BMP390: Failed to register misc device\n");
        spi_unregister_driver(&bmp390_spi_driver);
        i2c_del_driver(&bmp390_i2c_driver);
        return ret;
    }

    pr_info("BMP390 driver registered\n");
    return 0;
}

/**
 * bmp390_exit - Module cleanup
 */
static void __exit bmp390_exit(void)
{
    misc_deregister(&bmp390_miscdev);
    spi_unregister_driver(&bmp390_spi_driver);
    i2c_del_driver(&bmp390_i2c_driver);
    pr_info("BMP390 driver unloaded\n");
}

module_init(bmp390_init);
module_exit(bmp390_exit);

MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("BMP390 IIO Driver");
MODULE_LICENSE("GPL v2");