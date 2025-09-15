/*
 * bmp390.c - Main BMP390 kernel driver
 * Integrates core, IIO, I2C/SPI.
 */

#include "bmp390.h"

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

    pr_info("BMP390 driver registered\n");
    return 0;
}

static void __exit bmp390_exit(void)
{
    spi_unregister_driver(&bmp390_spi_driver);
    i2c_del_driver(&bmp390_i2c_driver);
    pr_info("BMP390 driver unloaded\n");
}

module_init(bmp390_init);
module_exit(bmp390_exit);

MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("BMP390 IIO Driver");
MODULE_LICENSE("GPL v2");