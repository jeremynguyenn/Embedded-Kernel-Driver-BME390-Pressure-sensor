/*
 * bmp390_test.c - User-space test for BMP390 kernel driver
 * Compile: gcc bmp390_test.c -o bmp390_test
 * Run: ./bmp390_test [times]
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#define IIO_DEV "iio:device0"
#define TEMP_RAW_PATH "/sys/bus/iio/devices/" IIO_DEV "/in_temp_raw"
#define PRESS_RAW_PATH "/sys/bus/iio/devices/" IIO_DEV "/in_pressure_raw"
#define TEMP_INPUT_PATH "/sys/bus/iio/devices/" IIO_DEV "/in_temp_input"
#define PRESS_INPUT_PATH "/sys/bus/iio/devices/" IIO_DEV "/in_pressure_input"
#define OSR_PRESS_PATH "/sys/bus/iio/devices/" IIO_DEV "/in_pressure_oversampling_ratio"
#define SAMPLING_FREQ_PATH "/sys/bus/iio/devices/" IIO_DEV "/sampling_frequency"
#define FIFO_WATERMARK_PATH "/sys/bus/iio/devices/" IIO_DEV "/buffer/watermark"  /* Assume attr for FIFO */

static int read_sysfs(const char *path, char *buf, size_t size)
{
    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        perror("open read");
        return -1;
    }
    ssize_t len = read(fd, buf, size - 1);
    close(fd);
    if (len < 0) {
        perror("read");
        return -1;
    }
    buf[len] = '\0';
    return 0;
}

static int write_sysfs(const char *path, const char *val)
{
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("open write");
        return -1;
    }
    ssize_t len = write(fd, val, strlen(val));
    close(fd);
    return (len < 0) ? -1 : 0;
}

int main(int argc, char **argv)
{
    char buf[256];
    int times = 3;

    if (argc > 1)
        times = atoi(argv[1]);

    printf("BMP390 Kernel Driver Test\n");

    /* Set example config */
    if (write_sysfs(OSR_PRESS_PATH, "16") == 0)
        printf("Set pressure oversampling to 16\n");
    if (write_sysfs(SAMPLING_FREQ_PATH, "25") == 0)
        printf("Set sampling frequency to 25 Hz\n");
    if (write_sysfs(FIFO_WATERMARK_PATH, "1") == 0)
        printf("Set FIFO watermark to 1\n");

    for (int i = 0; i < times; i++) {
        if (read_sysfs(TEMP_INPUT_PATH, buf, sizeof(buf)) == 0)
            printf("Temperature: %s C\n", buf);
        if (read_sysfs(PRESS_INPUT_PATH, buf, sizeof(buf)) == 0)
            printf("Pressure: %s Pa\n", buf);

        sleep(1);
    }

    /* Read raw for test */
    if (read_sysfs(TEMP_RAW_PATH, buf, sizeof(buf)) == 0)
        printf("Raw Temperature: %s\n", buf);
    if (read_sysfs(PRESS_RAW_PATH, buf, sizeof(buf)) == 0)
        printf("Raw Pressure: %s\n", buf);

    return 0;
}