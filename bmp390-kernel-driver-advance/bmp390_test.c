/*
 * bmp390_test.c - User-space test for BMP390 kernel driver
 * Compile: gcc -o bmp390_test bmp390_test.c -Wall -Wextra
 * Run: ./bmp390_test [times] [interval_ms]
 */
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <errno.h>
#include <time.h>

#define IIO_DEV "iio:device0"
#define TEMP_RAW_PATH "/sys/bus/iio/devices/" IIO_DEV "/in_temp_raw"
#define PRESS_RAW_PATH "/sys/bus/iio/devices/" IIO_DEV "/in_pressure_raw"
#define TEMP_INPUT_PATH "/sys/bus/iio/devices/" IIO_DEV "/in_temp_input"
#define PRESS_INPUT_PATH "/sys/bus/iio/devices/" IIO_DEV "/in_pressure_input"
#define OSR_PRESS_PATH "/sys/bus/iio/devices/" IIO_DEV "/in_pressure_oversampling_ratio"
#define SAMPLING_FREQ_PATH "/sys/bus/iio/devices/" IIO_DEV "/sampling_frequency"
#define FIFO_WATERMARK_PATH "/sys/bus/iio/devices/" IIO_DEV "/buffer/watermark"

#define BMP390_IOC_MAGIC 'b'
#define BMP390_IOC_SET_MODE _IOW(BMP390_IOC_MAGIC, 1, int)
#define BMP390_IOC_GET_SHARED_DATA _IOR(BMP390_IOC_MAGIC, 2, struct bmp390_shared_data)
#define BMP390_IOC_FLUSH_FIFO _IO(BMP390_IOC_MAGIC, 3)
#define BMP390_IOC_SET_ODR _IOW(BMP390_IOC_MAGIC, 4, int)
#define BMP390_IOC_GET_CALIB _IOR(BMP390_IOC_MAGIC, 5, struct bmp390_calib_param)

struct bmp390_shared_data {
    int32_t temperature;
    int32_t pressure;
    uint64_t timestamp;
    uint8_t status;
};

struct bmp390_calib_param {
    int16_t t1, t2;
    uint16_t t3;
    int16_t p1, p2;
    uint16_t p3, p4;
    int16_t p5, p6;
    uint16_t p7, p8, p9, p10, p11;
};

static volatile sig_atomic_t running = 1;
static volatile sig_atomic_t data_ready = 0;

static void sigio_handler(int sig)
{
    data_ready = 1;
}

static void sigint_handler(int sig)
{
    running = 0;
}

static int read_sysfs(const char *path, char *buf, size_t size)
{
    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "Failed to open %s: %s\n", path, strerror(errno));
        return -1;
    }
    ssize_t len = read(fd, buf, size - 1);
    if (len < 0) {
        fprintf(stderr, "Failed to read %s: %s\n", path, strerror(errno));
        close(fd);
        return -1;
    }
    buf[len] = '\0';
    close(fd);
    return 0;
}

static int write_sysfs(const char *path, const char *val)
{
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        fprintf(stderr, "Failed to open %s for writing: %s\n", path, strerror(errno));
        return -1;
    }
    ssize_t len = write(fd, val, strlen(val));
    if (len < 0) {
        fprintf(stderr, "Failed to write to %s: %s\n", path, strerror(errno));
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}

int main(int argc, char **argv)
{
    char buf[256];
    int times = 3;
    int interval_ms = 1000;
    int fd;
    struct bmp390_shared_data shared_data;
    struct bmp390_calib_param calib;
    struct sigaction sa_int, sa_io;

    /* Parse command-line arguments */
    if (argc > 1)
        times = atoi(argv[1]);
    if (argc > 2)
        interval_ms = atoi(argv[2]);

    /* Setup signal handlers */
    sa_int.sa_handler = sigint_handler;
    sigemptyset(&sa_int.sa_mask);
    sa_int.sa_flags = 0;
    sigaction(SIGINT, &sa_int, NULL);

    sa_io.sa_handler = sigio_handler;
    sigemptyset(&sa_io.sa_mask);
    sa_io.sa_flags = 0;
    sigaction(SIGIO, &sa_io, NULL);

    /* Open device */
    fd = open("/dev/bmp390", O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Failed to open /dev/bmp390: %s\n", strerror(errno));
        return -1;
    }

    /* Setup async I/O */
    if (fcntl(fd, F_SETOWN, getpid()) < 0) {
        fprintf(stderr, "Failed to set F_SETOWN: %s\n", strerror(errno));
        close(fd);
        return -1;
    }
    if (fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | FASYNC) < 0) {
        fprintf(stderr, "Failed to set FASYNC: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    printf("BMP390 Kernel Driver Test (times=%d, interval=%dms)\n", times, interval_ms);

    /* Set example configuration */
    if (write_sysfs(OSR_PRESS_PATH, "16") == 0)
        printf("Set pressure oversampling to 16\n");
    else
        fprintf(stderr, "Failed to set pressure oversampling\n");

    if (write_sysfs(SAMPLING_FREQ_PATH, "25") == 0)
        printf("Set sampling frequency to 25 Hz\n");
    else
        fprintf(stderr, "Failed to set sampling frequency\n");

    if (write_sysfs(FIFO_WATERMARK_PATH, "1") == 0)
        printf("Set FIFO watermark to 1\n");
    else
        fprintf(stderr, "Failed to set FIFO watermark\n");

    /* Test IOCTL: Set ODR */
    int odr = 5; /* BMP390_ODR_25_HZ */
    if (ioctl(fd, BMP390_IOC_SET_ODR, &odr) < 0)
        fprintf(stderr, "Failed to set ODR: %s\n", strerror(errno));
    else
        printf("Set ODR to 25 Hz via IOCTL\n");

    /* Test IOCTL: Get calibration data */
    if (ioctl(fd, BMP390_IOC_GET_CALIB, &calib) == 0) {
        printf("Calibration Parameters:\n");
        printf("  T1: %d, T2: %d, T3: %u\n", calib.t1, calib.t2, calib.t3);
        printf("  P1: %d, P2: %d, P3: %u, P4: %u\n", calib.p1, calib.p2, calib.p3, calib.p4);
        printf("  P5: %d, P6: %d, P7: %u, P8: %u\n", calib.p5, calib.p6, calib.p7, calib.p8);
        printf("  P9: %u, P10: %u, P11: %u\n", calib.p9, calib.p10, calib.p11);
    } else {
        fprintf(stderr, "Failed to get calibration data: %s\n", strerror(errno));
    }

    /* Test periodic mode */
    int mode = 1; /* BMP390_OP_MODE_PERIODIC */
    if (ioctl(fd, BMP390_IOC_SET_MODE, &mode) < 0) {
        fprintf(stderr, "Failed to set periodic mode: %s\n", strerror(errno));
    } else {
        printf("Set to periodic mode\n");
    }

    for (int i = 0; i < times && running; i++) {
        if (data_ready) {
            if (ioctl(fd, BMP390_IOC_GET_SHARED_DATA, &shared_data) == 0) {
                printf("Shared: Temp=%d C, Press=%d Pa, Time=%llu\n",
                       shared_data.temperature, shared_data.pressure, shared_data.timestamp);
            } else {
                fprintf(stderr, "Failed to get shared data: %s\n", strerror(errno));
            }
            data_ready = 0;
        }
        if (read_sysfs(TEMP_INPUT_PATH, buf, sizeof(buf)) == 0)
            printf("Temperature: %s C\n", buf);
        if (read_sysfs(PRESS_INPUT_PATH, buf, sizeof(buf)) == 0)
            printf("Pressure: %s Pa\n", buf);
        usleep(interval_ms * 1000);
    }

    /* Test async mode */
    mode = 2; /* BMP390_OP_MODE_ASYNC */
    if (ioctl(fd, BMP390_IOC_SET_MODE, &mode) < 0) {
        fprintf(stderr, "Failed to set async mode: %s\n", strerror(errno));
    } else {
        printf("Set to async mode\n");
    }

    sleep(2); /* Wait for async data */
    if (data_ready) {
        if (ioctl(fd, BMP390_IOC_GET_SHARED_DATA, &shared_data) == 0) {
            printf("Async: Temp=%d C, Press=%d Pa, Time=%llu\n",
                   shared_data.temperature, shared_data.pressure, shared_data.timestamp);
        } else {
            fprintf(stderr, "Failed to get async shared data: %s\n", strerror(errno));
        }
        data_ready = 0;
    }

    /* Test FIFO flush */
    if (ioctl(fd, BMP390_IOC_FLUSH_FIFO) < 0) {
        fprintf(stderr, "Failed to flush FIFO: %s\n", strerror(errno));
    } else {
        printf("FIFO flushed\n");
    }

    /* Read raw data for test */
    if (read_sysfs(TEMP_RAW_PATH, buf, sizeof(buf)) == 0)
        printf("Raw Temperature: %s\n", buf);
    else
        fprintf(stderr, "Failed to read raw temperature\n");
    if (read_sysfs(PRESS_RAW_PATH, buf, sizeof(buf)) == 0)
        printf("Raw Pressure: %s\n", buf);
    else
        fprintf(stderr, "Failed to read raw pressure\n");

    close(fd);
    printf("Test completed\n");
    return 0;
}