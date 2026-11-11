/* Linux i2c-dev port. Register reads use a single I2C_RDWR transaction with a
 * repeated start, which is what the MPU-6050 expects for burst reads. */
#include "mpu6050_port_linux.h"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#define MAX_WRITE 16u

static int linux_read(void *ctx, uint8_t dev_addr, uint8_t reg, uint8_t *buf, size_t len)
{
    int fd = *(int *)ctx;
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data xfer;

    if (len > 0xFFFFu) {
        return -1;
    }
    msgs[0].addr = dev_addr;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = &reg;
    msgs[1].addr = dev_addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = (__u16)len;
    msgs[1].buf = buf;
    xfer.msgs = msgs;
    xfer.nmsgs = 2;
    return ioctl(fd, I2C_RDWR, &xfer) == 2 ? 0 : -1;
}

static int linux_write(void *ctx, uint8_t dev_addr, uint8_t reg, const uint8_t *buf, size_t len)
{
    int fd = *(int *)ctx;
    uint8_t frame[1u + MAX_WRITE];
    struct i2c_msg msg;
    struct i2c_rdwr_ioctl_data xfer;

    if (len > MAX_WRITE) {
        return -1;
    }
    frame[0] = reg;
    memcpy(&frame[1], buf, len);
    msg.addr = dev_addr;
    msg.flags = 0;
    msg.len = (__u16)(len + 1u);
    msg.buf = frame;
    xfer.msgs = &msg;
    xfer.nmsgs = 1;
    return ioctl(fd, I2C_RDWR, &xfer) == 1 ? 0 : -1;
}

static void linux_delay_ms(void *ctx, uint32_t ms)
{
    struct timespec ts;
    (void)ctx;
    ts.tv_sec = (time_t)(ms / 1000u);
    ts.tv_nsec = (long)(ms % 1000u) * 1000000L;
    while (nanosleep(&ts, &ts) != 0) {
        /* resume after signal interruption */
    }
}

int mpu6050_linux_open(const char *bus_path)
{
    return open(bus_path, O_RDWR);
}

void mpu6050_linux_close(int fd)
{
    if (fd >= 0) {
        close(fd);
    }
}

void mpu6050_linux_hal(mpu6050_hal_t *hal, int *fd_storage)
{
    hal->read = linux_read;
    hal->write = linux_write;
    hal->delay_ms = linux_delay_ms;
    hal->ctx = fd_storage;
}
