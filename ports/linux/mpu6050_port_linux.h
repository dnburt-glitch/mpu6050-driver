/* Linux i2c-dev port (Raspberry Pi, Jetson, BeagleBone, ...). */
#ifndef MPU6050_PORT_LINUX_H
#define MPU6050_PORT_LINUX_H

#include "mpu6050.h"

/** Open e.g. "/dev/i2c-1". Returns a file descriptor (>= 0) or -1. */
int mpu6050_linux_open(const char *bus_path);
void mpu6050_linux_close(int fd);

/** Fill hal with callbacks that use the given fd. fd_storage must outlive the driver. */
void mpu6050_linux_hal(mpu6050_hal_t *hal, int *fd_storage);

#endif
