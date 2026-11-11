/* Read the MPU-6050 on a Linux board and print 100 Hz samples.
 *
 *   make linux-example && ./build/mpu6050_linux /dev/i2c-1
 */
#include <stdio.h>
#include <stdlib.h>

#include "mpu6050.h"
#include "mpu6050_port_linux.h"

int main(int argc, char **argv)
{
    const char *bus = argc > 1 ? argv[1] : "/dev/i2c-1";
    mpu6050_config_t cfg = MPU6050_CONFIG_DEFAULT;
    mpu6050_hal_t hal;
    mpu6050_t imu;
    mpu6050_status_t st;
    int fd, i;

    fd = mpu6050_linux_open(bus);
    if (fd < 0) {
        perror(bus);
        return EXIT_FAILURE;
    }
    mpu6050_linux_hal(&hal, &fd);

    cfg.sample_rate_hz = 100u;
    cfg.data_ready_irq = true;  /* needed for mpu6050_data_ready() polling */
    st = mpu6050_init(&imu, &hal, MPU6050_ADDR_AD0_LOW, &cfg);
    if (st != MPU6050_OK) {
        fprintf(stderr, "init failed: %s\n", mpu6050_status_str(st));
        mpu6050_linux_close(fd);
        return EXIT_FAILURE;
    }

    puts("Keep the sensor still: calibrating gyro...");
    st = mpu6050_calibrate_gyro(&imu, 200u);
    if (st != MPU6050_OK) {
        fprintf(stderr, "calibration failed: %s\n", mpu6050_status_str(st));
    }

    for (i = 0; i < 1000; ++i) {
        mpu6050_data_t d;
        bool ready = false;
        while (mpu6050_data_ready(&imu, &ready) == MPU6050_OK && !ready) {
            hal.delay_ms(hal.ctx, 1u);
        }
        if (mpu6050_read(&imu, &d) != MPU6050_OK) {
            fputs("read error\n", stderr);
            continue;
        }
        printf("a[m/s2] %7.3f %7.3f %7.3f | g[rad/s] %7.4f %7.4f %7.4f | %5.1f C\n",
               (double)d.accel_mps2[0], (double)d.accel_mps2[1], (double)d.accel_mps2[2],
               (double)d.gyro_rads[0], (double)d.gyro_rads[1], (double)d.gyro_rads[2],
               (double)d.temp_c);
    }
    mpu6050_linux_close(fd);
    return EXIT_SUCCESS;
}
