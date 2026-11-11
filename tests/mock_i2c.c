#include "mock_i2c.h"

#include <string.h>

static void power_on_defaults(mock_mpu_t *m)
{
    memset(m->regs, 0, sizeof m->regs);
    m->regs[MPU6050_REG_PWR_MGMT_1] = MPU6050_PWR1_SLEEP;  /* 0x40 after reset */
    m->regs[MPU6050_REG_WHO_AM_I] = MPU6050_WHO_AM_I_VALUE;
}

void mock_init(mock_mpu_t *m, uint8_t addr)
{
    memset(m, 0, sizeof *m);
    m->addr = addr;
    m->fail_read_reg = m->fail_write_reg = m->stuck_reg = -1;
    power_on_defaults(m);
}

static int mock_read(void *ctx, uint8_t dev_addr, uint8_t reg, uint8_t *buf, size_t len)
{
    mock_mpu_t *m = (mock_mpu_t *)ctx;
    size_t i;
    if (dev_addr != m->addr || (size_t)reg + len > sizeof m->regs) {
        return -1;  /* NACK */
    }
    if (m->fail_read_reg >= 0 && (unsigned)m->fail_read_reg >= reg
        && (unsigned)m->fail_read_reg < reg + len) {
        return -1;
    }
    for (i = 0; i < len; ++i) {
        uint8_t r = (uint8_t)(reg + i);
        if (r == MPU6050_REG_INT_STATUS) {
            buf[i] = m->data_ready ? MPU6050_INT_DATA_RDY : 0u;
            m->data_ready = false;  /* cleared on read */
        } else {
            buf[i] = m->regs[r];
        }
    }
    return 0;
}

static int mock_write(void *ctx, uint8_t dev_addr, uint8_t reg, const uint8_t *buf, size_t len)
{
    mock_mpu_t *m = (mock_mpu_t *)ctx;
    size_t i;
    if (dev_addr != m->addr || (size_t)reg + len > sizeof m->regs) {
        return -1;
    }
    if (m->fail_write_reg == reg) {
        return -1;
    }
    for (i = 0; i < len; ++i) {
        uint8_t r = (uint8_t)(reg + i);
        if (m->n_writes < MOCK_LOG_MAX) {
            m->log[m->n_writes].reg = r;
            m->log[m->n_writes].value = buf[i];
            m->n_writes++;
        }
        if (r == m->stuck_reg) {
            continue;
        }
        if (r == MPU6050_REG_PWR_MGMT_1 && (buf[i] & MPU6050_PWR1_DEVICE_RESET)) {
            power_on_defaults(m);  /* reset bit self-clears */
            continue;
        }
        if (r == MPU6050_REG_WHO_AM_I) {
            continue;  /* read-only */
        }
        m->regs[r] = buf[i];
    }
    return 0;
}

static void mock_delay(void *ctx, uint32_t ms)
{
    ((mock_mpu_t *)ctx)->total_delay_ms += ms;
}

mpu6050_hal_t mock_hal(mock_mpu_t *m)
{
    mpu6050_hal_t hal;
    hal.read = mock_read;
    hal.write = mock_write;
    hal.delay_ms = mock_delay;
    hal.ctx = m;
    return hal;
}

static void put_be16(uint8_t *p, int16_t v)
{
    uint16_t u = (uint16_t)v;
    p[0] = (uint8_t)(u >> 8);
    p[1] = (uint8_t)(u & 0xFFu);
}

void mock_set_sample(mock_mpu_t *m, const int16_t accel[3], int16_t temp, const int16_t gyro[3])
{
    uint8_t *p = &m->regs[MPU6050_REG_ACCEL_XOUT_H];
    int i;
    for (i = 0; i < 3; ++i) {
        put_be16(&p[2 * i], accel[i]);
        put_be16(&p[8 + 2 * i], gyro[i]);
    }
    put_be16(&p[6], temp);
}

int mock_first_write(const mock_mpu_t *m, uint8_t reg)
{
    size_t i;
    for (i = 0; i < m->n_writes; ++i) {
        if (m->log[i].reg == reg) {
            return (int)i;
        }
    }
    return -1;
}
