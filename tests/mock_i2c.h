/* Register-level MPU-6050 emulator used by the host unit tests. */
#ifndef MOCK_I2C_H
#define MOCK_I2C_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mpu6050.h"

#define MOCK_LOG_MAX 64

typedef struct {
    uint8_t  reg;
    uint8_t  value;
} mock_write_t;

typedef struct {
    uint8_t      addr;               /* address the emulated chip answers on */
    uint8_t      regs[128];
    mock_write_t log[MOCK_LOG_MAX];  /* every single-byte write, in order */
    size_t       n_writes;
    uint32_t     total_delay_ms;
    int          fail_read_reg;      /* -1 = never; else reads of this reg fail */
    int          fail_write_reg;     /* -1 = never */
    int          stuck_reg;          /* -1 = never; writes to this reg are ignored */
    bool         data_ready;         /* sets INT_STATUS.DATA_RDY on next read */
} mock_mpu_t;

void mock_init(mock_mpu_t *m, uint8_t addr);
mpu6050_hal_t mock_hal(mock_mpu_t *m);
void mock_set_sample(mock_mpu_t *m, const int16_t accel[3], int16_t temp, const int16_t gyro[3]);
/* Index of the first logged write to reg, or -1. */
int mock_first_write(const mock_mpu_t *m, uint8_t reg);

#endif
