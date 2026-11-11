/**
 * @file mpu6050.c
 * @brief MPU-6050 driver implementation. See mpu6050.h for the API contract.
 */
#include "mpu6050.h"

#define GRAVITY_MPS2   9.80665f
#define DEG_TO_RAD     0.017453292519943295f
#define RESET_DELAY_MS 100u
#define WAKE_DELAY_MS  10u
#define BURST_LEN      14u

static const float k_accel_lsb_per_g[] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
static const float k_gyro_lsb_per_dps[] = { 131.0f, 65.5f, 32.8f, 16.4f };

/* ---- Low-level helpers -------------------------------------------------- */

static mpu6050_status_t read_regs(const mpu6050_t *dev, uint8_t reg, uint8_t *buf, size_t len)
{
    int rc = dev->hal.read(dev->hal.ctx, dev->addr, reg, buf, len);
    return rc == 0 ? MPU6050_OK : MPU6050_ERR_BUS;
}

static mpu6050_status_t write_reg(const mpu6050_t *dev, uint8_t reg, uint8_t value)
{
    int rc = dev->hal.write(dev->hal.ctx, dev->addr, reg, &value, 1u);
    return rc == 0 ? MPU6050_OK : MPU6050_ERR_BUS;
}

/* Write a configuration register and read it back, so a bus glitch or a
 * marginal pull-up cannot leave the sensor silently misconfigured. */
static mpu6050_status_t write_reg_verified(const mpu6050_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t readback = 0u;
    mpu6050_status_t st = write_reg(dev, reg, value);

    if (st == MPU6050_OK) {
        st = read_regs(dev, reg, &readback, 1u);
    }
    if (st == MPU6050_OK && readback != value) {
        st = MPU6050_ERR_BUS;
    }
    return st;
}

static int16_t be16(const uint8_t *p)
{
    return (int16_t)(uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/* ---- Configuration ------------------------------------------------------ */

static uint32_t gyro_output_rate_hz(mpu6050_dlpf_t dlpf)
{
    /* DLPF_CFG 0 runs the gyro output at 8 kHz, otherwise at 1 kHz. */
    return dlpf == MPU6050_DLPF_260HZ ? 8000u : 1000u;
}

static mpu6050_status_t sample_rate_divider(const mpu6050_config_t *cfg, uint8_t *div)
{
    const uint32_t base = gyro_output_rate_hz(cfg->dlpf);
    uint32_t ratio;

    if (cfg->sample_rate_hz == 0u || cfg->sample_rate_hz > base) {
        return MPU6050_ERR_PARAM;
    }
    /* rate = base / (1 + div): pick the nearest achievable rate. */
    ratio = (base + cfg->sample_rate_hz / 2u) / cfg->sample_rate_hz;
    if (ratio == 0u || ratio - 1u > 255u) {
        return MPU6050_ERR_PARAM;
    }
    *div = (uint8_t)(ratio - 1u);
    return MPU6050_OK;
}

static mpu6050_status_t validate_config(const mpu6050_config_t *cfg)
{
    uint8_t div = 0u;

    if ((unsigned)cfg->accel_range > (unsigned)MPU6050_ACCEL_16G
        || (unsigned)cfg->gyro_range > (unsigned)MPU6050_GYRO_2000DPS
        || (unsigned)cfg->dlpf > (unsigned)MPU6050_DLPF_5HZ) {
        return MPU6050_ERR_PARAM;
    }
    return sample_rate_divider(cfg, &div);
}

typedef struct {
    uint8_t reg;
    uint8_t value;
    bool    verify;
} reg_write_t;

static mpu6050_status_t apply_config(mpu6050_t *dev, const mpu6050_config_t *cfg)
{
    uint8_t div = 0u;
    reg_write_t seq[6];
    size_t i;
    mpu6050_status_t st = validate_config(cfg);

    if (st != MPU6050_OK) {
        return st;
    }
    (void)sample_rate_divider(cfg, &div);  /* cannot fail after validation */

    seq[0] = (reg_write_t){ MPU6050_REG_CONFIG, (uint8_t)cfg->dlpf, true };
    seq[1] = (reg_write_t){ MPU6050_REG_SMPLRT_DIV, div, true };
    seq[2] = (reg_write_t){ MPU6050_REG_GYRO_CONFIG, (uint8_t)((unsigned)cfg->gyro_range << 3), true };
    seq[3] = (reg_write_t){ MPU6050_REG_ACCEL_CONFIG, (uint8_t)((unsigned)cfg->accel_range << 3), true };
    /* INT pin: push-pull, active high, flag cleared by reading INT_STATUS. */
    seq[4] = (reg_write_t){ MPU6050_REG_INT_PIN_CFG, 0x00u, false };
    seq[5] = (reg_write_t){ MPU6050_REG_INT_ENABLE,
                            cfg->data_ready_irq ? MPU6050_INT_DATA_RDY : 0x00u, true };

    for (i = 0u; i < sizeof seq / sizeof seq[0]; ++i) {
        st = seq[i].verify ? write_reg_verified(dev, seq[i].reg, seq[i].value)
                           : write_reg(dev, seq[i].reg, seq[i].value);
        if (st != MPU6050_OK) {
            return st;
        }
    }

    dev->cfg = *cfg;
    dev->accel_lsb_per_g = k_accel_lsb_per_g[cfg->accel_range];
    dev->gyro_lsb_per_dps = k_gyro_lsb_per_dps[cfg->gyro_range];
    return MPU6050_OK;
}

/* ---- Public API --------------------------------------------------------- */

mpu6050_status_t mpu6050_init(mpu6050_t *dev, const mpu6050_hal_t *hal,
                              uint8_t addr, const mpu6050_config_t *cfg)
{
    uint8_t who = 0u;
    mpu6050_status_t st;

    if (dev == NULL || hal == NULL || cfg == NULL
        || hal->read == NULL || hal->write == NULL || hal->delay_ms == NULL) {
        return MPU6050_ERR_NULL;
    }
    st = validate_config(cfg);
    if (st != MPU6050_OK) {
        return st;
    }

    dev->hal = *hal;
    dev->addr = addr;
    dev->initialized = false;
    dev->gyro_offset_raw[0] = 0.0f;
    dev->gyro_offset_raw[1] = 0.0f;
    dev->gyro_offset_raw[2] = 0.0f;

    st = read_regs(dev, MPU6050_REG_WHO_AM_I, &who, 1u);
    if (st != MPU6050_OK) {
        return st;
    }
    if (who != MPU6050_WHO_AM_I_VALUE) {
        return MPU6050_ERR_WHO_AM_I;
    }

    /* Full reset to a known state, then wake on the gyro PLL (more stable
     * than the internal 8 MHz oscillator). The reset leaves SLEEP set. */
    st = write_reg(dev, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET);
    if (st != MPU6050_OK) {
        return st;
    }
    dev->hal.delay_ms(dev->hal.ctx, RESET_DELAY_MS);

    st = write_reg_verified(dev, MPU6050_REG_PWR_MGMT_1, MPU6050_CLKSEL_PLL_XGYRO);
    if (st != MPU6050_OK) {
        return st;
    }
    dev->hal.delay_ms(dev->hal.ctx, WAKE_DELAY_MS);

    st = apply_config(dev, cfg);
    if (st == MPU6050_OK) {
        dev->initialized = true;
    }
    return st;
}

mpu6050_status_t mpu6050_configure(mpu6050_t *dev, const mpu6050_config_t *cfg)
{
    if (dev == NULL || cfg == NULL) {
        return MPU6050_ERR_NULL;
    }
    if (!dev->initialized) {
        return MPU6050_ERR_NOT_READY;
    }
    return apply_config(dev, cfg);
}

mpu6050_status_t mpu6050_read_raw(mpu6050_t *dev, mpu6050_raw_t *raw)
{
    uint8_t buf[BURST_LEN];
    mpu6050_status_t st;
    size_t i;

    if (dev == NULL || raw == NULL) {
        return MPU6050_ERR_NULL;
    }
    if (!dev->initialized) {
        return MPU6050_ERR_NOT_READY;
    }
    /* Single burst so all axes come from the same sample. */
    st = read_regs(dev, MPU6050_REG_ACCEL_XOUT_H, buf, sizeof buf);
    if (st != MPU6050_OK) {
        return st;
    }
    for (i = 0u; i < 3u; ++i) {
        raw->accel[i] = be16(&buf[2u * i]);
        raw->gyro[i] = be16(&buf[8u + 2u * i]);
    }
    raw->temp = be16(&buf[6]);
    return MPU6050_OK;
}

void mpu6050_convert(const mpu6050_t *dev, const mpu6050_raw_t *raw, mpu6050_data_t *out)
{
    size_t i;
    for (i = 0u; i < 3u; ++i) {
        out->accel_mps2[i] = (float)raw->accel[i] / dev->accel_lsb_per_g * GRAVITY_MPS2;
        out->gyro_rads[i] = ((float)raw->gyro[i] - dev->gyro_offset_raw[i])
                            / dev->gyro_lsb_per_dps * DEG_TO_RAD;
    }
    out->temp_c = (float)raw->temp / 340.0f + 36.53f;  /* datasheet formula */
}

mpu6050_status_t mpu6050_read(mpu6050_t *dev, mpu6050_data_t *out)
{
    mpu6050_raw_t raw;
    mpu6050_status_t st;

    if (out == NULL) {
        return MPU6050_ERR_NULL;
    }
    st = mpu6050_read_raw(dev, &raw);
    if (st == MPU6050_OK) {
        mpu6050_convert(dev, &raw, out);
    }
    return st;
}

mpu6050_status_t mpu6050_data_ready(mpu6050_t *dev, bool *ready)
{
    uint8_t status = 0u;
    mpu6050_status_t st;

    if (dev == NULL || ready == NULL) {
        return MPU6050_ERR_NULL;
    }
    if (!dev->initialized) {
        return MPU6050_ERR_NOT_READY;
    }
    st = read_regs(dev, MPU6050_REG_INT_STATUS, &status, 1u);
    if (st == MPU6050_OK) {
        *ready = (status & MPU6050_INT_DATA_RDY) != 0u;
    }
    return st;
}

mpu6050_status_t mpu6050_calibrate_gyro(mpu6050_t *dev, uint16_t n_samples)
{
    int32_t sum[3] = { 0, 0, 0 };
    mpu6050_raw_t raw;
    mpu6050_status_t st;
    uint32_t period_ms;
    uint16_t n;
    size_t i;

    if (dev == NULL) {
        return MPU6050_ERR_NULL;
    }
    if (!dev->initialized) {
        return MPU6050_ERR_NOT_READY;
    }
    if (n_samples == 0u) {
        return MPU6050_ERR_PARAM;
    }
    period_ms = (1000u + dev->cfg.sample_rate_hz - 1u) / dev->cfg.sample_rate_hz;

    for (n = 0u; n < n_samples; ++n) {
        dev->hal.delay_ms(dev->hal.ctx, period_ms);
        st = mpu6050_read_raw(dev, &raw);
        if (st != MPU6050_OK) {
            return st;  /* previous offsets are kept on failure */
        }
        for (i = 0u; i < 3u; ++i) {
            sum[i] += raw.gyro[i];
        }
    }
    for (i = 0u; i < 3u; ++i) {
        dev->gyro_offset_raw[i] = (float)sum[i] / (float)n_samples;
    }
    return MPU6050_OK;
}

float mpu6050_actual_rate_hz(const mpu6050_config_t *cfg)
{
    uint8_t div = 0u;
    if (cfg == NULL || sample_rate_divider(cfg, &div) != MPU6050_OK) {
        return 0.0f;
    }
    return (float)gyro_output_rate_hz(cfg->dlpf) / (float)(1u + div);
}

const char *mpu6050_status_str(mpu6050_status_t status)
{
    switch (status) {
    case MPU6050_OK:            return "ok";
    case MPU6050_ERR_NULL:      return "null argument";
    case MPU6050_ERR_BUS:       return "i2c bus error";
    case MPU6050_ERR_WHO_AM_I:  return "unexpected WHO_AM_I";
    case MPU6050_ERR_PARAM:     return "invalid parameter";
    case MPU6050_ERR_NOT_READY: return "not initialised";
    default:                    return "unknown";
    }
}
