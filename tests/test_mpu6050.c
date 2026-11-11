/* Host unit tests for the MPU-6050 driver, run against a register-level mock.
 * No framework dependency: build with `make test`. */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "mock_i2c.h"
#include "mpu6050.h"

static int g_checks, g_failures;

#define CHECK(cond) do { \
    g_checks++; \
    if (!(cond)) { g_failures++; printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } \
} while (0)
#define CHECK_EQ_INT(a, b) do { \
    long a_ = (long)(a), b_ = (long)(b); g_checks++; \
    if (a_ != b_) { g_failures++; \
        printf("  FAIL %s:%d: %s == %ld, expected %ld\n", __FILE__, __LINE__, #a, a_, b_); } \
} while (0)
#define CHECK_NEAR(a, b, tol) do { \
    double a_ = (double)(a), b_ = (double)(b); g_checks++; \
    if (fabs(a_ - b_) > (tol)) { g_failures++; \
        printf("  FAIL %s:%d: %s == %g, expected %g\n", __FILE__, __LINE__, #a, a_, b_); } \
} while (0)
#define RUN(test) do { printf("%s\n", #test); test(); } while (0)

static mock_mpu_t mock;
static mpu6050_t dev;
static mpu6050_hal_t hal;
static const mpu6050_config_t default_cfg = MPU6050_CONFIG_DEFAULT;

static mpu6050_status_t setup(const mpu6050_config_t *cfg)
{
    mock_init(&mock, MPU6050_ADDR_AD0_LOW);
    hal = mock_hal(&mock);
    memset(&dev, 0, sizeof dev);
    return mpu6050_init(&dev, &hal, MPU6050_ADDR_AD0_LOW, cfg);
}

/* ---------------------------------------------------------------------- */

static void test_init_programs_registers(void)
{
    CHECK_EQ_INT(setup(&default_cfg), MPU6050_OK);
    CHECK(dev.initialized);
    CHECK_EQ_INT(mock.regs[MPU6050_REG_PWR_MGMT_1], MPU6050_CLKSEL_PLL_XGYRO);  /* awake, PLL */
    CHECK_EQ_INT(mock.regs[MPU6050_REG_CONFIG], 3);          /* DLPF 44 Hz */
    CHECK_EQ_INT(mock.regs[MPU6050_REG_SMPLRT_DIV], 4);      /* 1 kHz / (1 + 4) = 200 Hz */
    CHECK_EQ_INT(mock.regs[MPU6050_REG_GYRO_CONFIG], 0x08);  /* FS_SEL = 1, 500 dps */
    CHECK_EQ_INT(mock.regs[MPU6050_REG_ACCEL_CONFIG], 0x08); /* AFS_SEL = 1, 4 g */
    CHECK_EQ_INT(mock.regs[MPU6050_REG_INT_ENABLE], 0x00);
}

static void test_init_resets_before_configuring(void)
{
    setup(&default_cfg);
    CHECK_EQ_INT(mock.log[0].reg, MPU6050_REG_PWR_MGMT_1);
    CHECK_EQ_INT(mock.log[0].value, MPU6050_PWR1_DEVICE_RESET);
    CHECK(mock_first_write(&mock, MPU6050_REG_GYRO_CONFIG) > 0);
    CHECK(mock.total_delay_ms >= 100u);  /* datasheet reset time */
}

static void test_init_rejects_wrong_device(void)
{
    mock_init(&mock, MPU6050_ADDR_AD0_LOW);
    mock.regs[MPU6050_REG_WHO_AM_I] = 0x70;  /* e.g. an MPU-6500 */
    hal = mock_hal(&mock);
    CHECK_EQ_INT(mpu6050_init(&dev, &hal, MPU6050_ADDR_AD0_LOW, &default_cfg), MPU6050_ERR_WHO_AM_I);
    CHECK_EQ_INT(mock.n_writes, 0);  /* nothing written to an unknown chip */
    CHECK(!dev.initialized);
}

static void test_init_wrong_address_is_bus_error(void)
{
    mock_init(&mock, MPU6050_ADDR_AD0_LOW);
    hal = mock_hal(&mock);
    CHECK_EQ_INT(mpu6050_init(&dev, &hal, MPU6050_ADDR_AD0_HIGH, &default_cfg), MPU6050_ERR_BUS);
}

static void test_null_arguments(void)
{
    mpu6050_hal_t bad;
    mock_init(&mock, MPU6050_ADDR_AD0_LOW);
    hal = mock_hal(&mock);
    bad = hal;
    bad.delay_ms = NULL;
    CHECK_EQ_INT(mpu6050_init(NULL, &hal, MPU6050_ADDR_AD0_LOW, &default_cfg), MPU6050_ERR_NULL);
    CHECK_EQ_INT(mpu6050_init(&dev, NULL, MPU6050_ADDR_AD0_LOW, &default_cfg), MPU6050_ERR_NULL);
    CHECK_EQ_INT(mpu6050_init(&dev, &hal, MPU6050_ADDR_AD0_LOW, NULL), MPU6050_ERR_NULL);
    CHECK_EQ_INT(mpu6050_init(&dev, &bad, MPU6050_ADDR_AD0_LOW, &default_cfg), MPU6050_ERR_NULL);
    CHECK_EQ_INT(mpu6050_read(&dev, NULL), MPU6050_ERR_NULL);
}

static void test_use_before_init(void)
{
    mpu6050_data_t d;
    bool ready;
    memset(&dev, 0, sizeof dev);
    CHECK_EQ_INT(mpu6050_read(&dev, &d), MPU6050_ERR_NOT_READY);
    CHECK_EQ_INT(mpu6050_data_ready(&dev, &ready), MPU6050_ERR_NOT_READY);
    CHECK_EQ_INT(mpu6050_configure(&dev, &default_cfg), MPU6050_ERR_NOT_READY);
    CHECK_EQ_INT(mpu6050_calibrate_gyro(&dev, 10), MPU6050_ERR_NOT_READY);
}

static void test_bus_errors_propagate(void)
{
    mpu6050_raw_t raw;
    setup(&default_cfg);
    mock.fail_read_reg = MPU6050_REG_ACCEL_XOUT_H + 9;  /* inside the 14-byte burst */
    CHECK_EQ_INT(mpu6050_read_raw(&dev, &raw), MPU6050_ERR_BUS);

    mock_init(&mock, MPU6050_ADDR_AD0_LOW);
    mock.fail_write_reg = MPU6050_REG_ACCEL_CONFIG;
    hal = mock_hal(&mock);
    CHECK_EQ_INT(mpu6050_init(&dev, &hal, MPU6050_ADDR_AD0_LOW, &default_cfg), MPU6050_ERR_BUS);
    CHECK(!dev.initialized);
}

static void test_write_verification_catches_stuck_register(void)
{
    mock_init(&mock, MPU6050_ADDR_AD0_LOW);
    mock.stuck_reg = MPU6050_REG_GYRO_CONFIG;  /* write ACKed but not applied */
    hal = mock_hal(&mock);
    CHECK_EQ_INT(mpu6050_init(&dev, &hal, MPU6050_ADDR_AD0_LOW, &default_cfg), MPU6050_ERR_BUS);
}

static void test_burst_read_big_endian_signed(void)
{
    const int16_t a[3] = { 16384, -16384, 32767 };
    const int16_t g[3] = { -1, -32768, 655 };
    mpu6050_raw_t raw;
    setup(&default_cfg);
    mock_set_sample(&mock, a, -521, g);
    CHECK_EQ_INT(mpu6050_read_raw(&dev, &raw), MPU6050_OK);
    CHECK_EQ_INT(raw.accel[0], 16384);
    CHECK_EQ_INT(raw.accel[1], -16384);
    CHECK_EQ_INT(raw.accel[2], 32767);
    CHECK_EQ_INT(raw.temp, -521);
    CHECK_EQ_INT(raw.gyro[0], -1);
    CHECK_EQ_INT(raw.gyro[1], -32768);
    CHECK_EQ_INT(raw.gyro[2], 655);
}

static void test_conversion_to_si_units(void)
{
    const int16_t a[3] = { 8192, 0, -8192 };  /* +/-1 g at +/-4 g range */
    const int16_t g[3] = { 655, -655, 0 };    /* +/-10 dps at 500 dps range */
    mpu6050_data_t d;
    setup(&default_cfg);
    mock_set_sample(&mock, a, 0, g);
    CHECK_EQ_INT(mpu6050_read(&dev, &d), MPU6050_OK);
    CHECK_NEAR(d.accel_mps2[0], 9.80665, 1e-4);
    CHECK_NEAR(d.accel_mps2[1], 0.0, 1e-6);
    CHECK_NEAR(d.accel_mps2[2], -9.80665, 1e-4);
    CHECK_NEAR(d.gyro_rads[0], 10.0 * M_PI / 180.0, 1e-5);
    CHECK_NEAR(d.gyro_rads[1], -10.0 * M_PI / 180.0, 1e-5);
    CHECK_NEAR(d.temp_c, 36.53, 1e-4);
}

static void test_every_range_scales_full_scale_correctly(void)
{
    static const float g_full[] = { 2.0f, 4.0f, 8.0f, 16.0f };
    static const float dps_full[] = { 250.0f, 500.0f, 1000.0f, 2000.0f };
    const int16_t a[3] = { 16384, 0, 0 };  /* half of full scale */
    const int16_t g[3] = { 16384, 0, 0 };
    mpu6050_config_t cfg = MPU6050_CONFIG_DEFAULT;
    mpu6050_data_t d;
    int r;
    for (r = 0; r < 4; ++r) {
        cfg.accel_range = (mpu6050_accel_range_t)r;
        cfg.gyro_range = (mpu6050_gyro_range_t)r;
        CHECK_EQ_INT(setup(&cfg), MPU6050_OK);
        CHECK_EQ_INT(mock.regs[MPU6050_REG_ACCEL_CONFIG], r << 3);
        mock_set_sample(&mock, a, 0, g);
        mpu6050_read(&dev, &d);
        CHECK_NEAR(d.accel_mps2[0], g_full[r] / 2.0 * 9.80665, 1e-3);
        /* gyro LSB/dps values are rounded in the datasheet: allow 0.5 % */
        CHECK_NEAR(d.gyro_rads[0], dps_full[r] / 2.0 * M_PI / 180.0, dps_full[r] / 2.0 * M_PI / 180.0 * 5e-3);
    }
}

static void test_sample_rate_divider(void)
{
    mpu6050_config_t cfg = MPU6050_CONFIG_DEFAULT;

    cfg.sample_rate_hz = 1000;
    CHECK_EQ_INT(setup(&cfg), MPU6050_OK);
    CHECK_EQ_INT(mock.regs[MPU6050_REG_SMPLRT_DIV], 0);

    cfg.dlpf = MPU6050_DLPF_260HZ;  /* DLPF off: gyro runs at 8 kHz */
    CHECK_EQ_INT(setup(&cfg), MPU6050_OK);
    CHECK_EQ_INT(mock.regs[MPU6050_REG_SMPLRT_DIV], 7);

    cfg.dlpf = MPU6050_DLPF_44HZ;
    cfg.sample_rate_hz = 300;  /* not exact: 1000/3 = 333.3 Hz is nearest */
    CHECK_NEAR(mpu6050_actual_rate_hz(&cfg), 333.333, 1e-2);

    cfg.sample_rate_hz = 0;
    CHECK_EQ_INT(setup(&cfg), MPU6050_ERR_PARAM);
    cfg.sample_rate_hz = 1001;  /* above 1 kHz with DLPF on */
    CHECK_EQ_INT(setup(&cfg), MPU6050_ERR_PARAM);
    cfg.sample_rate_hz = 3;     /* would need divider > 255 */
    CHECK_EQ_INT(setup(&cfg), MPU6050_ERR_PARAM);
}

static void test_invalid_enums_rejected(void)
{
    mpu6050_config_t cfg = MPU6050_CONFIG_DEFAULT;
    cfg.gyro_range = (mpu6050_gyro_range_t)4;
    CHECK_EQ_INT(setup(&cfg), MPU6050_ERR_PARAM);
    cfg = default_cfg;
    cfg.dlpf = (mpu6050_dlpf_t)7;
    CHECK_EQ_INT(setup(&cfg), MPU6050_ERR_PARAM);
}

static void test_reconfigure_at_runtime(void)
{
    mpu6050_config_t cfg = MPU6050_CONFIG_DEFAULT;
    setup(&default_cfg);
    cfg.gyro_range = MPU6050_GYRO_2000DPS;
    cfg.data_ready_irq = true;
    CHECK_EQ_INT(mpu6050_configure(&dev, &cfg), MPU6050_OK);
    CHECK_EQ_INT(mock.regs[MPU6050_REG_GYRO_CONFIG], 0x18);
    CHECK_EQ_INT(mock.regs[MPU6050_REG_INT_ENABLE], MPU6050_INT_DATA_RDY);
    CHECK_NEAR(dev.gyro_lsb_per_dps, 16.4, 1e-6);

    /* a rejected config leaves the previous one in force */
    cfg.sample_rate_hz = 0;
    CHECK_EQ_INT(mpu6050_configure(&dev, &cfg), MPU6050_ERR_PARAM);
    CHECK_EQ_INT(dev.cfg.gyro_range, MPU6050_GYRO_2000DPS);
}

static void test_data_ready_flag_clears_on_read(void)
{
    bool ready = false;
    setup(&default_cfg);
    mock.data_ready = true;
    CHECK_EQ_INT(mpu6050_data_ready(&dev, &ready), MPU6050_OK);
    CHECK(ready);
    CHECK_EQ_INT(mpu6050_data_ready(&dev, &ready), MPU6050_OK);
    CHECK(!ready);
}

static void test_gyro_calibration_removes_offset(void)
{
    const int16_t a[3] = { 0, 0, 8192 };
    const int16_t g[3] = { 120, -45, 7 };
    mpu6050_data_t d;
    uint32_t delay_before;
    setup(&default_cfg);
    mock_set_sample(&mock, a, 0, g);
    delay_before = mock.total_delay_ms;
    CHECK_EQ_INT(mpu6050_calibrate_gyro(&dev, 50), MPU6050_OK);
    CHECK_EQ_INT(mock.total_delay_ms - delay_before, 50 * 5);  /* one 5 ms period per sample at 200 Hz */
    CHECK_NEAR(dev.gyro_offset_raw[0], 120.0, 1e-6);
    CHECK_NEAR(dev.gyro_offset_raw[1], -45.0, 1e-6);
    mpu6050_read(&dev, &d);
    CHECK_NEAR(d.gyro_rads[0], 0.0, 1e-7);
    CHECK_NEAR(d.gyro_rads[1], 0.0, 1e-7);
    CHECK_NEAR(d.gyro_rads[2], 0.0, 1e-7);
    CHECK_EQ_INT(mpu6050_calibrate_gyro(&dev, 0), MPU6050_ERR_PARAM);
}

static void test_status_strings(void)
{
    CHECK(strcmp(mpu6050_status_str(MPU6050_OK), "ok") == 0);
    CHECK(strcmp(mpu6050_status_str((mpu6050_status_t)99), "unknown") == 0);
}

int main(void)
{
    RUN(test_init_programs_registers);
    RUN(test_init_resets_before_configuring);
    RUN(test_init_rejects_wrong_device);
    RUN(test_init_wrong_address_is_bus_error);
    RUN(test_null_arguments);
    RUN(test_use_before_init);
    RUN(test_bus_errors_propagate);
    RUN(test_write_verification_catches_stuck_register);
    RUN(test_burst_read_big_endian_signed);
    RUN(test_conversion_to_si_units);
    RUN(test_every_range_scales_full_scale_correctly);
    RUN(test_sample_rate_divider);
    RUN(test_invalid_enums_rejected);
    RUN(test_reconfigure_at_runtime);
    RUN(test_data_ready_flag_clears_on_read);
    RUN(test_gyro_calibration_removes_offset);
    RUN(test_status_strings);
    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
