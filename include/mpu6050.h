/**
 * @file mpu6050.h
 * @brief Portable, platform-independent driver for the InvenSense MPU-6050
 *        6-axis IMU (3-axis gyroscope + 3-axis accelerometer) over I2C.
 *
 * The driver has no dependency on any vendor HAL. The application supplies
 * three callbacks (I2C register read, I2C register write, millisecond delay)
 * through mpu6050_hal_t, so the same code runs on STM32, ESP32, nRF, Linux
 * i2c-dev or a unit-test mock.
 *
 * No dynamic memory, no global state: one mpu6050_t per sensor.
 */
#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- I2C addresses ------------------------------------------------------ */
#define MPU6050_ADDR_AD0_LOW   0x68u
#define MPU6050_ADDR_AD0_HIGH  0x69u

/* ---- Register map (subset used by this driver) -------------------------- */
#define MPU6050_REG_SMPLRT_DIV    0x19u
#define MPU6050_REG_CONFIG        0x1Au
#define MPU6050_REG_GYRO_CONFIG   0x1Bu
#define MPU6050_REG_ACCEL_CONFIG  0x1Cu
#define MPU6050_REG_INT_PIN_CFG   0x37u
#define MPU6050_REG_INT_ENABLE    0x38u
#define MPU6050_REG_INT_STATUS    0x3Au
#define MPU6050_REG_ACCEL_XOUT_H  0x3Bu  /* 14-byte burst: accel, temp, gyro */
#define MPU6050_REG_PWR_MGMT_1    0x6Bu
#define MPU6050_REG_WHO_AM_I      0x75u

#define MPU6050_WHO_AM_I_VALUE    0x68u
#define MPU6050_PWR1_DEVICE_RESET 0x80u
#define MPU6050_PWR1_SLEEP        0x40u
#define MPU6050_CLKSEL_PLL_XGYRO  0x01u
#define MPU6050_INT_DATA_RDY      0x01u

/* ---- Error codes -------------------------------------------------------- */
typedef enum {
    MPU6050_OK = 0,
    MPU6050_ERR_NULL,        /* NULL pointer argument or missing HAL callback */
    MPU6050_ERR_BUS,         /* I2C transaction failed */
    MPU6050_ERR_WHO_AM_I,    /* device did not identify as an MPU-6050 */
    MPU6050_ERR_PARAM,       /* configuration value out of range */
    MPU6050_ERR_NOT_READY    /* driver used before successful mpu6050_init() */
} mpu6050_status_t;

/* ---- Configuration enums (values match the register fields) ------------- */
typedef enum {
    MPU6050_ACCEL_2G = 0, MPU6050_ACCEL_4G, MPU6050_ACCEL_8G, MPU6050_ACCEL_16G
} mpu6050_accel_range_t;

typedef enum {
    MPU6050_GYRO_250DPS = 0, MPU6050_GYRO_500DPS, MPU6050_GYRO_1000DPS, MPU6050_GYRO_2000DPS
} mpu6050_gyro_range_t;

/** Digital low-pass filter bandwidth (accel / gyro, approximate). */
typedef enum {
    MPU6050_DLPF_260HZ = 0,  /* disables the DLPF: gyro output rate 8 kHz */
    MPU6050_DLPF_184HZ, MPU6050_DLPF_94HZ, MPU6050_DLPF_44HZ,
    MPU6050_DLPF_21HZ, MPU6050_DLPF_10HZ, MPU6050_DLPF_5HZ
} mpu6050_dlpf_t;

/* ---- Platform abstraction ----------------------------------------------- */
typedef struct {
    /** Read len bytes starting at reg (auto-increment). Return 0 on success. */
    int (*read)(void *ctx, uint8_t dev_addr, uint8_t reg, uint8_t *buf, size_t len);
    /** Write len bytes starting at reg. Return 0 on success. */
    int (*write)(void *ctx, uint8_t dev_addr, uint8_t reg, const uint8_t *buf, size_t len);
    /** Block for at least ms milliseconds. */
    void (*delay_ms)(void *ctx, uint32_t ms);
    /** Opaque pointer passed to every callback (bus handle, mock, ...). */
    void *ctx;
} mpu6050_hal_t;

typedef struct {
    mpu6050_accel_range_t accel_range;
    mpu6050_gyro_range_t  gyro_range;
    mpu6050_dlpf_t        dlpf;
    uint16_t              sample_rate_hz;   /* requested output data rate */
    bool                  data_ready_irq;   /* enable data-ready interrupt (INT pin + INT_STATUS flag) */
} mpu6050_config_t;

/** Sensible defaults: +/-4 g, +/-500 dps, 44 Hz DLPF, 200 Hz, no interrupt. */
#define MPU6050_CONFIG_DEFAULT { MPU6050_ACCEL_4G, MPU6050_GYRO_500DPS, \
                                 MPU6050_DLPF_44HZ, 200u, false }

typedef struct {
    int16_t accel[3];
    int16_t temp;
    int16_t gyro[3];
} mpu6050_raw_t;

typedef struct {
    float accel_mps2[3];  /* m/s^2 */
    float gyro_rads[3];   /* rad/s, gyro offset removed */
    float temp_c;         /* deg C */
} mpu6050_data_t;

typedef struct {
    mpu6050_hal_t    hal;
    uint8_t          addr;
    mpu6050_config_t cfg;
    float            accel_lsb_per_g;
    float            gyro_lsb_per_dps;
    float            gyro_offset_raw[3];  /* from mpu6050_calibrate_gyro() */
    bool             initialized;
} mpu6050_t;

/* ---- API ---------------------------------------------------------------- */

/** Reset the device, verify WHO_AM_I, wake it on the gyro PLL clock and apply cfg. */
mpu6050_status_t mpu6050_init(mpu6050_t *dev, const mpu6050_hal_t *hal,
                              uint8_t addr, const mpu6050_config_t *cfg);

/** Change ranges / filter / rate on an initialised device. */
mpu6050_status_t mpu6050_configure(mpu6050_t *dev, const mpu6050_config_t *cfg);

/** One 14-byte burst read: accel, temperature and gyro from the same sample. */
mpu6050_status_t mpu6050_read_raw(mpu6050_t *dev, mpu6050_raw_t *raw);

/** Read and convert to SI units. */
mpu6050_status_t mpu6050_read(mpu6050_t *dev, mpu6050_data_t *out);

/** Non-blocking: *ready = true when a new sample is available (clears the flag).
 *  Requires cfg.data_ready_irq = true: the chip only sets DATA_RDY_INT when the
 *  data-ready interrupt is enabled. The INT pin can stay unconnected. */
mpu6050_status_t mpu6050_data_ready(mpu6050_t *dev, bool *ready);

/** Average n_samples readings with the sensor at rest and store the gyro offset. */
mpu6050_status_t mpu6050_calibrate_gyro(mpu6050_t *dev, uint16_t n_samples);

/** Convert raw counts to SI units using the device's current scale and offsets. */
void mpu6050_convert(const mpu6050_t *dev, const mpu6050_raw_t *raw, mpu6050_data_t *out);

/** Actual output rate for a config (the divider is an integer, so it may differ). */
float mpu6050_actual_rate_hz(const mpu6050_config_t *cfg);

const char *mpu6050_status_str(mpu6050_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
