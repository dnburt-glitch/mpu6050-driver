# MPU-6050 Driver: Portable C, Unit-Tested Without Hardware

A production-style C99 driver for the InvenSense MPU-6050 6-axis IMU. It has no vendor dependencies, does no dynamic allocation and keeps no global state. The same code runs on STM32, ESP32 and Linux boards through a three-function platform layer. It is verified by 90 host-side checks against a register-level emulator of the chip.

## Features

- **Platform-independent core**: the application supplies `read`, `write` and `delay_ms` callbacks. Ports are included for STM32 HAL, ESP-IDF v5 (`i2c_master`) and Linux `i2c-dev` (Raspberry Pi, Jetson).
- **Safe initialisation**: verifies `WHO_AM_I` before writing anything, performs a full device reset, and wakes the chip on the gyro PLL clock.
- **Write-and-verify** for every configuration register, so a bus glitch or a marginal pull-up shows up as an error instead of a silently misconfigured sensor.
- **Coherent sampling**: one 14-byte burst read, so accelerometer, temperature and gyro values all come from the same sample.
- **Rate handling**: computes the sample-rate divider for the DLPF-on (1 kHz) and DLPF-off (8 kHz) cases, picks the nearest achievable rate, and rejects rates the hardware cannot produce.
- **SI units out**: m/s², rad/s, °C, plus gyro bias calibration.
- **Defensive API**: explicit status codes, NULL checks, use-before-init detection; a rejected reconfiguration leaves the previous settings in force.

## Usage

```c
#include "mpu6050.h"

mpu6050_t imu;
mpu6050_hal_t hal = { my_i2c_read, my_i2c_write, my_delay_ms, &my_bus };
mpu6050_config_t cfg = MPU6050_CONFIG_DEFAULT;   /* ±4 g, ±500 dps, 44 Hz DLPF, 200 Hz */

if (mpu6050_init(&imu, &hal, MPU6050_ADDR_AD0_LOW, &cfg) != MPU6050_OK) { /* handle */ }
mpu6050_calibrate_gyro(&imu, 200);               /* keep the board still */

mpu6050_data_t d;
mpu6050_read(&imu, &d);                          /* d.accel_mps2, d.gyro_rads, d.temp_c */
```

On STM32: `mpu6050_stm32_hal(&hal, &hi2c1);`. On ESP-IDF: `mpu6050_esp_idf_hal(&hal, dev_handle);`.

## Testing without hardware

`tests/mock_i2c.c` emulates the chip at register level: power-on defaults, self-clearing reset, read-only `WHO_AM_I`, auto-increment burst reads and clear-on-read `INT_STATUS`. It also injects faults: NACK on a wrong address, failing reads or writes on a chosen register, and a "stuck" register that ACKs writes but ignores them.

```bash
make test             # gcc, -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror
make test CC=clang
make test-sanitize    # AddressSanitizer + UndefinedBehaviorSanitizer
make linux-example    # build the Raspberry Pi / Linux example
```

```
17 tests, 90 checks, 0 failures
```

Covered:

- register programming and init order (reset first, ≥100 ms reset delay)
- a wrong chip is rejected with zero writes
- bus-error propagation, including a failure in the middle of a burst read
- write-verify catches a stuck register
- big-endian signed decoding at the int16 limits
- SI conversion for all four accelerometer and gyro ranges
- the sample-rate divider in both DLPF modes and its edge cases
- invalid enums
- runtime reconfiguration and rollback
- the data-ready flag
- calibration timing and offset removal

Static analysis: `clang-tidy` with `bugprone-*`, `cert-*` and `clang-analyzer-*` reports no findings.

## Layout

```
include/mpu6050.h              public API and register map
src/mpu6050.c                  driver
ports/linux/                   i2c-dev port (I2C_RDWR with repeated start), compiled in CI
ports/stm32_hal/               STM32 Cube HAL port
ports/esp_idf/                 ESP-IDF v5.2+ i2c_master port
examples/linux_main.c          calibrate and stream samples on a Linux board
tests/                         register-level mock and unit tests
```

> The STM32 and ESP-IDF ports are thin reference adapters around the vendor I2C calls. They are not built in CI because they need the vendor SDKs.

## License

MIT
