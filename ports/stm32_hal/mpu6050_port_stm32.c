/* STM32 HAL port (any STM32 family with the Cube HAL I2C driver).
 *
 * Usage:
 *     mpu6050_hal_t hal;
 *     mpu6050_stm32_hal(&hal, &hi2c1);
 *     mpu6050_init(&imu, &hal, MPU6050_ADDR_AD0_LOW, &cfg);
 *
 * Replace the include with your family's header (stm32f4xx_hal.h, stm32g4xx_hal.h, ...).
 */
#include "stm32f4xx_hal.h"
#include "mpu6050.h"

#define I2C_TIMEOUT_MS 10u

static int stm32_read(void *ctx, uint8_t dev_addr, uint8_t reg, uint8_t *buf, size_t len)
{
    /* HAL expects the 8-bit (left-shifted) address. Mem_Read issues a repeated start. */
    return HAL_I2C_Mem_Read((I2C_HandleTypeDef *)ctx, (uint16_t)(dev_addr << 1), reg,
                            I2C_MEMADD_SIZE_8BIT, buf, (uint16_t)len, I2C_TIMEOUT_MS) == HAL_OK ? 0 : -1;
}

static int stm32_write(void *ctx, uint8_t dev_addr, uint8_t reg, const uint8_t *buf, size_t len)
{
    return HAL_I2C_Mem_Write((I2C_HandleTypeDef *)ctx, (uint16_t)(dev_addr << 1), reg,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf, (uint16_t)len, I2C_TIMEOUT_MS) == HAL_OK ? 0 : -1;
}

static void stm32_delay_ms(void *ctx, uint32_t ms)
{
    (void)ctx;
    HAL_Delay(ms);
}

void mpu6050_stm32_hal(mpu6050_hal_t *hal, I2C_HandleTypeDef *hi2c)
{
    hal->read = stm32_read;
    hal->write = stm32_write;
    hal->delay_ms = stm32_delay_ms;
    hal->ctx = hi2c;
}
