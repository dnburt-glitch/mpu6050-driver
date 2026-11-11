/* ESP-IDF v5.2+ port using the i2c_master driver.
 *
 * The device handle already carries the 7-bit address, so dev_addr is unused:
 *
 *     i2c_device_config_t dev_cfg = {
 *         .dev_addr_length = I2C_ADDR_BIT_LEN_7,
 *         .device_address  = MPU6050_ADDR_AD0_LOW,
 *         .scl_speed_hz    = 400000,
 *     };
 *     i2c_master_dev_handle_t dev;
 *     ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &dev));
 *     mpu6050_esp_idf_hal(&hal, dev);
 */
#include <string.h>

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"

#define I2C_TIMEOUT_MS 20
#define MAX_WRITE      16u

static int esp_read(void *ctx, uint8_t dev_addr, uint8_t reg, uint8_t *buf, size_t len)
{
    (void)dev_addr;
    return i2c_master_transmit_receive((i2c_master_dev_handle_t)ctx, &reg, 1, buf, len,
                                       I2C_TIMEOUT_MS) == ESP_OK ? 0 : -1;
}

static int esp_write(void *ctx, uint8_t dev_addr, uint8_t reg, const uint8_t *buf, size_t len)
{
    uint8_t frame[1u + MAX_WRITE];
    (void)dev_addr;
    if (len > MAX_WRITE) {
        return -1;
    }
    frame[0] = reg;
    memcpy(&frame[1], buf, len);
    return i2c_master_transmit((i2c_master_dev_handle_t)ctx, frame, len + 1u,
                               I2C_TIMEOUT_MS) == ESP_OK ? 0 : -1;
}

static void esp_delay_ms(void *ctx, uint32_t ms)
{
    (void)ctx;
    /* Round up so short delays never become zero ticks. */
    vTaskDelay((ms + portTICK_PERIOD_MS - 1u) / portTICK_PERIOD_MS);
}

void mpu6050_esp_idf_hal(mpu6050_hal_t *hal, i2c_master_dev_handle_t dev)
{
    hal->read = esp_read;
    hal->write = esp_write;
    hal->delay_ms = esp_delay_ms;
    hal->ctx = dev;
}
