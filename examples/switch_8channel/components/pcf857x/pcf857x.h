/**
 * @file pcf8574.h
 *
 * ESP-IDF driver for PCF8574 compartible remote 8-bit I/O expanders for I2C-bus
 *
 * Copyright (C) nikwest (https://github.com/nikwest) inspired by (https://github.com/UncleRus)
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __PCF857x_H__
#define __PCF857x_H__

#include <stddef.h>
#include <i2cdev.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PCF8574=1,
    PCF8575=2
} pcf857x_type_t;

 typedef  void (*pcf857x_callback_t)(uint16_t value);

typedef struct {
    i2c_dev_t i2c;
    pcf857x_type_t type;
    pcf857x_callback_t cb;
} pcf857x_t;


/**
 * @brief Initialize device descriptior
 * SCL frequency is 100kHz
 * @param dev Pointer to I2C device descriptor
 * @param type PCF8574 or PCF8575
 * @param port I2C port number
 * @param addr I2C address (0b0100<A2><A1><A0> for PCF8574)
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t pcf857x_init(pcf857x_t *dev, pcf857x_type_t type, i2c_port_t port, uint8_t addr, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 * @param dev Pointer to I2C device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t pcf857x_free(pcf857x_t *dev);

/**
 * @brief Interrupt callback
 */
esp_err_t pcf857x_set_interrupt_handler(pcf857x_t *dev, gpio_num_t irq_gpio, pcf857x_callback_t handler);

/**
 * @brief Read GPIO port value
 * @param dev Pointer to I2C device descriptor
 * @param value Pointer to read value
 * @return `ESP_OK` on success
 */
esp_err_t pcf857x_read(const pcf857x_t *dev, uint16_t *value);

/**
 * @brief Write value to GPIO port
 * @param dev Pointer to I2C device descriptor
 * @param value to be written
 * @return ESP_OK on success
 */
esp_err_t pcf857x_write(const pcf857x_t *dev, uint16_t value);

#ifdef __cplusplus
}
#endif

#endif /* __PCF857x_H__ */
