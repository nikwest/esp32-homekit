/**
 * @file pcf8574.c
 *
 * ESP-IDF driver for PCF8574/PCF8575 compartible remote 8-bit I/O expanders for I2C-bus
 *
 * Copyright (C) nikwest (https://github.com/nikwest) inspired by (https://github.com/UncleRus)
 * MIT Licensed as described in the file LICENSE
 */
#include "pcf857x.h"
#include <esp_err.h>
#include "esp_log.h"

#define TAG "PCF857x"

#define I2C_FREQ_HZ 100000
#define ESP_INTR_FLAG_DEFAULT 0

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)


static xQueueHandle pcf857x_read_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    xQueueSendFromISR(pcf857x_read_queue, &arg, NULL);
}

static void pcf857x_task_read(void* arg) {
    pcf857x_t *dev;
    uint16_t val;
    while(1) {
        if(xQueueReceive(pcf857x_read_queue, &dev, portMAX_DELAY)) {
          pcf857x_read(dev, &val);
          if(dev->cb == NULL) {
              ESP_LOGE(TAG, "Interrupt handler is NULL!");
              continue;
           }
          ESP_LOGI(TAG, "calling interrupt handler with %d", val);
          dev->cb(val);
        } else {
          ESP_LOGE(TAG, "Failed to read from queue!");
        }
    }
}

esp_err_t pcf857x_init(pcf857x_t *dev, pcf857x_type_t type, i2c_port_t port, uint8_t addr, gpio_num_t sda_gpio, gpio_num_t scl_gpio) {
    CHECK_ARG(dev);
    CHECK_ARG(addr & 0x20);

    dev->type = type;

    dev->i2c.port = port;
    dev->i2c.addr = addr;
    dev->i2c.cfg.sda_io_num = sda_gpio;
    dev->i2c.cfg.scl_io_num = scl_gpio;
    dev->i2c.cfg.master.clk_speed = I2C_FREQ_HZ;

    dev->cb = NULL;

    CHECK(i2c_dev_create_mutex(&(dev->i2c)));

    return ESP_OK;
}

esp_err_t pcf857x_free(pcf857x_t *dev) {
    CHECK_ARG(dev);
    return i2c_dev_delete_mutex(&(dev->i2c));
}

esp_err_t pcf857x_set_interrupt_handler(pcf857x_t *dev, gpio_num_t irq_gpio, pcf857x_callback_t handler) {
    gpio_config_t io_conf;
    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL<<irq_gpio);
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
      //create a queue to handle gpio event from isr
    pcf857x_read_queue = xQueueCreate(10, sizeof(void*));
    if(pcf857x_read_queue == NULL) {
      ESP_LOGE(TAG, "Failed to create read queue!");
      return ESP_FAIL;
    }
    //start gpio task
    xTaskCreate(pcf857x_task_read, "pcf857x_read_task", 2048, NULL, 10, NULL);
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(irq_gpio, gpio_isr_handler, (void*) dev);
  
    dev->cb = handler;

    return ESP_OK;
}

esp_err_t pcf857x_read(const pcf857x_t *dev, uint16_t *value) {
    CHECK_ARG(dev);
    CHECK_ARG(value);
    return i2c_dev_read(&(dev->i2c), NULL, 0, value, (size_t) dev->type);
}

/**
 * @brief Write value to GPIO port
 * @param dev Pointer to I2C device descriptor
 * @return ESP_OK on success
 */
esp_err_t pcf857x_write(const pcf857x_t *dev, uint16_t value) {
    CHECK_ARG(dev);
    return i2c_dev_write(&(dev->i2c), NULL, 0, &value, (size_t) dev->type);
}