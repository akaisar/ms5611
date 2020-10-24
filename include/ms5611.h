/*
 * HTU21D Component
 *
 * esp-idf component to interface with HTU21D humidity and temperature sensor
 * by TE Connectivity (http://www.te.com/usa-en/product-CAT-HSC0004.html)
 *
 * Luca Dentella, www.lucadentella.it
 */

#ifndef __ESP_MS5611_H__
#define __ESP_MS5611_H__

#ifdef __cplusplus
extern "C" {
#endif

 // Error library
#include "esp_err.h"

// I2C driver
#include "driver/i2c.h"

// FreeRTOS (for delay)
#include "freertos/task.h"

// sensor address
#define MS5611_ADDRESS                0x77

#define MS5611_CMD_ADC_READ           0x00
#define MS5611_CMD_RESET              0x1E
#define MS5611_CMD_CONV_D1            0x48
#define MS5611_CMD_CONV_D2            0x58
#define MS5611_CMD_PROM_READ          0xA0

i2c_port_t _port;

int ms5611_init(i2c_port_t port, int sda_pin, int scl_pin, gpio_pullup_t sda_internal_pullup, gpio_pullup_t scl_internal_pullup);
bool ms5611_update(uint16_t calibration[6], long double *temperature, long double *dT, long double *pressure, long double *attitute);
void ms5611_begin(uint16_t calibration[6], long double *temperature, long double *dT, long double *pressure);
#ifdef __cplusplus
}
#endif

#endif  // __ESP_SI7021_H__
