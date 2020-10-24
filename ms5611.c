
// Component header file

#include "ms5611.h"
#include "math.h"

int ms5611_init(i2c_port_t port, int sda_pin, int scl_pin,  gpio_pullup_t sda_internal_pullup,  gpio_pullup_t scl_internal_pullup) {

	esp_err_t ret;
	_port = port;

	// setup i2c controller
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = sda_pin;
	conf.scl_io_num = scl_pin;
	conf.sda_pullup_en = sda_internal_pullup;
	conf.scl_pullup_en = scl_internal_pullup;
	conf.master.clk_speed = 100000;
	ret = i2c_param_config(port, &conf);

	// install the driver
	ret = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);

	// verify if a sensor is present
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (MS5611_ADDRESS << 1) | I2C_MASTER_WRITE, true);
	i2c_master_stop(cmd);

	return 0;
}

void ms5611_sendCommand(uint8_t reg) {
    esp_err_t ret;
    uint16_t value;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (MS5611_ADDRESS << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg, true);
	i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(_port, cmd, 10000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	 if(ret != ESP_OK)
         return 0;
	vTaskDelay(50 / portTICK_RATE_MS);
}

uint16_t ms5611_readPROM(uint16_t address) {
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ms5611_sendCommand(MS5611_CMD_PROM_READ | address);
    uint8_t vha, vla, vxa;
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (MS5611_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &vha, 0x00);
    i2c_master_read_byte(cmd, &vla, 0x01);
    i2c_master_read_byte(cmd, &vxa, 0x00);
	i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(_port, cmd, 10000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
    uint16_t value = vha << 8 | vla;
    return value;
}

uint32_t ms5611_readADC() {
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ms5611_sendCommand(MS5611_CMD_ADC_READ);
    uint8_t vxa, vha, vla;
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (MS5611_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &vxa, 0x00);
    i2c_master_read_byte(cmd, &vha, 0x00);
    i2c_master_read_byte(cmd, &vla, 0x00);
	i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(_port, cmd, 10000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	 if(ret != ESP_OK)
         return 0;
    uint32_t value = ((uint32_t)vxa << 16) | ((uint32_t)vha << 8) | (uint32_t)vla;
    return value;
}

void ms5611_reset() {
    ms5611_sendCommand(MS5611_CMD_RESET);
}

void ms5611_read_calibration(uint16_t calibration[6]){
	int C[30];
    for (uint8_t i = 0; i < 8; i++) {
		C[i] = ms5611_readPROM(i*2);
	}
	for (uint8_t i = 0; i < 6; i++) {
		calibration[i] = C[i+1];
	}
}

void ms5611_begin(uint16_t calibration[6], long double *temperature, long double *dT, long double *pressure) {
    *temperature = 0;
	*dT = 0;
	*pressure = 0;
    ms5611_init(I2C_NUM_0, 21, 22,
    	GPIO_PULLUP_DISABLE,GPIO_PULLUP_DISABLE);
    ms5611_reset();
    ms5611_read_calibration(calibration);
}

uint32_t ms5611_getRawPressure() {
	ms5611_sendCommand(MS5611_CMD_CONV_D1);
	return ms5611_readADC();
}

uint32_t ms5611_getRawTemperature() {
	ms5611_sendCommand(MS5611_CMD_CONV_D2);
	return ms5611_readADC();
}


// Calculate altitude from Pressure & Temperature with barometric formula
long double ms5611_altitude(long double P, long double T){
	long double R = 8.31446261815324;
	long double dP = P / 927.030406;
	long double g = 9.80665;
	long double M = 0.02896;
	long double altitude = log(dP) * R * (T + 273.15) / M / g;
    return -altitude;
}

// Calculate altitude from Pressure & Temperature with barometric step
long double ms5611_altitude_step(long double P, long double T){
	long double seaP = 1013.25;
	long double alpha = 1.0 / 273;
	long double altitude = 8000 * 2 * (seaP - P) / (seaP + P) * (1 + alpha * T);
    return altitude;
}

bool ms5611_update(uint16_t calibration[6], long double *temperature, long double *dT, long double *pressure, long double *attitute) {
	uint32_t D2 = ms5611_getRawTemperature();
	*dT = D2 * 1.0 - ((long double)calibration[4] * 256);
	*temperature = (2000.0 + ((long double) (*dT) * calibration[5]) / 8388608) / 100;
	uint32_t D1 = ms5611_getRawPressure();
	long double OFF = (long double) calibration[1] * 65536 + (long double) calibration[3] * (*dT) / 128;
	long double SENS = (long double) calibration[0] * 32768 + (long double) calibration[2] * (*dT) / 256;
	*pressure = (D1 * SENS / 2097152.0 - OFF) / 32768.0 / 100;
	*attitute = ms5611_altitude(*pressure, *temperature);
	return true;
}
