#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "esp_log.h"
#include "SCI.h"
#include "at24c.h"

#define CONFIG_24C512
//#define TAG "at24c"

#ifdef CONFIG_24C02
#define EEPROM_SIZE         2
#endif

#ifdef CONFIG_24C04
#define EEPROM_SIZE         4
#endif

#ifdef CONFIG_24C08
#define EEPROM_SIZE         8
#endif

#ifdef CONFIG_24C16
#define EEPROM_SIZE         16
#endif

#ifdef CONFIG_24C32
#define EEPROM_SIZE         32
#endif

#ifdef CONFIG_24C64
#define EEPROM_SIZE         64
#endif

#ifdef CONFIG_24C128
#define EEPROM_SIZE         128
#endif

#ifdef CONFIG_24C256
#define EEPROM_SIZE         256
#endif

#ifdef CONFIG_24C512
#define EEPROM_SIZE         512
#endif

#define EEPROM_I2C_ADDRESS	0x50

esp_err_t InitRom(EEPROM_t * dev, i2c_port_t i2c_port)
{
	// ESP_LOGI(TAG, "EEPROM is 24C%.02d",EEPROM_SIZE);
    // ESP_LOGI(TAG, "CONFIG_SDA_GPIO=%d",CONFIG_SDA_GPIO);
    // ESP_LOGI(TAG, "CONFIG_SCL_GPIO=%d",CONFIG_SCL_GPIO);
    // ESP_LOGI(TAG, "EEPROM_I2C_ADDRESS=0x%x",EEPROM_I2C_ADDRESS);

	dev->_i2c_port = i2c_port;
	dev->_chip_addr = EEPROM_I2C_ADDRESS;
	dev->_kbits = EEPROM_SIZE;
	dev->_bytes = 128 * EEPROM_SIZE;

	esp_err_t ret;
	// i2c_config_t conf = {
    //     .mode = I2C_MODE_MASTER,
    //     .sda_io_num = CONFIG_SDA_GPIO,
    //     .sda_pullup_en = GPIO_PULLUP_ENABLE,
    //     .scl_io_num = CONFIG_SCL_GPIO,
    //     .scl_pullup_en = GPIO_PULLUP_ENABLE,
    //     .master.clk_speed = I2C_FREQUENCY
	// };
	// ret = i2c_param_config(i2c_port, &conf);
	// ESP_LOGD(TAG, "i2c_param_config=%d", ret);
	// if (ret != ESP_OK) return ret;
	// ret = i2c_driver_install(i2c_port, I2C_MODE_MASTER, 0, 0, 0);
	// ESP_LOGD(TAG, "i2c_driver_install=%d", ret);
	ret = ESP_OK;
	return ret;
}


static esp_err_t ReadReg8(EEPROM_t * dev, i2c_port_t i2c_port, int chip_addr, uint8_t data_addr, uint8_t * data)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, chip_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, data_addr, ACK_CHECK_EN);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, chip_addr << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
	i2c_master_read_byte(cmd, data, NACK_VAL);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	return ret;
}

static esp_err_t WriteReg8(EEPROM_t * dev, i2c_port_t i2c_port, int chip_addr, uint8_t data_addr, uint8_t data)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, chip_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, data_addr, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	usleep(1000*2);
	return ret;
}

static esp_err_t ReadReg16(EEPROM_t * dev, i2c_port_t i2c_port, int chip_addr, uint16_t data_addr, uint8_t * data)
{
	uint8_t high_addr = (data_addr >> 8) & 0xff;
	uint8_t low_addr = data_addr & 0xff;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, chip_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, high_addr, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, low_addr, ACK_CHECK_EN);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, chip_addr << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
	i2c_master_read_byte(cmd, data, NACK_VAL);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	return ret;
}

static esp_err_t WriteReg16(EEPROM_t * dev, i2c_port_t i2c_port, int chip_addr, uint16_t data_addr, uint8_t data)
{
	uint8_t high_addr = (data_addr >> 8) & 0xff;
	uint8_t low_addr = data_addr & 0xff;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, chip_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, high_addr, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, low_addr, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	usleep(1000*2);
	return ret;
}

esp_err_t ReadRom(EEPROM_t * dev, uint16_t data_addr, uint8_t * data)
{
	if (data_addr > dev->_bytes) return 0;

	if (dev->_kbits < 32) {
		int blockNumber = data_addr / 256;
		uint16_t _data_addr = data_addr - (blockNumber * 256);
		int _chip_addr = dev->_chip_addr + blockNumber;
		ESP_LOGD(TAG, "ReadRom _chip_addr=%x _data_addr=%d", _chip_addr, _data_addr);
		return ReadReg8(dev, dev->_i2c_port, _chip_addr, _data_addr, data);
	} else {
		int _chip_addr = dev->_chip_addr;
		return ReadReg16(dev, dev->_i2c_port, _chip_addr, data_addr, data);
	}
}

esp_err_t WriteRom(EEPROM_t * dev, uint16_t data_addr, uint8_t data)
{
	if (data_addr > dev->_bytes) return 0;

	if (dev->_kbits < 32) {
		int blockNumber = data_addr / 256;
		uint16_t _data_addr = data_addr - (blockNumber * 256);
		int _chip_addr = dev->_chip_addr + blockNumber;
		ESP_LOGD(TAG, "WriteRom _chip_addr=%x _data_addr=%d", _chip_addr, _data_addr);
		return WriteReg8(dev, dev->_i2c_port, _chip_addr, _data_addr, data);
	} else {
		int _chip_addr = dev->_chip_addr;
		return WriteReg16(dev, dev->_i2c_port, _chip_addr, data_addr, data); 
	}
}

uint16_t MaxAddress(EEPROM_t * dev) {
	return dev->_bytes;
}

