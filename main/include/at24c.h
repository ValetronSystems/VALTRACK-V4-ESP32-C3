#ifndef _AT24C_H_
#define _AT24C_H_
#include "esp_err.h"
#include "driver/i2c.h"
#define I2C_FREQUENCY   100000
#define ACK_CHECK_EN    0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS   0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL         0x0     /*!< I2C ack value */
#define NACK_VAL        0x1     /*!< I2C nack value */

typedef struct {
	uint16_t _i2c_port;
	uint16_t _chip_addr;
	uint16_t _kbits;
	uint16_t _bytes;
} EEPROM_t;

esp_err_t InitRom(EEPROM_t * dev, i2c_port_t i2c_port);
//static esp_err_t ReadReg8(EEPROM_t * dev, i2c_port_t i2c_port, int chip_addr, uint8_t data_addr, uint8_t * data);
//static esp_err_t WriteReg8(EEPROM_t * dev, i2c_port_t i2c_port, int chip_addr, uint8_t data_addr, uint8_t data);
//static esp_err_t ReadReg16(EEPROM_t * dev, i2c_port_t i2c_port, int chip_addr, uint16_t data_addr, uint8_t * data);
//static esp_err_t WriteReg16(EEPROM_t * dev, i2c_port_t i2c_port, int chip_addr, uint16_t data_addr, uint8_t data);
esp_err_t ReadRom(EEPROM_t * dev, uint16_t data_addr, uint8_t * data);
esp_err_t WriteRom(EEPROM_t * dev, uint16_t data_addr, uint8_t data);
uint16_t MaxAddress(EEPROM_t * dev);


#endif /* AT24C_H_ */
