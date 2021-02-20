#pragma once

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t i2c_master_init(void);
void i2c_detect(void);

esp_err_t i2c_master_read_slave_reg(uint8_t i2c_addr, uint8_t i2c_reg,
                                    uint8_t *data_rd, size_t size);
esp_err_t i2c_master_write_slave_reg(uint8_t i2c_addr, uint8_t i2c_reg,
                                     uint8_t *data_wr, size_t size);


uint16_t i2c_master_read_slave_reg_16(uint8_t i2c_addr, uint8_t i2c_reg);
void i2c_master_write_slave_reg_16_with_mask(uint8_t i2c_addr, uint8_t i2c_reg,
                                             uint16_t _mask, uint16_t _bits,
                                             uint8_t _start_position);

#ifdef __cplusplus
}
#endif