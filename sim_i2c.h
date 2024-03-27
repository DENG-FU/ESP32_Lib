#if !defined(_SIM_I2C_H_)
#define _SIM_I2C_H_

#if !defined(CONFIG_SIM_I2C_MAX_PORT)
#define CONFIG_SIM_I2C_MAX_PORT 1
#endif

#if !defined(CONFIG_SIM_I2C_SPEED_DELAY)
#define CONFIG_SIM_I2C_SPEED_DELAY 50
#endif

#if !defined(CONFIG_SIM_I2C_LOCK_TIMEOUT)
#define CONFIG_SIM_I2C_LOCK_TIMEOUT 5000 // 5s
#endif // CONFIG_SIM_I2C_LOCK_TIMEOUT

#include "freertos/semphr.h"

typedef struct 
{
    uint8_t scl_port;
    uint8_t scl_pin;

    uint8_t sda_port;
    uint8_t sda_pin;

}sim_i2c_pin_t;

void sim_i2c_test(void);

void    sim_i2c_init          (sim_i2c_pin_t *i2c_pin, SemaphoreHandle_t *i2c_mutex);
bool    sim_i2c_lock          (uint8_t port);
bool    sim_i2c_unlock        (uint8_t port);
uint8_t sim_i2c_read_buf      (uint8_t port, uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
uint8_t sim_i2c_write_buf     (uint8_t port, uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
uint8_t sim_i2c_read_buf_lock (uint8_t port, uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
uint8_t sim_i2c_write_buf_lock(uint8_t port, uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);


#endif // _SIM_I2C_H_
