//----------------------------------------------------------------------------
// Description: 模拟I2C驱动代码，支持互斥锁保护读写函数
//
// Author     : dengfu
// Date       : 2024/03/02
// Filename   : sim_i2c.c
//----------------------------------------------------------------------------
#include <stdint.h>
#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "sim_i2c.h"

// Mcu初始化时调用sim_i2c_init进行传入
static SemaphoreHandle_t* sim_i2c_mutex;
static sim_i2c_pin_t *sim_i2c_pin;

extern uint8_t mcu_get_pin_value(uint8_t port, uint8_t pin);
extern void    mcu_set_pin_value(uint8_t port, uint8_t pint, uint8_t level);

#define MCU_SDA_R(port) mcu_get_pin_value(sim_i2c_pin[port].sda_port, sim_i2c_pin[port].sda_pin)
#define MCU_SDA_1(port) mcu_set_pin_value(sim_i2c_pin[port].sda_port, sim_i2c_pin[port].sda_pin, 1)
#define MCU_SDA_0(port) mcu_set_pin_value(sim_i2c_pin[port].sda_port, sim_i2c_pin[port].sda_pin, 0)
#define MCU_SCL_1(port) mcu_set_pin_value(sim_i2c_pin[port].scl_port, sim_i2c_pin[port].scl_pin, 1)
#define MCU_SCL_0(port) mcu_set_pin_value(sim_i2c_pin[port].scl_port, sim_i2c_pin[port].scl_pin, 0)

#define SIM_I2C_PORT_CHECK(port, fail) { \
    if (port >= CONFIG_SIM_I2C_MAX_PORT) return fail; \
}

static void i2c_delay(void)
{
    volatile uint16_t i = CONFIG_SIM_I2C_SPEED_DELAY;

    while(i--);
}

static void i2c_delay_half(void)
{
    volatile uint16_t i = CONFIG_SIM_I2C_SPEED_DELAY / 2;

    while(i--);
}

static void i2c_start(uint8_t port)
{
    MCU_SDA_1(port);
    i2c_delay();
    MCU_SCL_1(port);

    i2c_delay();
    i2c_delay();
    MCU_SDA_0(port);

    i2c_delay();
    i2c_delay();
    MCU_SCL_0(port);
}

static uint8_t i2c_write_byte(uint8_t port, uint8_t data)
{
    uint8_t i;
    uint8_t ack;

    for (i = 0; i < 8; i++)
    {
        MCU_SCL_0(port);
        i2c_delay_half();

        if (data & 0x80) 
        {
            MCU_SDA_1(port);
        }
        else
        {
            MCU_SDA_0(port);
        }

        i2c_delay_half(port);

        MCU_SCL_1(port);
        i2c_delay();
        data <<= 1;
    }

    MCU_SCL_0(port);
    MCU_SDA_1(port);
    i2c_delay();
    i2c_delay();
    MCU_SCL_1(port);
    i2c_delay();

    ack = MCU_SDA_R(prot) ? 1 : 0;
    if (ack == 0)
        MCU_SDA_0(port);

    MCU_SCL_0(port);
    i2c_delay();
    MCU_SDA_1(port);
    i2c_delay();

    return ack;
}

static uint8_t i2c_read_byte(uint8_t port, uint8_t ack)
{
    uint8_t i, read_data = 0;

    MCU_SCL_0(port);
    i2c_delay();
    MCU_SDA_1(port);
    i2c_delay();

    for (i = 0; i < 8; i++) 
    {
        MCU_SCL_1(port);
        i2c_delay();
        read_data <<= 1;

        if(MCU_SDA_R(port))
            read_data |= 1;

        MCU_SCL_0(port);
        i2c_delay();
    }

    if (ack)
    {
        MCU_SDA_1(port);
    }
    else
    {
        MCU_SDA_0(port);
    }

    i2c_delay();
    MCU_SCL_1(port);
    i2c_delay();
    MCU_SCL_0(port);

    return read_data;
}

static void i2c_stop(uint8_t port)
{
    MCU_SDA_0(port);
    i2c_delay();
    MCU_SCL_1(port);
    i2c_delay();
    i2c_delay();
    MCU_SDA_1(port);
}

uint8_t sim_i2c_read_buf(uint8_t port, uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i, ack;

    SIM_I2C_PORT_CHECK(port, 1);

    i2c_start(port);

    do
    {
        ack = i2c_write_byte(port, addr & 0xFE);    // SlaveID + Write

        if (ack) break;

        ack = i2c_write_byte(port, reg);            // RegAddr

        if (ack) break;

        i2c_start(port);                            // Restart

        ack = i2c_write_byte(port, addr | 0x01);    // SlaveID + Read

        if (ack) break;

        for (i = 0; i < len; i++)                   // Read data
        {
            *buf++ = i2c_read_byte(port, ((i + 1) != len) ? 0 : 1); 
        }

    } while (0);

     i2c_stop(port);

     return ack;
}

uint8_t sim_i2c_write_buf(uint8_t port, uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i, ack;

    SIM_I2C_PORT_CHECK(port, 1);

    i2c_start(port);

    ack = i2c_write_byte(port, addr & 0xFE);

    if(ack == 0)
    {
        ack = i2c_write_byte(port, reg);

        for(i = 0; i < len && (ack == 0); i++)
        {
            ack = i2c_write_byte(*buf++);
        }
    }

    i2c_stop(port);

    return ack;
}

static bool sim_i2c_force_unlock(uint8_t port)
{
    if (sim_i2c_mutex[port])
    {
        vSemaphoreDelete(sim_i2c_mutex[port]);
    }
    sim_i2c_mutex[port] = xSemaphoreCreateMutex();
    return true;
}

bool sim_i2c_lock(uint8_t port)
{
    TickType_t timeout;

    SIM_I2C_PORT_CHECK(port, false);

    timeout = pdMS_TO_TICKS(CONFIG_SIM_I2C_LOCK_TIMEOUT);

    if (xSemaphoreTake(sim_i2c_mutex[port], timeout) == pdTRUE)
    {
        return true;
    }
    else
    {
        sim_i2c_force_unlock(port);
        return (xSemaphoreTake(sim_i2c_mutex[port], timeout)) == pdTRUE);
    }
}

bool sim_i2c_unlock(uint8_t port)
{
    SIM_I2C_PORT_CHECK(port, false);

    return (xSemaphoreGive(sim_i2c_mutex[port], timeout)) == pdTRUE);
}

// mutex and gpio contorl function config
void sim_i2c_init(sim_i2c_pin_t *i2c_pin, SemaphoreHandle_t *i2c_mutex)
{
    sim_i2c_pin = i2c_pin;
    sim_i2c_mutex = i2c_mutex;
}


uint8_t sim_i2c_read_buf_lock(uint8_t port, uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t nack = 1;

    SIM_I2C_PORT_CHECK(port, 1);

    if (sim_i2c_lock(port))
    {
        nack = sim_i2c_read_buf(port, addr, reg, buf, len);

        sim_i2c_unlock(port);
    }

    return nack;
}


uint8_t sim_i2c_write_buf_lock(uint8_t port, uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t nack = 1;

    SIM_I2C_PORT_CHECK(port, 1);

    if (sim_i2c_lock(port))
    {
        nack = sim_i2c_write_buf(port, addr, reg, buf, len);

        sim_i2c_unlock(port);
    }

    return nack;
}

