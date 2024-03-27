#if !defined(_GPIO_H_)
#define _GPIO_H_

typedef struct
{
    uint8_t port;
    uint8_t pin;
    uint8_t function;
    uint8_t output_en;
    uint8_t input_en;
    uint8_t opendrian_en;
    uint8_t default_output;
    
} mcu_pin_config_t;



void    mcu_gpio_init    (void);
uint8_t mcu_get_pin_value(uint8_t port, uint8_t pin);
void    mcu_set_pin_value(uint8_t port, uint8_t pint, uint8_t level);

#endif // _GPIO_H_


