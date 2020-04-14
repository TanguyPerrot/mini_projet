#ifndef GPIO_H
#define GPIO_H

#include <stm32f407xx.h>
#include <stdbool.h>

void gpio_config_output_opendrain(GPIO_TypeDef *port, unsigned int pin);
void gpio_set(GPIO_TypeDef *port, unsigned int pin);
void gpio_clear(GPIO_TypeDef *port, unsigned int pin);
void gpio_toggle(GPIO_TypeDef *port, unsigned int pin);
void gpio_config_push_pull(GPIO_TypeDef *port, unsigned int pin);
bool gpio_one_selector(GPIO_TypeDef *port, unsigned int pin);
int gpio_read_selector(void);
//GPIO_TypeDef *port, unsigned int pin
#endif /* GPIO_H */
