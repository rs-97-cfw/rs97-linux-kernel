#ifndef __JZ_GPIO_CTRL_H__
#define __JZ_GPIO_CTRL_H__


void gpio_as_output(unsigned int pin);
void gpio_as_input(unsigned int pin);
void gpio_set_pin(unsigned int pin);
int gpio_get_pin(unsigned int pin);
void gpio_clear_pin(unsigned int pin);
void gpio_enable_pull(unsigned int pin);
void gpio_disable_pull(unsigned int pin);

#endif /*__JZ_GPIO_CTRL_H__*/

