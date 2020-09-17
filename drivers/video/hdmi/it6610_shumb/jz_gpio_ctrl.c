#include <asm/jzsoc.h>

#include "jz_gpio_ctrl.h"

//#undef GPIO_GPA0_SPEC_USED

void gpio_as_output(unsigned int pin)
{
#ifndef GPIO_GPA0_SPEC_USED
	if(pin == 0)	return ;
#endif
	__gpio_as_output(pin);
}

void gpio_as_input(unsigned int pin)
{
#ifndef GPIO_GPA0_SPEC_USED
	if(pin == 0)	return ;
#endif
	__gpio_as_input(pin);
}

void gpio_set_pin(unsigned int pin)
{
#ifndef GPIO_GPA0_SPEC_USED
	if(pin == 0)	return ;
#endif
	__gpio_as_output(pin);
	__gpio_set_pin(pin);
}

int gpio_get_pin(unsigned int pin)
{
#ifndef GPIO_GPA0_SPEC_USED
	if(pin == 0){
		printk("%s:%d  GPA0 is true ???",__FILE__,__LINE__);
		return 0;
	}
#endif
	return __gpio_get_pin(pin);
}

void gpio_clear_pin(unsigned int pin)
{
#ifndef GPIO_GPA0_SPEC_USED
	if(pin == 0)	return ;
#endif
	__gpio_as_output(pin);
	__gpio_clear_pin(pin);
}

void gpio_enable_pull(unsigned int pin)
{
#ifndef GPIO_GPA0_SPEC_USED
	if(pin == 0)	return ;
#endif
	__gpio_enable_pull(pin);
}

void gpio_disable_pull(unsigned int pin)
{
#ifndef GPIO_GPA0_SPEC_USED
	if(pin == 0)	return ;
#endif
	__gpio_disable_pull(pin);
}

