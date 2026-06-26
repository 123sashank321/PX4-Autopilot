/**
 * MatekSys F405 LED backend
 * LED1: PB5 (active low)
 */

#include <px4_platform_common/px4_config.h>
#include <stdbool.h>
#include "stm32.h"
#include "board_config.h"

/* Forward declarations without including nuttx/board.h which redefines these as macros */
__BEGIN_DECLS
void led_init(void);
void led_on(int led);
void led_off(int led);
void led_toggle(int led);
__END_DECLS

__EXPORT void led_init(void)
{
	stm32_configgpio(GPIO_LED_BLUE);
}

static void phy_set_led(int led, bool state)
{
	if (led == 0) {
		stm32_gpiowrite(GPIO_LED_BLUE, !state);
	}
}

__EXPORT void led_on(int led)
{
	phy_set_led(led, true);
}

__EXPORT void led_off(int led)
{
	phy_set_led(led, false);
}

__EXPORT void led_toggle(int led)
{
	if (led == 0) {
		phy_set_led(led, !stm32_gpioread(GPIO_LED_BLUE));
	}
}

__EXPORT void board_autoled_initialize(void)
{
	stm32_configgpio(GPIO_LED_BLUE);
}

__EXPORT void board_autoled_on(int led)
{
	if (led == 1) {
		stm32_gpiowrite(GPIO_LED_BLUE, false);
	}
}

__EXPORT void board_autoled_off(int led)
{
	if (led == 1) {
		stm32_gpiowrite(GPIO_LED_BLUE, true);
	}
}
