/**
 * MatekSys F405 USB board functions
 */

#include <px4_platform_common/px4_config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>
#include <arm_internal.h>
#include <stm32.h>
#include "board_config.h"

__EXPORT void stm32_usbinitialize(void)
{
#ifdef CONFIG_STM32_OTGFS
	stm32_configgpio(GPIO_OTGFS_VBUS);
#endif
}

__EXPORT void stm32_usbsuspend(FAR struct usbdev_s *dev, bool resume)
{
	uinfo("resume: %d\n", resume);
}
