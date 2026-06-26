/**
 * MatekSys F405 board initialization
 */

#include <px4_platform_common/px4_config.h>
#include <nuttx/config.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
#include <nuttx/spi/spi.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <syslog.h>

#include <arm_internal.h>
#include <stm32.h>
#include <stm32_gpio.h>
#include <stm32_spi.h>

/* drv_board_led not used on constrained F4 — call led_init() directly */
#include <systemlib/px4_macros.h>
#include <px4_arch/io_timer.h>
#include <px4_platform_common/init.h>
#include <px4_platform/board_dma_alloc.h>

#if defined(FLASH_BASED_PARAMS)
#  include <parameters/flashparams/flashfs.h>
#endif

__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
extern void stm32_spiinitialize(void);
__END_DECLS

__EXPORT void board_peripheral_reset(int ms) { UNUSED(ms); }

__EXPORT void board_on_reset(int status)
{
	for (int i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; ++i) {
		px4_arch_configgpio(io_timer_channel_get_gpio_output(i));
	}

	if (status >= 0) {
		up_mdelay(400);
	}
}

__EXPORT void stm32_boardinitialize(void)
{
	board_on_reset(-1);
	board_autoled_initialize();

	/* ADC pins: PC2=VBAT, PC3=CURR */
	stm32_configgpio(GPIO_ADC1_IN12);
	stm32_configgpio(GPIO_ADC1_IN13);

	stm32_configgpio(GPIO_PPM_IN);

	stm32_spiinitialize();
}

static struct spi_dev_s *spi1;
static struct spi_dev_s *spi2;

__EXPORT int board_app_initialize(uintptr_t arg)
{
	px4_platform_init();

	if (board_dma_alloc_init() < 0) {
		syslog(LOG_ERR, "DMA alloc FAILED\n");
	}

	/* LEDs — init directly, no /dev/led driver on this board */
	led_init();
	led_off(0);

	/* SPI1 — ICM42688-P IMU */
	spi1 = stm32_spibus_initialize(1);

	if (!spi1) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI1\n");
		return -ENODEV;
	}

	SPI_SETFREQUENCY(spi1, 10000000);
	SPI_SETBITS(spi1, 8);
	SPI_SETMODE(spi1, SPIDEV_MODE3);
	up_udelay(20);

	/* SPI2 — SD card */
	spi2 = stm32_spibus_initialize(2);

	if (!spi2) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI2\n");
		return -ENODEV;
	}

	up_udelay(20);

#if defined(FLASH_BASED_PARAMS)
	static sector_descriptor_t params_sector_map[] = {
		{1, 16 * 1024, 0x08004000},
		{0, 0, 0},
	};

	int result = parameter_flashfs_init(params_sector_map, NULL, 0);

	if (result != OK) {
		syslog(LOG_ERR, "[boot] FAILED to init flash params: %d\n", result);
		return -ENODEV;
	}

#endif

	px4_platform_configure();
	return OK;
}
