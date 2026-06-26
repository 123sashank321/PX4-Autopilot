/**
 * MatekSys F405 SPI board hooks (pure C)
 *
 * NuttX calls stm32_spiNregister as a board-level hook.
 * Must be in a .c file to avoid C++ name-mangling conflicts.
 */

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <stm32.h>
#include <stdint.h>
#include <errno.h>

#include "board_config.h"

/* ── CS initialisation ───────────────────────────────────────────────────── */

void stm32_spiinitialize(void)
{
	/* SPI1 — PA4 CS idle high */
	stm32_configgpio(GPIO_SPI1_SCK);
	stm32_configgpio(GPIO_SPI1_MISO);
	stm32_configgpio(GPIO_SPI1_MOSI);
	stm32_configgpio(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |
	                 GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN4);

	/* SPI2 — PB12 CS idle high */
	stm32_configgpio(GPIO_SPI2_SCK);
	stm32_configgpio(GPIO_SPI2_MISO);
	stm32_configgpio(GPIO_SPI2_MOSI);
	stm32_configgpio(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |
	                 GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN12);
}

/* ── Per-bus CS select / status ─────────────────────────────────────────── */

void stm32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	stm32_gpiowrite(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |
	                GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN4, !selected);
}

uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}

void stm32_spi2select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	stm32_gpiowrite(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |
	                GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN12, !selected);
}

uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}

/* SD card media-change callback registration — no card detect pin on this board */
int stm32_spi1register(struct spi_dev_s *dev, spi_mediachange_t callback, void *arg)
{
	return OK;
}

int stm32_spi2register(struct spi_dev_s *dev, spi_mediachange_t callback, void *arg)
{
	return OK;
}
