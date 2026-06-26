/**
 * MatekSys F405 timer configuration — Fixed-Wing
 *
 * PWM outputs (all TIM3 and TIM2 channels grouped):
 *  S1: TIM3 CH1 - PA6
 *  S2: TIM3 CH2 - PA7
 *  S3: TIM3 CH3 - PB0
 *  S4: TIM3 CH4 - PB1
 *  S5: TIM2 CH1 - PA0
 *  S6: TIM2 CH2 - PA1
 *  S7: TIM2 CH3 - PA2
 *  S8: TIM2 CH4 - PA3
 *
 * TIM2 and TIM3 must NOT appear in NuttX defconfig (PX4 owns them).
 */

#include <px4_arch/io_timer_hw_description.h>

constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
	initIOTimer(Timer::Timer3, DMA{DMA::Index1, DMA::Stream2, DMA::Channel5}),
	initIOTimer(Timer::Timer2, DMA{DMA::Index1, DMA::Stream1, DMA::Channel3}),
};

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	// TIM3 channels — grouped together
	initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel1}, {GPIO::PortA, GPIO::Pin6}),
	initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel2}, {GPIO::PortA, GPIO::Pin7}),
	initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel3}, {GPIO::PortB, GPIO::Pin0}),
	initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel4}, {GPIO::PortB, GPIO::Pin1}),
	// TIM2 channels — grouped together
	initIOTimerChannel(io_timers, {Timer::Timer2, Timer::Channel1}, {GPIO::PortA, GPIO::Pin0}),
	initIOTimerChannel(io_timers, {Timer::Timer2, Timer::Channel2}, {GPIO::PortA, GPIO::Pin1}),
	initIOTimerChannel(io_timers, {Timer::Timer2, Timer::Channel3}, {GPIO::PortA, GPIO::Pin2}),
	initIOTimerChannel(io_timers, {Timer::Timer2, Timer::Channel4}, {GPIO::PortA, GPIO::Pin3}),
};

constexpr io_timers_channel_mapping_t io_timers_channel_mapping =
	initIOTimerChannelMapping(io_timers, timer_io_channels);
