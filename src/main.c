#include <string.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_adc.h"
#include "adcdrv.h"
#include "dmactrl.h"
#include "msg.h"
#include "em_cmu.h"
#include "timer.h"
#include "udelay.h"
#include "hal-config.h"
#include "main.h"
#include "uartdrv.h"
#include "Typedefs.h"

#define CMU_MODE_DIGEXTCLK	2

void CMU_ClockModeSet(uint32_t mode)
{
  CMU->CTRL = (CMU->CTRL & ~_CMU_CTRL_HFXOMODE_MASK) | mode;
}

void Clock_config(void)
{
	/*
	 * Enable clocks required
	 * */
	/* Prescale the HFCORECLK -> HF/2 = 50/2 = 25Mhz */
	//CMU_ClockDivSet(cmuClock_HF, cmuClkDiv_2);
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_USART0, true);

	SystemCoreClockUpdate();

	CMU_OscillatorEnable(cmuOsc_HFXO, true, true);
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
	CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);

	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);
	CMU_OscillatorEnable(cmuSelect_ULFRCO, true, true);
	CMU_ClockEnable(cmuClock_HFLE, true);
}

int main(void)
{
	/* Chip errata */
	CHIP_Init();

	/*
	 * config needed clock
	 * */
	Clock_config();

	/*
	 * RS422 Uart init for delivering converted data
	 * */
	uartSetup();

	/*
	 * config 7 ADC channels
	 * */
	ADCConfig();

	/*
	 * config one DMA channel for transferring ADC sample results
	 * to specific RAM buffer.
	 * */
	DMAConfig();

	/*
	 * enable ADC
	 * */
	DMA_ADC_Start();

  	UDELAY_Calibrate();
  	Delay_ms(500);

	while (1) {
		collectFrame();
		sendFrame();
		Delay_ms(2);
	}
}





