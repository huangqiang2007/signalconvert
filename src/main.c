#include <string.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_adc.h"
#include "adcdrv.h"
#include "dmactrl.h"
#include "em_cmu.h"
#include "timer.h"
#include "udelay.h"
#include "hal-config.h"
#include "main.h"
#include "timer.h"
#include "uartdrv.h"
#include "Typedefs.h"

void Clock_config(void)
{
	SystemCoreClockUpdate();

	/*
	 * chose external crystal oscillator as clock source.
	 * */
	CMU_OscillatorEnable(cmuOsc_HFXO, true, true);
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
	CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);

	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);
	CMU_OscillatorEnable(cmuSelect_ULFRCO, true, true);
	CMU_ClockEnable(cmuClock_HFLE, true);

	/*
	 * Enable clocks required
	 * */
	//CMU_ClockDivSet(cmuClock_HF, cmuClkDiv_2);
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_USART0, true);
}

int main(void)
{
	/* Chip errata */
	CHIP_Init();

	/*
	 * config needed clock
	 * */
	Clock_config();

	//adc_test();

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

	/*
	 * timer init
	 * */
	Timer_init();

	//UDELAY_Calibrate();
	Delay_ms(500);

	while (1) {
		collectFrame();

		/*
		 * to send one frame according to 20ms interval,
		 * move the logic to TIMER0_IRQHandler().
		 * */
		//sendFrame();
		//Delay_ms(2);
	}
}





