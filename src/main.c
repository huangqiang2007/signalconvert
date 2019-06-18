#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_adc.h"
#include "adcdrv.h"
#include "dmactrl.h"
//#include "temperature.h"
#include "msg.h"
#include "em_cmu.h"
//#include "i2csisconfig.h"
//#include "i2csis.h"
#include "timer.h"
#include "crc.h"
#include "udelay.h"
#include "hal-config.h"
#include "main.h"
#include "Uart.h"
#include "EFM32_Timer.h"
#include <string.h>
#include "Typedefs.h"
#define CMU_MODE_DIGEXTCLK    2

uint32_t sleeptime=30;//30s»½ÐÑÒ»´Î
int sleepflag=0;

void CMU_ClockModeSet(uint32_t mode)
{
  CMU->CTRL = (CMU->CTRL & ~_CMU_CTRL_HFXOMODE_MASK) | mode;
}

int main(void)
{
	/* Chip errata */
	CHIP_Init();

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

	/*
	 * Uart init for delivering converted data
	 * */
	Uart_Init();

	/*
	 * config 7 ADC channels
	 * */
	ADCConfig();

	/*
	 * config one DMA channel for transferring ADC sample results
	 * to specific RAM buffer.
	 * */
	DMAConfig();


  	UDELAY_Calibrate();
  	setupTimer1();
  	Delay_ms(500);


	while (1) {
	}
}





