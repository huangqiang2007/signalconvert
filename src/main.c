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

#if 0

#define adcFreq   13000000

volatile uint32_t sample;
volatile uint32_t millivolts;

/**************************************************************************//**
 * @brief  Initialize ADC function
 *****************************************************************************/
void initADC (void)
{
  // Enable ADC0 clock
  CMU_ClockEnable(cmuClock_ADC0, true);

  // Declare init structs
  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

  // Modify init structs and initialize
  init.prescale = ADC_PrescaleCalc(adcFreq, 0); // Init to max ADC clock for Series 0

  initSingle.diff       = false;        // single ended
  initSingle.reference  = adcRef2V5;    // internal 2.5V reference
  initSingle.resolution = adcRes12Bit;  // 12-bit resolution

  // Select ADC input. See README for corresponding EXP header pin.
  initSingle.input = adcSingleInputCh7;
  init.timebase = ADC_TimebaseCalc(0);

  ADC_Init(ADC0, &init);
  ADC_InitSingle(ADC0, &initSingle);
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int adc_test(void)
{

  initADC();

  // Infinite loop
  while(1)
  {
    // Start ADC conversion
    ADC_Start(ADC0, adcStartSingle);

    // Wait for conversion to be complete
    while(!(ADC0->STATUS & _ADC_STATUS_SINGLEDV_MASK));

    // Get ADC result
    sample = ADC_DataSingleGet(ADC0);

    // Calculate input voltage in mV
    millivolts = (sample * 2500) / 4096;
  }
}
#endif

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
		sendFrame();
		Delay_ms(2);
	}
}





