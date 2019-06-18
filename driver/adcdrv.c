/*
 * adcdrv.c
 *
 *  Created on: 2017Äê7ÔÂ4ÈÕ
 *      Author: Administrator
 */

#include <string.h>
#include "em_dma.h"
#include "em_adc.h"
#include "em_cmu.h"
#include "time.h"
#include "adcdrv.h"
#include "dmactrl.h"
#include "uartdrv.h"
#include <stdbool.h>

#define ADC_CLK_1M 1000000

volatile ADC_SAMPLE_DATA_QUEUEDef adc_sample_data_queue = {0};
DMA_CB_TypeDef dma_adc_cb;
uint8_t sample_result[ADC_SCAN_LOOPS * ADC_CHNL_NUM];
uint8_t g_convResult[ADC_CHNL_NUM] = {0};

/*
 * frame number
 * */
volatile uint8_t g_frameNum = 0;

/*
 * RS422 data frame
 * */
volatile UartFrame g_RS422frame = {0};

/*
 * @brief
 *   Configure ADC for scan mode.
 * */
void ADCConfig(void)
{
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
	ADC_InitScan_TypeDef scanInit = ADC_INITSCAN_DEFAULT;

	CMU_ClockEnable(cmuClock_ADC0, true);

	/*
	* Init common issues for both single conversion and scan mode
	* */
	init.timebase = ADC_TimebaseCalc(0);
	init.prescale = ADC_PrescaleCalc(ADC_CLK_1M, 0);
	init.ovsRateSel = _ADC_CTRL_OVSRSEL_X32;
	ADC_Init(ADC0, &init);

	/*
	* Init for scan sequence use: 7 AD sample channels
	* */
	scanInit.rep = 1;
	scanInit.reference = adcRef2V5;
	scanInit.resolution = _ADC_SINGLECTRL_RES_8BIT;
	scanInit.input = ADC_SCANCTRL_INPUTMASK_CH0 | ADC_SCANCTRL_INPUTMASK_CH1 | ADC_SCANCTRL_INPUTMASK_CH2 |
					   ADC_SCANCTRL_INPUTMASK_CH3 | ADC_SCANCTRL_INPUTMASK_CH4 | ADC_SCANCTRL_INPUTMASK_CH5 |
					   ADC_SCANCTRL_INPUTMASK_CH6;
	ADC_InitScan(ADC0, &scanInit);
}

static inline bool sampleQueueEmpty(ADC_SAMPLE_DATA_QUEUEDef *queue)
{
	if (queue->samples == 0)
		return true;
	else
		return false;
}

static inline bool sampleQueueFull(ADC_SAMPLE_DATA_QUEUEDef *queue)
{
	if (queue->samples == ADC_SAMPLE_BUFFER_NUM)
		return true;
	else
		return false;
}

static ADC_SAMPLE_BUFFERDef* DMA_getFreeSampleBuffer(void)
{
	ADC_SAMPLE_BUFFERDef *pbuff = NULL;

	if (!sampleQueueFull(&adc_sample_data_queue)) {
		pbuff = &(adc_sample_data_queue.adc_smaple_data[adc_sample_data_queue.in]);
		adc_sample_data_queue.in++;
		if (adc_sample_data_queue.in == ADC_SAMPLE_BUFFER_NUM - 1)
			adc_sample_data_queue.in = 0;
	}

	return pbuff;
}

static ADC_SAMPLE_BUFFERDef* DMA_getValidBuffer(void)
{
	ADC_SAMPLE_BUFFERDef *pbuff = NULL;

	if (!sampleQueueEmpty(&adc_sample_data_queue)) {
		pbuff = &(adc_sample_data_queue.adc_smaple_data[adc_sample_data_queue.out]);
		adc_sample_data_queue.out++;
		if (adc_sample_data_queue.out == ADC_SAMPLE_BUFFER_NUM - 1)
			adc_sample_data_queue.out = 0;
	}

	return pbuff;
}

void DMA_ADC_callback(unsigned int channel, bool primary, void *user)
{
	ADC_SAMPLE_BUFFERDef *pSampleBuff = NULL;

	pSampleBuff = DMA_getFreeSampleBuffer();
	if (!pSampleBuff)
		return;

	memcpy(pSampleBuff->adc_sample_buffer, sample_result, ADC_SCAN_LOOPS * ADC_CHNL_NUM);

	/*
	 * update sample counter
	 * */
	adc_sample_data_queue.samples++;
}

/*
 *@brief
 *   Configure DMA usage for this application.
 * */
void DMAConfig(void)
{
	DMA_Init_TypeDef dmaInit;
	DMA_CfgDescr_TypeDef descrCfg;
	DMA_CfgChannel_TypeDef chnlCfg;

	CMU_ClockEnable(cmuClock_DMA, true);

	/*
	* Configure general DMA issues
	* */
	dmaInit.hprot = 0;
	dmaInit.controlBlock = dmaControlBlock;
	DMA_Init(&dmaInit);

	/*
	* Configure DMA channel used
	* */
	dma_adc_cb.cbFunc = DMA_ADC_callback;
	dma_adc_cb.userPtr = (void *)&adc_sample_data_queue;

	chnlCfg.highPri = false;
	chnlCfg.enableInt = false;
	chnlCfg.select = DMAREQ_ADC0_SCAN;
	chnlCfg.cb = &dma_adc_cb;
	DMA_CfgChannel(DMA_CHANNEL, &chnlCfg);

	/*
	* one byte per transfer
	* */
	descrCfg.dstInc = dmaDataInc1;
	descrCfg.srcInc = dmaDataIncNone;
	descrCfg.size = dmaDataSize1;
	descrCfg.arbRate = dmaArbitrate1;
	descrCfg.hprot = 0;
	DMA_CfgDescr(DMA_CHANNEL, true, &descrCfg);
}

void DMA_ADC_start(void)
{
	DMA_ActivateBasic(DMA_CHANNEL, true, false, (void *)sample_result,
		(void *)&(ADC0->SCANDATA), ADC_SCAN_LOOPS * ADC_CHNL_NUM - 1);

	/*
	 * Start Scan
	 * */
	ADC_Start(ADC0, adcStartScan);
}

int getAverageSampleValue(void)
{
	ADC_SAMPLE_BUFFERDef *pSampleBuff = NULL;
	int i = 0;
	uint32_t sumSampleValue[ADC_CHNL_NUM] = {0};

	pSampleBuff = DMA_getValidBuffer();
	if (!pSampleBuff)
		return -1;

	for (i = 0; i < ADC_SCAN_LOOPS; i += 7) {
		sumSampleValue[0] += pSampleBuff->adc_sample_buffer[i];
		sumSampleValue[1] += pSampleBuff->adc_sample_buffer[i + 1];
		sumSampleValue[2] += pSampleBuff->adc_sample_buffer[i + 2];
		sumSampleValue[3] += pSampleBuff->adc_sample_buffer[i + 3];
		sumSampleValue[4] += pSampleBuff->adc_sample_buffer[i + 4];
		sumSampleValue[5] += pSampleBuff->adc_sample_buffer[i + 5];
		sumSampleValue[6] += pSampleBuff->adc_sample_buffer[i + 6];
	}

	/*
	 * update sample counter to return the buffer to queue,
	 * disable interrupt to avoid race condition.
	 * */
	CORE_CriticalDisableIrq();
	adc_sample_data_queue.samples--;
	CORE_CriticalEnableIrq();

	for (i = 0; i < ADC_CHNL_NUM; i++)
		g_convResult[i] = sumSampleValue[i] / ADC_SCAN_LOOPS;

	return 0;
}

/*
 * frame format uint8_t[12]:< 0x55, 0xAA, frame_len, adcSample[7], frame_cnt, crc >
 * */
void constructFrame(void)
{
	int i = 0;
	int sum = 0;

	g_RS422frame.in_sending = 0;
	g_RS422frame.uartFrame[0] = 0x55;
	g_RS422frame.uartFrame[1] = 0xAA;
	g_RS422frame.uartFrame[2] = 8;

	for (i = 3; i <= 9; i++)
		g_RS422frame.uartFrame[i] = g_convResult[i - 3];

	g_RS422frame.uartFrame[10] = (++g_frameNum) & 0xFF;

	for (i = 2; i <= 10; i++)
		sum += g_RS422frame.uartFrame[i];

	g_RS422frame.uartFrame[11] = sum & 0xFF;
}

void sendFrame(void)
{
	/*
	 * if the current frame is in UART sending, it should not
	 * send a new frame.
	 * */
	if (g_RS422frame.in_sending == 1)
		return;

	/*
	 * if there is no valid converted sample data, go back directly.
	 * */
	if (getAverageSampleValue() < 0)
		return;

	constructFrame();

	uartPutData(g_RS422frame.uartFrame, UARTFRAME_LEN_12B);
}
