#ifndef ADCDRV_H_
#define ADCDRV_H_

#include "main.h"

#define ADC_CHNL_NUM 8
#define ADC_SCAN_LOOPS 1
#define DMA_BUFF_LEN (ADC_CHNL_NUM * 2)

#define ADC_SAMPLE_BUFFER_NUM 4

/*
 * one ADC sample buffer, the total buffer size 100 * 7 bytes
 *
 * @free: 0: the buffer is empty; 1: the buffer is full and valid
 * @adc_sample_buffer: the array stores the sampled data
 * */
typedef struct {
	volatile uint8_t adc_sample_buffer[DMA_BUFF_LEN];
} ADC_SAMPLE_BUFFERDef;

/*
 * the buffer queue for sampled data
 *
 * @samples: how many items in the queue
 * @in: queue head for data enqueue
 * @out: queue tail for data dequeue
 * @adc_smaple_data[]: the array of ADC_SAMPLE_BUFFER type
 * */
typedef struct {
	volatile int8_t samples;
	int8_t in, out;
	ADC_SAMPLE_BUFFERDef adc_smaple_data[ADC_SAMPLE_BUFFER_NUM];
} AdcSampleDataQueueDef;

extern void ADCConfig(void);
extern void DMAConfig(void);
extern void DMA_ADC_Start(void);
extern void collectFrame(void);
extern void sendFrame(void);

#endif /* ADCDRV_H_ */
