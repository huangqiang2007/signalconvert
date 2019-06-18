/*
 * adcdrv.h
 *
 *  Created on: 2016-11-8
 *      Author: admin
 */

#ifndef ADCDRV_H_
#define ADCDRV_H_

#include "main.h"

#define ADC_CHNL_NUM 7
#define ADC_SCAN_LOOPS 100

#define ADC_SAMPLE_BUFFER_NUM 4

/*
 * one ADC sample buffer, the total buffer size 100 * 7 bytes
 *
 * @free: 0: the buffer is empty; 1: the buffer is full and valid
 * @adc_sample_buffer: the array stores the sampled data
 * */
typedef struct {
	bool free;
	uint8_t adc_sample_buffer[ADC_SCAN_LOOPS * ADC_CHNL_NUM];
} ADC_SAMPLE_BUFFERDef;

/*
 * the buffer queue for sampled data
 *
 * @valid_buff_num: the number indicates how many buffer stores valid sampled data
 * @adc_smaple_data[]: the array of ADC_SAMPLE_BUFFER type
 * */
typedef struct {
	int8_t valid_buff_num;
	volatile ADC_SAMPLE_BUFFERDef adc_smaple_data[ADC_SAMPLE_BUFFER_NUM];
} ADC_SAMPLE_DATA_QUEUEDef;

void ADCConfig(void);
void DMAConfig(void);
int VoltageDetect(uint32_t minvol);

//void GetVoltage(struct RF_Record *record);

void ADCSingleConfig(void);
uint32_t ADCSingleGet();

#endif /* ADCDRV_H_ */
