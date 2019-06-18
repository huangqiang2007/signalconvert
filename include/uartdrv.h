/*
 * uartdrv.h
 *
 *  Created on: 2017-1-17
 *      Author: admin
 */

#ifndef UARTDRV_H_
#define UARTDRV_H_

#include <stdint.h>

/*
 * convert AD samples into RS422 frame
 * */
typedef struct {
	/*
	 * 1: in sending, 0: invalid frame
	 * */
	uint8_t in_sending;

	/*
	 * the frame of 12 bytes
	 * */
	uint8_t uartFrame[12];
} UartFrame;

/* Function prototypes */
void uartSetup(void);
void uartPutData(uint8_t * dataPtr, uint32_t dataLen);
uint32_t uartGetData(uint8_t * dataPtr, uint32_t dataLen);
void    uartPutChar(uint8_t charPtr);
uint8_t uartGetChar(void);
uint32_t uartReadChar(uint8_t *data);

#endif /* UARTDRV_H_ */
