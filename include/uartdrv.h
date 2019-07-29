#ifndef UARTDRV_H_
#define UARTDRV_H_

#include <stdint.h>

/*
 * frame format layout
 *
 *struct UartFrameDef {
 *	uint8_t header0;
 *	uint8_t header1;
 *	uint8_t len;
 *	uint8_t data0;
 *	uint8_t data1;
 *	uint8_t data2;
 *	uint8_t data3;
 *	uint8_t data4;
 *	uint8_t data5;
 *	uint8_t data6;
 *	uint8_t serial;
 *	uint8_t sum;
 *};
*/

#define UARTFRAME_LEN_12B 12
/*
 * convert AD samples into RS422 frame
 * */
typedef struct {
	/*
	 * the frame of 12 bytes
	 * */
	uint8_t uartFrame[UARTFRAME_LEN_12B];
} UartFrame;

/* Function prototypes */
extern void uartSetup(void);
extern void uartPutData(volatile uint8_t * dataPtr, uint32_t dataLen);
extern uint32_t uartGetData(uint8_t * dataPtr, uint32_t dataLen);
extern void    uartPutChar(uint8_t charPtr);
extern uint8_t uartGetChar(void);
extern uint32_t uartReadChar(uint8_t *data);
extern int UartFrameEnqueue(UartFrame *uFrame);
extern UartFrame* UartFrameDequeue(void);

#endif /* UARTDRV_H_ */
