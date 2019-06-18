/*
 * crc.h
 *
 *  Created on: 2017Äê7ÔÂ10ÈÕ
 *      Author: Administrator
 */

#ifndef COMMON_CRC_H_
#define COMMON_CRC_H_

#include <stdint.h>


/************************** Constant Definitions *****************************/

#define CRC16_INIT  0x0000

/************************** Function Prototypes ******************************/

uint16_t CalCRC16(void *pData, uint32_t dwNumOfBytes);


#endif /* COMMON_CRC_H_ */
