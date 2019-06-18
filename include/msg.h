/*
 * config.h
 *
 *  Created on: 2017Äê7ÔÂ10ÈÕ
 *      Author: Administrator
 */

#ifndef INCLUDE_MSG_H_
#define INCLUDE_MSG_H_

#include <stdint.h>

struct msg
{
  uint8_t headA;
  uint8_t headB;
  uint8_t len;
  uint8_t seq;
  uint8_t dev;
  uint8_t type;
  uint8_t buf[8];
  uint8_t crcA;
  uint8_t crcB;
} __attribute__ ((packed));

typedef uint32_t (*CallBackFun)(uint8_t *ch);

int get_msg(struct msg *pMsg, CallBackFun GetOneByte);

#endif /* INCLUDE_MSG_H_ */
