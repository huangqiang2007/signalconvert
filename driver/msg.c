/*
 * config.c
 *
 *  Created on: 2017Äê7ÔÂ10ÈÕ
 *      Author: Administrator
 */

#include "msg.h"
#include "crc.h"

#define FIND_HEADA  0
#define FIND_HEADB  1
#define GET_LENGTH  2
#define GET_DATA    3
#define GET_CRCA    4
#define GET_CRCB    5

int get_msg(struct msg *pMsg, CallBackFun GetOneByte)
{
  static uint32_t state = FIND_HEADA;
  static uint32_t index = 0;
  static uint32_t index_max = 0;

  uint8_t ch;
  uint32_t ret;
  uint8_t *pBuf = (uint8_t *)&pMsg->seq;
  uint16_t crc;

  ret = GetOneByte(&ch);
  while (ret == 1) {
    switch(state)
    {
    case FIND_HEADA:
      if (ch == pMsg->headA)
        state = FIND_HEADB;
      break;

    case FIND_HEADB:
      if (ch == pMsg->headB)
        state = GET_LENGTH;
      else
        state = FIND_HEADA;
      break;

    case GET_LENGTH:
      pMsg->len = ch;
      index = 0;
      index_max = pMsg->len + (pMsg->buf - &pMsg->seq);
      state = GET_DATA;
      break;

    case GET_DATA:
      pBuf[index++] = ch;
      if (index >= index_max)
        state = GET_CRCA;
      break;

    case GET_CRCA:
      pMsg->crcA = ch;
      state = GET_CRCB;
      break;

    case GET_CRCB:
      pMsg->crcB = ch;
      crc = CalCRC16(&pMsg->len, index_max + sizeof(pMsg->len));
      state = FIND_HEADA;
      if (crc == (pMsg->crcA | (pMsg->crcB << 8))) {
        return 1;
      } else {
        return -1;
      }
      break;

    default:
      state = FIND_HEADA;
      break;
    }

    ret = GetOneByte(&ch);
  }

  return 0;
}
