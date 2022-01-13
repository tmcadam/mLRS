//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// FRAME TYPES & DEFS
//*******************************************************
#ifndef FRAME_TYPES_H
#define FRAME_TYPES_H
#pragma once


#include <inttypes.h>


#ifndef PACKED
#  define PACKED(__Declaration__)  __Declaration__ __attribute__((packed)) //that's for __GNUC__
#endif


//-------------------------------------------------------
// common types
//-------------------------------------------------------

typedef enum {
  CHANNEL_ORDER_AETR = 0, // most common
  CHANNEL_ORDER_TAER, // spektrum/graupner/jr
  CHANNEL_ORDER_ETAR,
} CHANNEL_ORDER_ENUM;


typedef enum {
  ANTENNA_1 = 0x00,
  ANTENNA_2 = 0x01,
} ANTENNA_ENUM;


//-------------------------------------------------------
// frames types only
//-------------------------------------------------------

typedef struct
{
  uint8_t seq_no;
  uint8_t ack;
  uint8_t antenna;
  int8_t rssi;
  int8_t snr;
  uint8_t LQ;
  uint8_t LQ_rc_data;
} tFrameStats;


typedef enum {
  FRAME_TYPE_TX = 0x00,
  FRAME_TYPE_RX = 0x01,
} FRAME_TYPE_ENUM;


#define FRAME_HEADER_LEN        7
#define FRAME_TX_RCDATA1_LEN    6
#define FRAME_TX_RCDATA2_LEN    10
#define FRAME_TX_PAYLOAD_LEN    64 // 82 - 10-6(rcdata) - 2(crc) = 64
#define FRAME_RX_PAYLOAD_LEN    82


PACKED(
typedef struct
{
  uint8_t seq_no:3;
  uint8_t ack:1;
  uint8_t frame_type:4;
  uint32_t antenna:1;
  uint32_t rssi_u7:7;
  uint32_t LQ:7;
  uint32_t LQ_rc_data:7; // not used in TX frame, 127 = invalid
  uint32_t spare:3;
  uint32_t payload_len:7;
}) tFrameStatus;


//-- Tx Frames ----------

#define RC_DATE_LEN  18


typedef struct
{
  uint16_t ch[RC_DATE_LEN]; // 0 .. 1024 .. 2047, 11 bits
} tRcData;


PACKED(
typedef struct
{
  uint16_t ch0  : 11; // 0 .. 1024 .. 2047, 11 bits
  uint16_t ch1  : 11;
  uint16_t ch2  : 11;
  uint16_t ch3  : 11;
  uint16_t ch14 :  1; // 0..1, 1 bit
  uint16_t ch15 :  1;
  uint16_t ch16 :  1;
  uint16_t ch17 :  1;
}) tFrameRcData1; // 6 bytes


PACKED(
typedef struct
{
  uint8_t ch[FRAME_TX_RCDATA2_LEN]; // 0 .. 128 .. 255, 8 bits
}) tFrameRcData2;


PACKED(
typedef struct
{
  uint16_t sync_word; // 2 bytes
  tFrameStatus status; // 5 bytes
  tFrameRcData1 rc1; // 6 bytes
  uint16_t crc1;
  tFrameRcData2 rc2; // 10 bytes
  uint8_t payload[64]; // = FRAME_TX_PAYLOAD_LEN
  uint16_t crc;
}) tTxFrame; // 91 bytes


//-- Rx Frames ----------

PACKED(
typedef struct
{
  uint16_t sync_word; // 2 bytes
  tFrameStatus status; // 5 bytes
  uint8_t payload[82]; // = FRAME_RX_PAYLOAD_LEN
  uint16_t crc;
}) tRxFrame; // 91 bytes



#endif // FRAME_TYPES_H
