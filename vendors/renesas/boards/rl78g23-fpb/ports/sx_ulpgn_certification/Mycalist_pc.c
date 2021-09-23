 /**
  *  SharkSSLParseCAList.  Build 3413.
  *  Copyright (c) 2014 Real Time Logic.
  */

#include "platform.h"

#if defined(__CCRL__)
#pragma section const const_cert
#endif

/*
 * @todo : Set SharkSSL-encoded CA list for echo server.
 */
const uint8_t sharkSslCAList_PC[] = { 0x00 };

const uint32_t sharkSslCAList_PCLength = sizeof(sharkSslCAList_PC);

#if defined(__CCRL__)
#pragma section
#endif
