 /**
  *  SharkSSLParseCAList.  Build 3413.
  *  Copyright (c) 2014 Real Time Logic.
  */

#include "platform.h"

#if defined(__CCRL__)
#pragma section const const_cert
#endif

/*
 * @todo : Set SharkSSL-encoded CA list for AWS IoT Broker.
 */
const uint8_t sharkSslCAList[] = { 0x00 };

const uint32_t sharkSslCAListLength = sizeof(sharkSslCAList);

#if defined(__CCRL__)
#pragma section
#endif
