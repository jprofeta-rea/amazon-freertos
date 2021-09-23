 /**
  *  SharkSSLParseCert.  Build 3370.
  *  Copyright (c) 2014 Real Time Logic.
  */

#include "platform.h"

#if defined(__CCRL__)
#pragma section const const_cert
#endif

/*
 * @todo : Set SharkSSL-encoded client certificate for AWS IoT Broker.
 */
const uint8_t sharkSslRSACert[] = { 0x00 };

const uint32_t sharkSslRSACertLength = sizeof(sharkSslRSACert);

#if defined(__CCRL__)
#pragma section
#endif
