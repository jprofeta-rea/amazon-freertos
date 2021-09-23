 /**
  *  SharkSSLParseCert.  Build 3370.
  *  Copyright (c) 2014 Real Time Logic.
  */

#include "platform.h"

#if defined(__CCRL__)
#pragma section const const_cert
#endif

/*
 * @todo : Set SharkSSL-encoded client certificate for echo server.
 */
const uint8_t sharkSslRSACert_PC[] = { 0x00 };

const uint32_t sharkSslRSACert_PCLength = sizeof(sharkSslRSACert_PC);

#if defined(__CCRL__)
#pragma section
#endif
