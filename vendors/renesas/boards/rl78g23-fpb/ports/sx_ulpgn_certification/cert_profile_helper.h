/**********************************************************************************************************************
 * DISCLAIMER
 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
 * other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
 * applicable laws, including copyright laws.
 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
 * EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
 * SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO
 * THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
 * this software. By using this software, you agree to the additional terms and conditions found by accessing the
 * following link:
 * http://www.renesas.com/disclaimer
 *
 * Copyright (C) 2021 Renesas Electronics Corporation. All rights reserved.
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * File Name    : cert_profile_helper.h
 * Version      : 1.0
 * Description  : .
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description
 *         : 01.01.2018 1.00     First Release
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Includes   <System Includes> , "Project Includes"
 *********************************************************************************************************************/

#if defined(ENABLE_UNIT_TESTS) || defined(FREERTOS_ENABLE_UNIT_TESTS)
#include "aws_test_tcp.h"
#else
#include "FreeRTOSConfig.h"
#endif

/**********************************************************************************************************************
 Macro definitions
 *********************************************************************************************************************/

#ifndef CERTIFICATE_PROFILE_H_
#define CERTIFICATE_PROFILE_H_

/**********************************************************************************************************************
 Global Typedef definitions
 *********************************************************************************************************************/

#if defined(ENABLE_UNIT_TESTS) || defined(FREERTOS_ENABLE_UNIT_TESTS)
#define CONFIG_CERT_PROFILE_ECHO_SERVER_ADDR0       (tcptestECHO_SERVER_TLS_ADDR0)
#define CONFIG_CERT_PROFILE_ECHO_SERVER_ADDR1       (tcptestECHO_SERVER_TLS_ADDR1)
#define CONFIG_CERT_PROFILE_ECHO_SERVER_ADDR2       (tcptestECHO_SERVER_TLS_ADDR2)
#define CONFIG_CERT_PROFILE_ECHO_SERVER_ADDR3       (tcptestECHO_SERVER_TLS_ADDR3)
#else
#define CONFIG_CERT_PROFILE_ECHO_SERVER_ADDR0       (configECHO_SERVER_ADDR0)
#define CONFIG_CERT_PROFILE_ECHO_SERVER_ADDR1       (configECHO_SERVER_ADDR1)
#define CONFIG_CERT_PROFILE_ECHO_SERVER_ADDR2       (configECHO_SERVER_ADDR2)
#define CONFIG_CERT_PROFILE_ECHO_SERVER_ADDR3       (configECHO_SERVER_ADDR3)
#endif

/**********************************************************************************************************************
 External global variables
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Exported global functions
 *********************************************************************************************************************/

void prvSetCertificateProfile(void);
#if defined(ENABLE_UNIT_TESTS) || defined(FREERTOS_ENABLE_UNIT_TESTS)
void prvFakeSetCertificateProfile(void);
void prvWifiSetCertification(void);
#endif

#endif /* CERTIFICATE_PROFILE_H_ */
