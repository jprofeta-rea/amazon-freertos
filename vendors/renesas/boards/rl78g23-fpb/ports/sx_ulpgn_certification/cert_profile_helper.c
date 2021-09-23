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
 * File Name    : certificate_profile.c
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

#include <stdint.h>
#include <stddef.h>
#include "cert_profile_helper.h"

#include "aws_clientcredential.h"
#include "iot_secure_sockets.h"
#include "r_wifi_sx_ulpgn_if.h"

/**********************************************************************************************************************
 Macro definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Local Typedef definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Exported global variables
 *********************************************************************************************************************/

#if defined(ENABLE_UNIT_TESTS) || defined(FREERTOS_ENABLE_UNIT_TESTS)
extern const uint8_t sharkSslCAList_PC;
extern const uint32_t sharkSslCAList_PCLength;
extern const uint8_t sharkSslCAList;
extern const uint32_t sharkSslCAListLength;
extern const uint8_t sharkSslRSACert_PC;
extern const uint32_t sharkSslRSACert_PCLength;
extern const uint8_t sharkSslRSACert;
extern const uint32_t sharkSslRSACertLength;
#endif

/**********************************************************************************************************************
 Private (static) variables and functions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: prvSetCertificateProfile
 * Description  : Sets the host address to the certificate profile that matches the specified Id.
 * Arguments    : none
 * Return Value : none
 *********************************************************************************************************************/
void prvSetCertificateProfile(void)
{
    uint32_t ipaddress;

    ipaddress = SOCKETS_inet_addr_quick(CONFIG_CERT_PROFILE_ECHO_SERVER_ADDR3, CONFIG_CERT_PROFILE_ECHO_SERVER_ADDR2,
                                        CONFIG_CERT_PROFILE_ECHO_SERVER_ADDR1, CONFIG_CERT_PROFILE_ECHO_SERVER_ADDR0);
    R_WIFI_SX_ULPGN_SetCertificateProfile (0, ipaddress, NULL);
    R_WIFI_SX_ULPGN_SetCertificateProfile (1, 0, (const char*) clientcredentialMQTT_BROKER_ENDPOINT);
}
/**********************************************************************************************************************
 * End of Function prvSetCertificateProfile
 *********************************************************************************************************************/

#if defined(ENABLE_UNIT_TESTS) || defined(FREERTOS_ENABLE_UNIT_TESTS)

/**********************************************************************************************************************
 * Function Name: prvFakeSetCertificateProfile
 * Description  : Set the fake certificate profile.
 * Arguments    : none
 * Return Value : none
 *********************************************************************************************************************/
void prvFakeSetCertificateProfile(void)
{
    uint32_t ipaddress;

    ipaddress = SOCKETS_inet_addr_quick(CONFIG_CERT_PROFILE_ECHO_SERVER_ADDR3, CONFIG_CERT_PROFILE_ECHO_SERVER_ADDR2,
                                        CONFIG_CERT_PROFILE_ECHO_SERVER_ADDR1, CONFIG_CERT_PROFILE_ECHO_SERVER_ADDR0);
    R_WIFI_SX_ULPGN_SetCertificateProfile (1, ipaddress, NULL);
    R_WIFI_SX_ULPGN_SetCertificateProfile (0, 0, (const char*) clientcredentialMQTT_BROKER_ENDPOINT);
}
/**********************************************************************************************************************
 * End of Function prvFakeSetCertificateProfile
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: prvWifiSetCertification
 * Description  :
 * Arguments    : none
 * Return Value : none
 *********************************************************************************************************************/
void prvWifiSetCertification(void)
{
    wifi_certificate_infomation_t certificate_information;
    wifi_certificate_infomation_t *pcertificate_information;
    pcertificate_information = (wifi_certificate_infomation_t*) &certificate_information;

    /* Get Initial Server Certificate Information */
    R_WIFI_SX_ULPGN_GetServerCertificate (pcertificate_information);

    /* Erase All Certificate */
    R_WIFI_SX_ULPGN_EraseAllServerCertificate ();
    /* Get Server Certificate Information */
    R_WIFI_SX_ULPGN_GetServerCertificate (pcertificate_information);

    /* Write 2Set Certificate */
    /* Secure Echo Server Certificate cert0.crt(sharkSslRSACert_PC),calist0.crt(sharkSslCAList_PC) */
    /* AWS Broker         Certificate cert1.crt(sharkSslRSACert)   ,calist1.crt(sharkSslCAList)    */
    R_WIFI_SX_ULPGN_WriteServerCertificate (0, 1, (const uint8_t*) &sharkSslRSACert_PC,
                                            (uint32_t) sharkSslRSACert_PCLength);
    R_WIFI_SX_ULPGN_WriteServerCertificate (0, 0, (const uint8_t*) &sharkSslCAList_PC,
                                            (uint32_t) sharkSslCAList_PCLength);
    R_WIFI_SX_ULPGN_WriteServerCertificate (1, 1, (const uint8_t*) &sharkSslRSACert, (uint32_t) sharkSslRSACertLength);
    R_WIFI_SX_ULPGN_WriteServerCertificate (1, 0, (const uint8_t*) &sharkSslCAList, (uint32_t) sharkSslCAListLength);

}
/**********************************************************************************************************************
 * End of Function prvWifiSetCertification
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: prvEraseAllCertificateFile
 * Description  : none
 * Arguments    : none
 * Return Value : none
 *********************************************************************************************************************/
void prvEraseAllCertificateFile(void)
{
    R_WIFI_SX_ULPGN_EraseAllServerCertificate ();
}
/**********************************************************************************************************************
 * End of Function prvEraseAllCertificateFile
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: prvWriteAllCertificateFile
 * Description  : none
 * Arguments    : none
 * Return Value : none
 *********************************************************************************************************************/
void prvWriteAllCertificateFile(void)
{
    /* Write 2Set Certificate */
    /* Secure Echo Server Certificate cert0.crt(sharkSslRSACert_PC),calist0.crt(sharkSslCAList_PC) */
    R_WIFI_SX_ULPGN_WriteServerCertificate (0, 1, (const uint8_t*) &sharkSslRSACert_PC,
                                            (uint32_t) sharkSslRSACert_PCLength);
    R_WIFI_SX_ULPGN_WriteServerCertificate (0, 0, (const uint8_t*) &sharkSslCAList_PC,
                                            (uint32_t) sharkSslCAList_PCLength);
    /* AWS Broker         Certificate cert1.crt(sharkSslRSACert)   ,calist1.crt(sharkSslCAList)    */
    R_WIFI_SX_ULPGN_WriteServerCertificate (1, 1, (const uint8_t*) &sharkSslRSACert, (uint32_t) sharkSslRSACertLength);
    R_WIFI_SX_ULPGN_WriteServerCertificate (1, 0, (const uint8_t*) &sharkSslCAList, (uint32_t) sharkSslCAListLength);
}
/**********************************************************************************************************************
 * End of Function prvWriteAllCertificateFile
 *********************************************************************************************************************/

#endif /* defined(ENABLE_UNIT_TESTS) || defined(FREERTOS_ENABLE_UNIT_TESTS) */
