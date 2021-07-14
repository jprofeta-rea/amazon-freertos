/*
 * Amazon FreeRTOS Wi-Fi V1.0.0
 * Copyright (C) 2018 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

/**
 * @file aws_wifi.c
 * @brief Wi-Fi Interface.
 */
#include <stdio.h>
#include <string.h>
/* FreeRTOS includes. */
#include "FreeRTOS.h"

/* Socket and WiFi interface includes. */
#include "iot_wifi.h"

/* WiFi configuration includes. */
#include "aws_wifi_config.h"

/* WiFi configuration includes. */
#include "platform.h"
#include "bg96_driver.h"
#include "FreeRTOSIPConfig.h"

/**
 * @brief Wi-Fi initialization status.
 */
static BaseType_t xWIFIInitDone;
static uint32_t prvConvertSecurityFromSilexAT( WIFISecurity_t xSecurity );

static uint32_t prvConvertSecurityFromSilexAT( WIFISecurity_t xSecurity )
{
    uint32_t xConvertedSecurityType = WIFI_SECURITY_UNDEFINED;

    switch( xSecurity )
    {
        case eWiFiSecurityOpen:
            xConvertedSecurityType = WIFI_SECURITY_OPEN;
            break;

        case eWiFiSecurityWEP:
            xConvertedSecurityType = WIFI_SECURITY_WEP;
            break;

        case eWiFiSecurityWPA:
            xConvertedSecurityType = WIFI_SECURITY_WPA;
            break;

        case eWiFiSecurityWPA2:
            xConvertedSecurityType = WIFI_SECURITY_WPA2;
            break;

        case eWiFiSecurityNotSupported:
            xConvertedSecurityType = WIFI_SECURITY_UNDEFINED;
            break;
    }

    return xConvertedSecurityType;
}

/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_On( void )
{
    /* FIX ME. */
    WIFIReturnCode_t xRetVal = eWiFiFailure;

    /* One time Wi-Fi initialization */
    if( xWIFIInitDone == pdFALSE )
    {
        /* Wi-Fi init done*/
        xWIFIInitDone = pdTRUE;
    }
    if(0 == bg96_wifi_init())
    {
    	xRetVal = eWiFiSuccess;
    }

	return xRetVal;
}
/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_Off( void )
{
    R_CELLULAR_BG96_Close();

    return eWiFiSuccess;
}
/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_ConnectAP( const WIFINetworkParams_t * const pxNetworkParams )
{
	wifi_err_t ret;
	uint8_t dhcp_enable;
	uint32_t convert_security;
	convert_security = prvConvertSecurityFromSilexAT(pxNetworkParams->xSecurity);

	return eWiFiSuccess;
}
/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_Disconnect( void )
{
    R_CELLULAR_BG96_Disconnect();
    return eWiFiSuccess;
}
/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_Reset( void )
{
	WIFIReturnCode_t ret;
    WIFI_Off();
    ret = WIFI_On();
    return ret;
}
/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_Scan( WIFIScanResult_t * pxBuffer,
                            uint8_t ucNumNetworks )
{
    WIFIReturnCode_t result = eWiFiFailure;
    wifi_err_t ret;
    uint32_t list;

	result = eWiFiSuccess;
    return result;
}
/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_SetMode( WIFIDeviceMode_t xDeviceMode )
{
    return eWiFiNotSupported;
}
/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_GetMode( WIFIDeviceMode_t * pxDeviceMode )
{
    return eWiFiNotSupported;
}
/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_NetworkAdd( const WIFINetworkProfile_t * const pxNetworkProfile,
                                  uint16_t * pusIndex )
{
    return eWiFiNotSupported;
}
/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_NetworkGet( WIFINetworkProfile_t * pxNetworkProfile,
                                  uint16_t usIndex )
{
    return eWiFiNotSupported;
}
/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_NetworkDelete( uint16_t usIndex )
{
    return eWiFiNotSupported;
}
/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_Ping( uint8_t * pucIPAddr,
                            uint16_t usCount,
                            uint32_t ulIntervalMS )
{
    WIFIReturnCode_t result = eWiFiFailure;
    wifi_err_t ret;
    uint32_t ipaddress;
    if(NULL == pucIPAddr)
    {
    	return eWiFiFailure;
    }
	ipaddress = ((uint32_t)pucIPAddr[0] << 24) | ((uint32_t)pucIPAddr[1] << 16) | ((uint32_t)pucIPAddr[2] << 8) | ((uint32_t)pucIPAddr[3]);
	result = eWiFiSuccess;
    return result;
}
/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_GetIPInfo( WIFIIPConfiguration_t * pxIPConfig )
{
	return eWiFiSuccess;
}
/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_GetMAC( uint8_t * pucMac )
{
    return eWiFiSuccess;
}
/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_GetHostIP( char * pcHost,
                                 uint8_t * pucIPAddr )
{
    return eWiFiSuccess;
}
/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_StartAP( void )
{
    return eWiFiNotSupported;
}
/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_StopAP( void )
{
    return eWiFiNotSupported;
}
/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_ConfigureAP( const WIFINetworkParams_t * const pxNetworkParams )
{
    return eWiFiNotSupported;
}
/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_SetPMMode( WIFIPMMode_t xPMModeType,
                                 const void * pvOptionValue )
{
    return eWiFiNotSupported;
}
/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_GetPMMode( WIFIPMMode_t * pxPMModeType,
                                 void * pvOptionValue )
{
    return eWiFiNotSupported;
}
/*-----------------------------------------------------------*/

BaseType_t WIFI_IsConnected( const WIFINetworkParams_t * pxNetworkParams )
{
	BaseType_t ret = pdFALSE;

	if(0 ==  R_CELLULAR_BG96_IsConnected())
	{
		ret = pdTRUE;
	}
	return ret;
}
WIFIReturnCode_t WIFI_RegisterEvent( WIFIEventType_t xEventType, WIFIEventHandler_t xHandler  )
{
    return eWiFiNotSupported;
}
