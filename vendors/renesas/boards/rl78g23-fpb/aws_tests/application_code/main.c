/*
 * FreeRTOS V1.1.4
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Test includes */
#include "aws_test_runner.h"

/* AWS library includes. */
#include "iot_system_init.h"
#include "iot_logging_task.h"
#include "iot_wifi.h"
#include "aws_clientcredential.h"
#include "aws_dev_mode_key_provisioning.h"

#include "iot_clock.h"
#include "serial_term_uart.h"
#include "cert_profile_helper.h"

/* Logging Task Defines. */
#define mainLOGGING_MESSAGE_QUEUE_LENGTH    ( 15 )
#define mainLOGGING_TASK_STACK_SIZE         ( configMINIMAL_STACK_SIZE * 10)//8 )

/* Unit test defines. */
#define mainTEST_RUNNER_TASK_STACK_SIZE     ( configMINIMAL_STACK_SIZE * 50 )

/* Use connect to Wi-Fi access point. */
#define _NM_PARAMS( networkType, networkState )    ( ( ( uint32_t ) networkType ) << 16 | ( ( uint16_t ) networkState ) )
#define _NM_GET_NETWORK_TYPE( params )             ( ( uint32_t ) ( ( params ) >> 16 ) & 0x0000FFFFUL )
#define _NM_GET_NETWORK_STATE( params )            ( ( AwsIotNetworkState_t ) ( ( params ) & 0x0000FFFFUL ) )
#define _NM_WIFI_CONNECTION_RETRY_INTERVAL_MS    ( 1000 )
#define _NM_WIFI_CONNECTION_RETRIES              ( 10 )

/* The task delay for allowing the lower priority logging task to print out Wi-Fi 
 * failure status before blocking indefinitely. */
#define mainLOGGING_WIFI_STATUS_DELAY       pdMS_TO_TICKS( 1000 )

/* The name of the devices for xApplicationDNSQueryHook. */
//#define mainDEVICE_NICK_NAME				"vendor_demo" /* FIX ME.*/
#define mainDEVICE_NICK_NAME                "RenesasRL78_test" /* FIX ME.*/

/* Static arrays for FreeRTOS-Plus-TCP stack initialization for Ethernet network 
 * connections are declared below. If you are using an Ethernet connection on your MCU 
 * device it is recommended to use the FreeRTOS+TCP stack. The default values are 
 * defined in FreeRTOSConfig.h. */


/**
 * @brief Application task startup hook for applications using Wi-Fi. If you are not 
 * using Wi-Fi, then start network dependent applications in the vApplicationIPNetorkEventHook
 * function. If you are not using Wi-Fi, this hook can be disabled by setting 
 * configUSE_DAEMON_TASK_STARTUP_HOOK to 0.
 */
void vApplicationDaemonTaskStartupHook( void );

/**
 * @brief Application IP network event hook called by the FreeRTOS+TCP stack for
 * applications using Ethernet. If you are not using Ethernet and the FreeRTOS+TCP stack,
 * start network dependent applications in vApplicationDaemonTaskStartupHook after the
 * network status is up.
 */

/**
 * @brief Connects to Wi-Fi.
 */
static bool _wifiEnable( void );

/**
 * @brief Initializes the board.
 */
static void prvMiscInitialization( void );

/*-----------------------------------------------------------*/
char xtaskList[512];

void main_task( void *pvParameters)
{
    P5 = 0x00U;
    PM5 = 0xFEU;

    while(1)
    {
        vTaskDelay(10000);
        P5_bit.no0 ^= 1;
    }
}

/*-----------------------------------------------------------*/

static void prvMiscInitialization( void )
{
    /* FIX ME: Perform any hardware initializations, that don't require the RTOS to be 
     * running, here.
     */
    uart_config();
    configPRINT_STRING(("Hello World.\r\n"));

    /*  */
    prvSetCertificateProfile();

    /* Start logging task. */
    xLoggingTaskInitialize( mainLOGGING_TASK_STACK_SIZE,
                            tskIDLE_PRIORITY,
                            mainLOGGING_MESSAGE_QUEUE_LENGTH );
}
/*-----------------------------------------------------------*/

void vApplicationDaemonTaskStartupHook( void )
{
    /* FIX ME: Perform any hardware initialization, that require the RTOS to be
     * running, here. */
    
    prvMiscInitialization();

    /* FIX ME: If your MCU is using Wi-Fi, delete surrounding compiler directives to 
     * enable the unit tests and after MQTT, Bufferpool, and Secure Sockets libraries 
     * have been imported into the project. If you are not using Wi-Fi, see the 
     * vApplicationIPNetworkEventHook function. */
#if 1
    if (SYSTEM_Init () == pdPASS)
    {
        /* Connect to the Wi-Fi before running the tests. */
        if (false == _wifiEnable ())
        {
            printf("wifi initilaize failed.\r\n");
            while(1)
            {
                portNOP();
            };
        };

        /* Provision the device with AWS certificate and private key. */
//        vDevModeKeyProvisioning();

        /* Create the task to run unit tests. */
        xTaskCreate (TEST_RUNNER_RunTests_task, "RunTests_task",
                     mainTEST_RUNNER_TASK_STACK_SIZE,
                     NULL,
                     tskIDLE_PRIORITY,
                     NULL);
    }
#endif /* if 0 */
}
/*-----------------------------------------------------------*/

// RX65N Cloud Kit 20200923 -->>
static bool _wifiConnectAccessPoint( void )
{
    bool ret = false;
    size_t xSSIDLength, xPasswordLength;
    static WIFINetworkParams_t xNetworkParams = { 0 };

    /* Setup WiFi parameters to connect to access point. */
    if( clientcredentialWIFI_SSID != NULL )
    {
        xSSIDLength = strlen( clientcredentialWIFI_SSID );
        if( ( xSSIDLength > 0 ) && ( xSSIDLength <= wificonfigMAX_SSID_LEN ) )
        {
            xNetworkParams.ucSSIDLength = xSSIDLength;
            memcpy( xNetworkParams.ucSSID, clientcredentialWIFI_SSID, xSSIDLength );
        }
        else
        {
            configPRINTF(( "Invalid WiFi SSID configured, empty or exceeds allowable length %d.\n", wificonfigMAX_SSID_LEN ));
        }
    }
    else
    {
        configPRINTF(( "WiFi SSID is not configured.\n" ));
    }

    xNetworkParams.xSecurity = clientcredentialWIFI_SECURITY;
    if( clientcredentialWIFI_SECURITY == eWiFiSecurityWPA2 )
    {
        if( clientcredentialWIFI_PASSWORD != NULL )
        {
            xPasswordLength = strlen( clientcredentialWIFI_PASSWORD );
            if( ( xPasswordLength > 0 ) && ( xSSIDLength <= wificonfigMAX_PASSPHRASE_LEN ) )
            {
                xNetworkParams.xPassword.xWPA.ucLength = xPasswordLength;
                memcpy( xNetworkParams.xPassword.xWPA.cPassphrase, clientcredentialWIFI_PASSWORD, xPasswordLength );
            }
            else
            {
                configPRINTF(( "Invalid WiFi password configured, empty password or exceeds allowable password length %d.\n", wificonfigMAX_PASSPHRASE_LEN ));
            }
        }
        else
        {
            configPRINTF(( "WiFi password is not configured.\n" ));
        }
    }
    xNetworkParams.ucChannel = 0;

    uint32_t numRetries = _NM_WIFI_CONNECTION_RETRIES;
    uint32_t delayMilliseconds = _NM_WIFI_CONNECTION_RETRY_INTERVAL_MS;


    /* Try to connect to wifi access point with retry and exponential delay */
    do
    {
        if( WIFI_ConnectAP( &( xNetworkParams ) ) == eWiFiSuccess )
        {
            ret = true;
            break;
        }
        else
        {
            if( numRetries > 0 )
            {
                IotClock_SleepMs( delayMilliseconds );
                delayMilliseconds = delayMilliseconds * 2;
            }
        }
    } while( numRetries-- > 0 );

    return ret;
}

static bool _wifiEnable( void )
{
    bool ret = true;

    if( WIFI_On() != eWiFiSuccess )
    {
        ret = false;
    }

//    prvWifiSetCertification();

    #if ( IOT_BLE_ENABLE_WIFI_PROVISIONING == 0 )
        if( ret == true )
        {
            ret = _wifiConnectAccessPoint();
        }
    #else
        if( ret == true )
        {
            if( IotBleWifiProv_Init() != pdTRUE )
            {
                ret = false;
            }
        }

        if( ret == true )
        {
            if( xWiFiConnectTaskInitialize() != pdTRUE )
            {
                ret = false;
            }
        }
    #endif /* if ( IOT_BLE_ENABLE_WIFI_PROVISIONING == 0 ) */

    return ret;
}

/*-----------------------------------------------------------*/

