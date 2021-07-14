/*
Amazon FreeRTOS
Copyright (C) 2018 Amazon.com, Inc. or its affiliates.  All Rights Reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 http://aws.amazon.com/freertos
 http://www.FreeRTOS.org
*/

#include <stdio.h>
#include <string.h>

/* Renesas. */
#include "serial_term_uart.h"

/* Test application include. */
#include "aws_test_runner.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Version includes. */
#include "aws_application_version.h"

/* System init includes. */
#include "iot_system_init.h"

/* Logging includes. */
#include "iot_logging_task.h"

/* Key provisioning includes. */
#include "aws_dev_mode_key_provisioning.h"
#include "aws_clientcredential.h"

#include "RE01_1500KB.h"
#include "r_core_cfg.h"
#include "r_system_api.h"
#include "r_lpm_api.h"
#include "iot_wifi.h"

#define mainLOGGING_TASK_STACK_SIZE         ( configMINIMAL_STACK_SIZE * 6 )
#define mainLOGGING_MESSAGE_QUEUE_LENGTH    ( 15 )
#define mainTEST_RUNNER_TASK_STACK_SIZE    ( configMINIMAL_STACK_SIZE * 8 )

/* The task delay for allowing the lower priority logging task to print out Wi-Fi
 * failure status before blocking indefinitely. */
#define mainLOGGING_WIFI_STATUS_DELAY       pdMS_TO_TICKS( 1000 )

/* The MAC address array is not declared const as the MAC address will
normally be read from an EEPROM and not hard coded (in real deployed
applications).*/
static uint8_t ucMACAddress[ 6 ] =
{
    configMAC_ADDR0,
    configMAC_ADDR1,
    configMAC_ADDR2,
    configMAC_ADDR3,
    configMAC_ADDR4,
    configMAC_ADDR5
}; //XXX

/* Define the network addressing.  These parameters will be used if either
ipconfigUDE_DHCP is 0 or if ipconfigUSE_DHCP is 1 but DHCP auto configuration
failed. */
static const uint8_t ucIPAddress[ 4 ] =
{
    configIP_ADDR0,
    configIP_ADDR1,
    configIP_ADDR2,
    configIP_ADDR3
};
static const uint8_t ucNetMask[ 4 ] =
{
    configNET_MASK0,
    configNET_MASK1,
    configNET_MASK2,
    configNET_MASK3
};
static const uint8_t ucGatewayAddress[ 4 ] =
{
    configGATEWAY_ADDR0,
    configGATEWAY_ADDR1,
    configGATEWAY_ADDR2,
    configGATEWAY_ADDR3
};

/* The following is the address of an OpenDNS server. */
static const uint8_t ucDNSServerAddress[ 4 ] =
{
    configDNS_SERVER_ADDR0,
    configDNS_SERVER_ADDR1,
    configDNS_SERVER_ADDR2,
    configDNS_SERVER_ADDR3
};

/**
 * @brief Application task startup hook.
 */
void vApplicationDaemonTaskStartupHook( void );

/**
 * @brief Initializes the board.
 */
static void prvMiscInitialization( void );

/* Functions */
void NMI_Handler( void )  __attribute__ ((section(".ramfunc"))) ;     /* This is NMI Handler for User template*/
void LVD_for_EHC( void ) __attribute__ ((section(".ramfunc"))) ;   /* This is  User template code of EHC initialization in NMI Handler*/

static void prvWifiConnect( void );

/*-----------------------------------------------------------*/

/**
 * @brief The application entry point from a power on reset is PowerON_Reset_PC()
 * in resetprg.c.
 */
int main()
{
	R_SYS_CodeCopy();
	R_SYS_Initialize();
	R_LPM_Initialize();
	R_LPM_IOPowerSupplyModeSet((uint8_t)LPM_IOPOWER_SUPPLY_NONE);
	prvMiscInitialization();
	/* Call the kernel startup (should not return) */
	vTaskStartScheduler();
	return 0;
}
/*-----------------------------------------------------------*/

static void prvMiscInitialization( void )
{
    /* Initialize UART for serial terminal. */
    uart_config();

    /* Start logging task. */
    xLoggingTaskInitialize( mainLOGGING_TASK_STACK_SIZE,
                            tskIDLE_PRIORITY,
                            mainLOGGING_MESSAGE_QUEUE_LENGTH );
}
/*-----------------------------------------------------------*/

void vApplicationDaemonTaskStartupHook( void )
{

    if( SYSTEM_Init() == pdPASS )
    {
        /* Connect to the Wi-Fi before running the tests. */
        prvWifiConnect();
        /* Provision the device with AWS certificate and private key. */
        vDevModeKeyProvisioning();
        vTaskDelay(10000);	// todo: this is renesas issue.
        /* Create the task to run tests. */
        xTaskCreate( TEST_RUNNER_RunTests_task,
                     "RunTests_task",
                     mainTEST_RUNNER_TASK_STACK_SIZE,
                     NULL,
                     tskIDLE_PRIORITY,
                     NULL );
    }
}

/***********************************************************************************************************************
* Function Name: BoardInit
* Description  : Configure board initial setting.
*                This function is called by SystemInit() function in system_RE01_1500KB.c file.
*                This is reference to perform BoardInit process. Sample code of target is Evaluation Kit RE01_1500KB
*                 on Renesas. Please modify this function to suit your board.
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
void BoardInit(void)
{
    /*****   This function performs at beginning of start-up after released reset pin *****/
    /***** Please set pins here if your board is needed pins setting at the device start-up. ****/
    /*****   This function is suiting Evaluation Kit RE01_1500KB. Please change to your board pin setting *****/

    /* Handling of Unused ports (IOVCC domain) */
    /* PORT4 Setting: Evaluation Kit RE01_1500KB has DCDCs. Those are connect P404 and P405. */
    /* This perform to disable those output.                                     */
    /* Those are needed to enable when using EHC start up of this device. */
    /* Set P404 and P405 not to be used as DCDC_EN (output low: Disable) */

    /* PODR - Port Output Data
       b15-b0   PODR15 - PORD00     - Output Low Level */
    PORT4->PODR = 0x0000;

    /* PDR - Port Direction
       b15-b6   PDR15 - PRD06       - Input
       b5 -b4   PDR05 - PRD04       - Output
       b3 -b0   PDR03 - PRD00       - Input */
    PORT4->PDR  = 0x0030;

    /* Handling of Unused pors (AVCC1 domain) */

    /* PORT0 Setting */
    /* Set P009, P008 and P007 as LEDs (output high) */

    /* PODR - Port Output Data
       b15-b10  PODR15 - PORD10     - Output Low Level
       b9 -b7   PODR09 - PORD07     - Output High Level
       b6 -b0   PODR06 - PORD00     - Output Low Level */
    PORT0->PODR = 0x0380;

    /* PDR - Port Direction
       b15-b10  PDR15 - PRD10       - Input PORT
       b9 -b7   PDR09 - PRD07       - Output PORT
       b6 -b0   PDR06 - PRD00       - Input PORT */
    PORT0->PDR = 0x0380;

} /* End of function BoardInit */

/**********************************************************************************************************************
* Function Name: NMI_Handler
* Description  : NMI handler for User for template code when using EHC start-up.
*                This function is called when NMI interrupt is happen such as LVD, WDT and Oscillation stop detection.
*                This is must need to create when User uses Energy harvesting start-up(EHC).
*                (EHC is enable when SYSTEM_CFG_EHC_MODE of definition in r_core_cfg.h set "1".)
*                If use uses EHC, User must need to create process to initialize EHC circuit in this device
*                when detect voltage drop using LVD1.
*                This main.c is providing such a NMI Handler template code as default.
*
* Arguments    : none
* Return Value : none
**********************************************************************************************************************/
void NMI_Handler( void )
{
  volatile uint16_t f_status;
  /* Read status flag for NMI interrupts */
  f_status = ICU->NMISR;

  if(0x0004 == (f_status & 0x0004))
  {
    /* When detect LVD1 */
    LVD_for_EHC();
    return; /* This return is executed when LVD1 detection was misdetection by any noise. */
  }

} /* End of function NMI_Handler() */

/**********************************************************************************************************************
* Function Name: LVD_for_EHC
* Description  : This function performs EHC initialization when it checks No misdetection of LVD.
*                This function checks this LVD is misdetection or not.
*                When this is not misdetection, it performs initialization of EHC.
*
* Arguments    : none
* Return Value : -1 : Error : This LVD detection is misdetection.
*                -  : When Properly , this is perform while loop after cut of power supply.
**********************************************************************************************************************/
void LVD_for_EHC()
{
  volatile uint16_t i;
#if SYSTEM_CFG_EHC_MODE == (1)
   /* Disable protect for LVD */
   R_SYS_RegisterProtectDisable(SYSTEM_REG_PROTECT_LVD);
   /* Clear LVD1 detection flag */
   SYSTEM->LVD1SR_b.DET = 0;
   /* Clear LVD1 NMI interrpt flag */
   ICU->NMICLR_b.LVD1CLR = 1;
   while(ICU->NMISR_b.LVD1ST != 0);
   /* Enable protect for LVD */
   R_SYS_RegisterProtectEnable(SYSTEM_REG_PROTECT_LVD);
   /**************************************************************************/
   /* Write user's code if you have any process to do before device power off.*/
   /**************************************************************************/
   /* Wait 100m second for stabilizetion of noise */
   R_SYS_SoftwareDelay(100, SYSTEM_DELAY_UNITS_MILLISECONDS);
   /* Check if LVD detected due to noise */
   for(i = 0; i < 3 ; i++)
   {
      if(0 != SYSTEM->LVD1SR_b.MON)
      {
        /* This LVD detection is misdetection. */
        /* VCC level is higher than LVD1det level. */
        /* So, This detection is error by any noise */
          return ;
       }
    }

    /*--------------------------------------------------------------------------
     * Set the protect function of the register
     *------------------------------------------------------------------------*/
    SYSTEM->PRCR = 0xA503U;

     /*--------------------------------------------------------------------------
     * Start the LOCO operation
     *------------------------------------------------------------------------*/
    SYSTEM->LOCOCR = 0x00U;     /* Start LOCO */

    /*--------------------------------------------------------------------------
     * Wait the LOCO clock stabilization
     *------------------------------------------------------------------------*/
    while(0x00 != SYSTEM->LOCOCR)
    {
        /* Retry the writing */
        SYSTEM->LOCOCR = 0x00U;
    }

    /*--------------------------------------------------------------------------
     * Set the system clock source
     * b2-b0 : [   CKSEL] System clock source select
     *                      - [010] LOCO selection
     *------------------------------------------------------------------------*/
    SYSTEM->SCKSCR = 0x02U;     /* Clock source : LOCO */
    while(0x02U != SYSTEM->SCKSCR)
    {
        /* Retry the writing */
        SYSTEM->SCKSCR = 0x02U;
    }

    /**** Enable Stop module clock for DTC/DMAC */
    SYSTEM->MSTPCRA_b.MSTPA22 = 1U;     /* Stop module clock for DTC/DMAC */

    /* Enable EHC Charging Capacitor Quick Wake-up function */
    /* This is must need for initializing EHC Circuit when using EHC mode. */
    SYSTEM->EHCCR1_b.QUICKMODE = 0U;

    while(1)
    {
        ;    /* loop */
    }
#endif /* SYSTEM_CFG_EHC_MODE == (1) */

} /*End of function of LVD_for_EHC() */

void prvWifiConnect( void )
{
    /* FIX ME: Delete surrounding compiler directives when the Wi-Fi library is ported. */
    #if 1
        WIFINetworkParams_t xNetworkParams;
        WIFIReturnCode_t xWifiStatus;
        uint8_t ucTempIp[4] = { 0 };

        xWifiStatus = WIFI_On();

        if( xWifiStatus == eWiFiSuccess )
        {
            configPRINTF( ( "Wi-Fi module initialized. Connecting to AP...\r\n" ) );
        }
        else
        {
            configPRINTF( ( "Wi-Fi module failed to initialize.\r\n" ) );

            /* Delay to allow the lower priority logging task to print the above status.
             * The while loop below will block the above printing. */
            vTaskDelay( mainLOGGING_WIFI_STATUS_DELAY );

            while( 1 )
            {
            }
        }
		#if 0
        /* Setup parameters. */
        xNetworkParams.pcSSID = clientcredentialWIFI_SSID;
        xNetworkParams.ucSSIDLength = sizeof( clientcredentialWIFI_SSID );
        xNetworkParams.pcPassword = clientcredentialWIFI_PASSWORD;
        xNetworkParams.ucPasswordLength = sizeof( clientcredentialWIFI_PASSWORD );
        xNetworkParams.xSecurity = clientcredentialWIFI_SECURITY;
        xNetworkParams.cChannel = 0;

        xWifiStatus = WIFI_ConnectAP( &( xNetworkParams ) );

        if( xWifiStatus == eWiFiSuccess )
        {
            configPRINTF( ( "Wi-Fi Connected to AP. Creating tasks which use network...\r\n" ) );

            xWifiStatus = WIFI_GetIP( ucTempIp );
            if ( eWiFiSuccess == xWifiStatus )
            {
                configPRINTF( ( "IP Address acquired %d.%d.%d.%d\r\n",
                                ucTempIp[ 0 ], ucTempIp[ 1 ], ucTempIp[ 2 ], ucTempIp[ 3 ] ) );
            }
        }
        else
        {
            configPRINTF( ( "Wi-Fi failed to connect to AP.\r\n" ) );

            /* Delay to allow the lower priority logging task to print the above status.
             * The while loop below will block the above printing. */
            TaskDelay( mainLOGGING_WIFI_STATUS_DELAY );

            while( 1 )
            {
            }
        }
		#endif /* if 0 */

	#endif /* if 0 */
}
