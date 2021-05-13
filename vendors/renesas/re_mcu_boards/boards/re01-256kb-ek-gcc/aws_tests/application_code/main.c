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

#include <stdio.h>	//[RE01-test] Add from RX65N-RSK
#include <string.h>	//[RE01-test] Add from RX65N-RSK

/* Renesas. */
#include "serial_term_uart.h"	//[RE01-test] Add from RX65N-RSK

/* Test application include. */
#include "aws_test_runner.h"	//[RE01-test] Add from RX65N-RSK

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

/* FreeRTOS+TCP includes. */
// changed 2020/10 start
/////#include "FreeRTOS_IP.h"
// changed 2020/10 end

/* Demo includes */
//#include "aws_demo.h"	//[RE01-test] Delete for test
#include "aws_clientcredential.h"

// changed 2020/10 start
//#include "RE01_1500KB.h"
#include "RE01_256KB.h"
#include "r_core_cfg.h"
#include "r_system_api.h"
#include "r_lpm_api.h"
// changed 2020/10 end

#include "iot_wifi.h"	//[RE01-test] Add for test

#define mainLOGGING_TASK_STACK_SIZE         ( configMINIMAL_STACK_SIZE * 6 )
#define mainLOGGING_MESSAGE_QUEUE_LENGTH    ( 15 )
#define mainTEST_RUNNER_TASK_STACK_SIZE    ( configMINIMAL_STACK_SIZE * 8 )

/* The task delay for allowing the lower priority logging task to print out Wi-Fi
 * failure status before blocking indefinitely. */
#define mainLOGGING_WIFI_STATUS_DELAY       pdMS_TO_TICKS( 1000 )	//[RE01-test] Add for test

// added 2020/10 start
// LED SAMPLE
#define LED_SAMPLE_CODE           (0)    // 0 : disable, 1 : enable
// added 2020/10 end

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

// added 2020/10 start
// moved RE01_1500KB_DFP\main.c
/* Functions */
void NMI_Handler( void )  __attribute__ ((section(".ramfunc"))) ;     /* This is NMI Handler for User template*/
void LVD_for_EHC( void ) __attribute__ ((section(".ramfunc"))) ;   /* This is  User template code of EHC initialization in NMI Handler*/
// added 2020/10 end

static void prvWifiConnect( void );	//[RE01-test] Add for test

// added 2020/10 start
// LED SAMPLE
#if (LED_SAMPLE_CODE == 1)
void Make_Tasks(void);
void main_led0( void );
void main_led1( void );
void main_led2( void );
#endif
// added 2020/10 end

/*-----------------------------------------------------------*/

/**
 * @brief The application entry point from a power on reset is PowerON_Reset_PC()
 * in resetprg.c.
 */
void main( void )
{
    while(1)
    {
        vTaskDelay(10000);
    }
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
    prvMiscInitialization();

    if( SYSTEM_Init() == pdPASS )
    {
// changed 2020/10 start
#if 0
// changed 2020/10 end

        /* Initialise the RTOS's TCP/IP stack.  The tasks that use the network
        are created in the vApplicationIPNetworkEventHook() hook function
        below.  The hook function is called when the network connects. */
        FreeRTOS_IPInit( ucIPAddress,
                         ucNetMask,
                         ucGatewayAddress,
                         ucDNSServerAddress,
                         ucMACAddress );

        /* We should wait for the network to be up before we run any demos. */
        while( FreeRTOS_IsNetworkUp() == pdFALSE )
        {
            vTaskDelay(300);
        }
		FreeRTOS_printf( ( "The network is up and running\n" ) );

// changed 2020/10 start
#endif
// changed 2020/10 end

        /* Connect to the Wi-Fi before running the tests. */	//[RE01-test] Add from "amazon-freertos\vendors\vendor\boards\board\aws_tests\application_code\main.c"
        prvWifiConnect();										//[RE01-test] Add from "amazon-freertos\vendors\vendor\boards\board\aws_tests\application_code\main.c"

// added 2020/10 start
// LED SAMPLE
#if (LED_SAMPLE_CODE == 1)
		Make_Tasks();
#else
// added 2020/10 end

        /* Provision the device with AWS certificate and private key. */
        vDevModeKeyProvisioning();

        /* Run all demos. */
//        DEMO_RUNNER_RunDemos(); //[RE01-test] Delete for test

        //[RE01-test] Add from "amazon-freertos\vendors\vendor\boards\board\aws_tests\application_code\main.c" (start)
        vTaskDelay(10000);	// todo: this is renesas issue.
        /* Create the task to run tests. */
        xTaskCreate( TEST_RUNNER_RunTests_task,
                     "RunTests_task",
                     mainTEST_RUNNER_TASK_STACK_SIZE,
                     NULL,
                     tskIDLE_PRIORITY,
                     NULL );
        //[RE01-test] Add from "amazon-freertos\vendors\vendor\boards\board\aws_tests\application_code\main.c" (end)

// added 2020/10 start
#endif
// added 2020/10 end
    }
}

// added 2020/10 start
// moved RE01_1500KB_DFP\main.c
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
   SYSC->LVD1SR_b.DET = 0;
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
      if(0 != SYSC->LVD1SR_b.MON)
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
    SYSC->PRCR = 0xA503U;

     /*--------------------------------------------------------------------------
     * Start the LOCO operation
     *------------------------------------------------------------------------*/
    SYSC->LOCOCR = 0x00U;     /* Start LOCO */

    /*--------------------------------------------------------------------------
     * Wait the LOCO clock stabilization
     *------------------------------------------------------------------------*/
    while(0x00 != SYSC->LOCOCR)
    {
        /* Retry the writing */
        SYSC->LOCOCR = 0x00U;
    }

    /*--------------------------------------------------------------------------
     * Set the system clock source
     * b2-b0 : [   CKSEL] System clock source select
     *                      - [010] LOCO selection
     *------------------------------------------------------------------------*/
    SYSC->SCKSCR = 0x02U;     /* Clock source : LOCO */
    while(0x02U != SYSC->SCKSCR)
    {
        /* Retry the writing */
        SYSC->SCKSCR = 0x02U;
    }

    /**** Enable Stop module clock for DTC/DMAC */
    SYSC->MSTPCRA_b.MSTPA22 = 1U;     /* Stop module clock for DTC/DMAC */

    /* Enable EHC Charging Capacitor Quick Wake-up function */
    /* This is must need for initializing EHC Circuit when using EHC mode. */
    EHC->EHCCR1_b.QUICKMODE = 0U;

    while(1)
    {
        ;    /* loop */
    }
#endif /* SYSTEM_CFG_EHC_MODE == (1) */

} /*End of function of LVD_for_EHC() */

//[RE01_test] Add from "amazon-freertos\vendors\vendor\boards\board\aws_tests\application_code\main.c" (start)
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
//[RE01_test] Add from "amazon-freertos\vendors\vendor\boards\board\aws_tests\application_code\main.c" (end)

// added 2020/10 start
// LED SAMPLE
#if (LED_SAMPLE_CODE == 1)
void Make_Tasks(void)
{
    BaseType_t ret;

    /************** task creation ****************************/
    /* LED0 task. */
    ret = xTaskCreate(main_led0, "LED0_TASK", 512, NULL, 3, NULL);

    if (pdPASS != ret)
    {
        while(1)
        {
            /* Failed! Task can not be created. */
        }
    }

    /************** task creation ****************************/
    /* LED1 task. */
    ret = xTaskCreate(main_led1, "LED1_TASK", 512, NULL, 3, NULL);

    if (pdPASS != ret)
    {
        while(1)
        {
            /* Failed! Task can not be created. */
        }
    }

    /************** task creation ****************************/
    /* LED2 task. */
    ret = xTaskCreate(main_led2, "LED2_TASK", 512, NULL, 3, NULL);

    if (pdPASS != ret)
    {
        while(1)
        {
            /* Failed! Task can not be created. */
        }
    }


    /*set P210 and P410 output to high (LED off)*/
    /*set P007, 008, and 009 output to high (LED off)*/
    PORT0->PODR = 0x0380;

    /*Set Port direction control (set as input/output)*/
    /*1 means output, 0 means input*/
    /*set P007, 008, and 009 as output (1)*/
    PORT0->PDR = 0x0380;
}

void main_led0( void )
{
    for( ; ; )
    {
    	configPRINT_STRING("LED0 task.");
    	configPRINT_STRING( "\r\n" );

    	for (int i=0; i < 2; i++)
    	{
            /* Toggle the bit 9 of Port0 */
            /*only target bit (P009) is changed*/
            PORT0->PODR = (PORT0->PODR ^ 0x0200);

            R_SYS_SoftwareDelay(500, SYSTEM_DELAY_UNITS_MILLISECONDS);
    	}

        /*set P009 output to high (LED off)*/
        PORT0->PODR |= 0x0200;

        vTaskDelay( 1 );
    }
}

void main_led1( void )
{
    for( ; ; )
    {
    	configPRINT_STRING("LED1 task.");
    	configPRINT_STRING( "\r\n" );

    	for (int i=0; i < 2; i++)
    	{
            /* Toggle the bit 8 of Port0 */
            /*only target bit (P008) is changed*/

            R_SYS_SoftwareDelay(500, SYSTEM_DELAY_UNITS_MILLISECONDS);

            PORT0->PODR = (PORT0->PODR ^ 0x0100);

    	}

        /*set P008 output to high (LED off)*/
        PORT0->PODR |= 0x0100;

        vTaskDelay( 5 );
    }
}

void main_led2( void )
{
    for( ; ; )
    {
    	configPRINT_STRING("LED2 task.");
    	configPRINT_STRING( "\r\n" );

    	for (int i=0; i < 2; i++)
    	{
            /* Toggle the bit 7 of Port0 */
            /*only target bit (P007) is changed*/

            R_SYS_SoftwareDelay(750, SYSTEM_DELAY_UNITS_MILLISECONDS);

            PORT0->PODR = (PORT0->PODR ^ 0x0080);
    	}

        /*set P007 output to high (LED off)*/
    	PORT0->PODR |= 0x0080;

        vTaskDelay( 10 );
    }
}
#endif
// added 2020/10 end

// added 2020/10 end

/*-----------------------------------------------------------*/
