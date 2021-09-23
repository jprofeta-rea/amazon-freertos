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
 * File Name    : freertos_start.c
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
#include <stddef.h>
#include "FreeRTOS.h"

#include "freertos_start.h"
#include "r_smc_entry.h"

/**********************************************************************************************************************
 Macro definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Local Typedef definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Exported global variables
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Private (static) variables and functions
 *********************************************************************************************************************/

/* Hook functions used by FreeRTOS. */


/* FreeRTOS's processing before start the kernel. */
void Processing_Before_Start_Kernel(void);

extern void main_task(void * pvParameteres);

void main(void);

/**
 * function name: main
 * Description  : .
 * Arguments    : None.
 * Return Value : None.
 * */
void main(void)
{

    Processing_Before_Start_Kernel();

    /* Start the scheduler.  Initialization that requires the OS to be running,
     * including the Wi-Fi initialization, is performed in the RTOS daemon task
     * startup hook. */
    vTaskStartScheduler();

    while(1)
    {
        ;
    }

}
/**
 * End of function main
 * */

/******************************************************************************
* Function Name: vAssertCalled
* Description  : This function is used to validate the input parameters.
* Arguments    : None.
* Return Value : None.
******************************************************************************/
void vAssertCalled(void)
{
#if defined(CONFIG_FREERTOS_ASSERT_FAIL_ABORT)
    /* Assert call defined for debug builds. */
    /* debugging with E1/E2/E2L emulator */
    /* if not using a emulator, you can use LED on/off or serial terminal */
    volatile unsigned long ul = 0;

    taskENTER_CRITICAL();
    {
        /* Program may stop here when you stop it by debugger. In the case,
        use the debugger to set ul to a non-zero value in order to step out
        of this function to determine why it was called. */
        printf("vAssertCalled \n");
        while( 0 == ul )
        {
            R_BSP_NOP();
        }
    }
    taskEXIT_CRITICAL();
#elif defined(ENABLE_UNIT_TESTS) || defined(FREERTOS_ENABLE_UNIT_TESTS)
    /* unity testing */
    /* TEST_ABORT() of unity_internal.h (and also TEST_PASS() of unity.h)
    jumps to the place where TEST_PROTECT() was executed. */
    TEST_ABORT();
#else /* defined(CONFIG_FREERTOS_ASSERT_DISABLE) || defined(NDEBUG) or nothing */
    /* Disable Assert call for release builds. */
    /* nothing to do */
#endif
} /* End of function vAssertCalled() */



/******************************************************************************
* Function Name: vApplicationIdleHook
* Description  : This function will be called on each cycle of the idle task.
*                NOTE: vApplicationIdleHook() MUST NOT CALL A FUNCTION
*                      THAT MIGHT BLOCK UNDER ANY CIRCUMSTANCES.
* Arguments    : None.
* Return Value : None.
******************************************************************************/
void vApplicationIdleHook(void)
{
    /* Implement user-code for user own purpose. */

    static TickType_t xLastPrint = 0;
    TickType_t xTimeNow;
    const TickType_t xPrintFrequency = pdMS_TO_TICKS( 5000 );

    xTimeNow = xTaskGetTickCount();

    if( ( xTimeNow - xLastPrint ) > xPrintFrequency )
    {
        configPRINT_STRING(("."));
        printf(".");
        xLastPrint = xTimeNow;
    }

}
/**
 * End of function vApplicationIdleHook()
 **/

/******************************************************************************
* Function Name: vApplicationTickHook
* Description  : This function will be called every tick interrupt.
*                NOTE: vApplicationTickHook() EXECUTES FROM WITHIN AN ISR,
*                      SO MUST BE VERY SHORT AND NOT USE MUCH STACK.
*                      IN ADDITION, NOT CALL ANY APIs WITHOUT "FromISR" OR
*                      "FROM_ISR" AT THE END.
* Arguments    : None.
* Return Value : None.
******************************************************************************/
void vApplicationTickHook(void)
{
    /* Implement user-code for user own purpose. */

}
/**
 * End of function vApplicationTickHook()
 **/

/******************************************************************************
* Function Name : Processing_Before_Start_Kernel
* Description   : Create a main task, FreeRTOS's objects (e.g. mailbox, task,
*                 semaphore, mutex...) if required.
* Arguments     : None.
* Return value  : None.
******************************************************************************/
void Processing_Before_Start_Kernel(void)
{
    BaseType_t ret;

    /************** task creation ****************************/
    /* Main task. */
    ret = xTaskCreate(main_task, "MAIN_TASK", 512, NULL, 3, NULL);
    if (pdPASS != ret)
    {
        while(1)
        {
            /* Failed! Task can not be created. */
        }
    }
}
/**
 * End of function Processing_Before_Start_Kernel()
 **/

/**
 * Function name: vApplicationSetupTimerInterrupt
 * Description  : none
 * Arguments    : none
 * Return value : none
 **/
void vApplicationSetupTimerInterrupt(void)
{
    uint32_t usClockHz = R_BSP_GetFclkFreqHz (); /* Get peripheral clock. */
    const uint16_t usCompareMatch = (usClockHz / configTICK_RATE_HZ) - 1UL;

    /* Start 32-bits IT clock */
    TML32EN = (uint16_t) 1U;

    /* Stop 32-bit interval timer */
    ITLCTL0 = 0x00U;
    ITLMKF0 |= 0x01U;
    ITLS0 &= (uint16_t) ~0x01;
    ITLEN00 = 0U;

    /* Disable INTITL interrupt */
    ITLMK = 1U;

    /* Clear INTITL interrupt flag */
    ITLIF = 0U;

    /* Set INTITL high priority */
    ITLPR1 = 1U;
    ITLPR0 = 1U;

    /* 32-bit interval timer used as 16-bit timer */
    ITLCTL0 = 0x40U;
    ITLCSEL0 &= 0xF8U;
    ITLCSEL0 |= 0x01U;
    ITLFDIV00 &= 0xF8U;
    ITLFDIV00 |= 0x00U;
    ITLCMP00 = usCompareMatch;

    /* Start interval timer */
    ITLMK = 0U;
    ITLMKF0 &= ~0x01U;
    ITLEN00 = 1U;

}
/***
 * End of function vApplicationSetupTimerInterrupt
 **/

/*-----------------------------------------------------------*/

