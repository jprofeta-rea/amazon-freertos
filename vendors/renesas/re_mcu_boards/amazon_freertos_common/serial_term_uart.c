/*******************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only
* intended for use with Renesas products. No other uses are authorized. This
* software is owned by Renesas Electronics Corporation and is protected under
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
* AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software
* and to discontinue the availability of this software. By using this software,
* you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2018 Renesas Electronics Corporation. All rights reserved.
*******************************************************************************/
/**********************************************************************************************************************
* File Name    : rskrx65n_uart.c
* Device(s)    : RSKRX65-2M
* Tool-Chain   : Renesas RX
* Description  :
***********************************************************************************************************************/
/**********************************************************************************************************************
* History : DD.MM.YYYY Version  Description
***********************************************************************************************************************/

/*****************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include <string.h>             // For strlen
#include "serial_term_uart.h"   // Serial Transfer Demo interface file.
#include "platform.h"           // Located in the FIT BSP module/
#include "r_usart_cmsis_api.h"
#define UART_BUS_SPEED          (115200)              /* UART bus speed(bps) */

/*******************************************************************************
 Exported global variables and functions (to be accessed by other files)
 *******************************************************************************/

/*******************************************************************************
 Private variables and functions
 *******************************************************************************/

/*****************************************************************************
Private global variables and functions
******************************************************************************/
static void my_sci_callback(void *pArgs);
#ifdef RE01_256KB
    extern ARM_DRIVER_USART Driver_USART0;
    static ARM_DRIVER_USART *gsp_sci0_dev = &Driver_USART0;
#else
    extern ARM_DRIVER_USART Driver_USART4;
    static ARM_DRIVER_USART *gsp_sci4_dev = &Driver_USART4;
#endif
static void usart_callback(uint32_t event);

/*****************************************************************************
* Function Name: uart_config
* Description  : prepares UART for operation
* Arguments    : none
* Return Value : none
******************************************************************************/
void uart_config(void)
{
#ifdef RE01_256KB
    if (ARM_DRIVER_OK != gsp_sci0_dev->Initialize(usart_callback))
#else
    if (ARM_DRIVER_OK != gsp_sci4_dev->Initialize(usart_callback))
#endif
    {
        while(1)
        {
            ;   /* Intentionally empty braces. */
        }
    }

#ifdef RE01_256KB
    if (ARM_DRIVER_OK != gsp_sci0_dev->PowerControl(ARM_POWER_FULL))
#else
    if (ARM_DRIVER_OK != gsp_sci4_dev->PowerControl(ARM_POWER_FULL))
#endif
    {
        while(1)
        {
            ;   /* Intentionally empty braces. */
        }
    }

#ifdef RE01_256KB
    if (ARM_DRIVER_OK != gsp_sci0_dev->Control((ARM_USART_MODE_ASYNCHRONOUS |    /* UART (Asynchronous) */
                                                ARM_USART_DATA_BITS_8       |    /* 8 Data bits */
                                                ARM_USART_PARITY_NONE       |    /* No Parity */
                                                ARM_USART_STOP_BITS_1       |    /* 1 Stop bit */
                                                ARM_USART_FLOW_CONTROL_NONE)     /* No Flow Control */
                                               ,UART_BUS_SPEED))
#else
    if (ARM_DRIVER_OK != gsp_sci4_dev->Control((ARM_USART_MODE_ASYNCHRONOUS |    /* UART (Asynchronous) */
                                                ARM_USART_DATA_BITS_8       |    /* 8 Data bits */
                                                ARM_USART_PARITY_NONE       |    /* No Parity */
                                                ARM_USART_STOP_BITS_1       |    /* 1 Stop bit */
                                                ARM_USART_FLOW_CONTROL_NONE)     /* No Flow Control */
                                               ,UART_BUS_SPEED))
#endif
    {
        while(1)
        {
            ;   /* Intentionally empty braces. */
        }
    }

    /** enable transmit and receive */
#ifdef RE01_256KB
    if (ARM_DRIVER_OK != gsp_sci0_dev->Control(ARM_USART_CONTROL_TX_RX,1))
#else
    if (ARM_DRIVER_OK != gsp_sci4_dev->Control(ARM_USART_CONTROL_TX,1))
#endif
    {
        while(1)
        {
            ;   /* Intentionally empty braces. */
        }
    }

} /* End of function uart_config() */

void uart_string_printf(char *pString)
{
    volatile uint16_t str_length = 0;
#ifdef RE01_256KB
    volatile uint8_t sci_err;
#else
    volatile uint8_t sci_err = 0;
#endif
    volatile ARM_USART_STATUS sci_status;
    volatile uint32_t retry = 0xFFFF;
#ifdef RE01_256KB
    str_length = (uint16_t)strlen(pString);
#else
    str_length = (uint32_t)strlen(pString);
#endif

    while ((retry > 0) && (str_length > 0))
    {

#ifdef RE01_256KB
    	sci_err = gsp_sci0_dev->Send(pString, str_length);
#else
    	sci_err = gsp_sci4_dev->Send(pString, str_length);
#endif


    	if (sci_err == ARM_DRIVER_ERROR_BUSY)
    	{
            retry--; // retry if previous transmission still in progress or tx buffer is insufficient.
            continue;
    	}

    	while(str_length > 0)
    	{
#ifdef RE01_256KB
            sci_status = gsp_sci0_dev->GetStatus();
#else
    		sci_status = gsp_sci4_dev->GetStatus();
#endif
            if (sci_status.tx_busy == 0 )
            {
                str_length = 0;
            }
    	}
    }
}

/***********************************************************************************************************************
* Function Name: usart_callback
* Description  : SCI0 Callback Function
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
static void usart_callback(uint32_t event)
{
    /** Check event */
    switch( event )
    {
        case ARM_USART_EVENT_SEND_COMPLETE:
            {
            	__NOP();
            ;   /* Describe the process when sending is completed */
            }
        break;

        case ARM_USART_EVENT_RECEIVE_COMPLETE:
            {
            	__NOP();
            ;   /* Describe processing when receiving is completed */
            }
        break;

        default:
            {
            /* Resume reception when a reception error occurs */
            	__NOP();
            }
        break;
    }

}   /* End of function usart_callback() */
