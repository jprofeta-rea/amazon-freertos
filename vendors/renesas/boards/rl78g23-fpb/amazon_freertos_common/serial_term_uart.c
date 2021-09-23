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
 * File Name    : serial_term_uart.c
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
#include "serial_term_uart.h"
#include "r_sci_rl_if.h"

/**********************************************************************************************************************
 Macro definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Local Typedef definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Exported global variables
 *********************************************************************************************************************/
sci_hdl_t serial_term_sci_handle;

/**********************************************************************************************************************
 Private (static) variables and functions
 *********************************************************************************************************************/

static void sci_serial_term_callback(void *pArgs);

void send0(unsigned char ch);
void send2(unsigned char ch);

/**********************************************************************************************************************
 * Function Name: sci_serial_term_callback
 * Description  : This is an SCI Async Mode callback function.
 * Arguments    : pArgs -
 *                 pointer to sci_cb_p_args_t structure cast to as void. Structure contains event as associated data.
 * Return Value : none
 *********************************************************************************************************************/
static void sci_serial_term_callback(void *pArgs)
{
    sci_cb_args_t *p_args;

    p_args = (sci_cb_args_t*) pArgs;

    if (SCI_EVT_RX_CHAR == p_args->event)
    {
        /* From RXI interrupt; received character data is in p_args->byte */
        R_BSP_NOP();
    }
    else if (SCI_EVT_RXBUF_OVFL == p_args->event)
    {
        /* From RXI interrupt; rx queue is full; 'lost' data is in p_args->byte
         You will need to increase buffer size or reduce baud rate */
        R_BSP_NOP();
    }
    else if (SCI_EVT_OVFL_ERR == p_args->event)
    {
        /* From receiver overflow error interrupt; error data is in p_args->byte
         Error condition is cleared in calling interrupt routine */
        R_BSP_NOP();
    }
    else if (SCI_EVT_FRAMING_ERR == p_args->event)
    {
        /* From receiver framing error interrupt; error data is in p_args->byte
         Error condition is cleared in calling interrupt routine */
        R_BSP_NOP();
    }
    else if (SCI_EVT_PARITY_ERR == p_args->event)
    {
        /* From receiver parity error interrupt; error data is in p_args->byte
         Error condition is cleared in calling interrupt routine */
        R_BSP_NOP();
    }
    else
    {
        /* Do nothing */
    }

}
/**********************************************************************************************************************
 * End of function sci_serial_term_callback
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: uart_config
 * Description  : Peripheral UART operation for log outpt.
 * Arguments    : none
 * Return Value : none
 *********************************************************************************************************************/
void uart_config(void)
{
    sci_cfg_t sci_config;
    sci_err_t sci_error;

#if (false)
    /* Set up the configuration data structure for asynchronous (UART) operation. */
    sci_config.async.baud_rate = 115200;
    sci_config.async.data_size = SCI_DATA_8BIT;
    sci_config.async.parity_en = SCI_PARITY_OFF;
    sci_config.async.parity_type = SCI_EVEN_PARITY;
    sci_config.async.stop_bits = SCI_STOPBITS_1;
    sci_config.async.int_priority = 2;
#endif

    /* OPEN ASYNC CHANNEL
     *  provide address of the configure structure.
     *  the callback function to be assigned,
     *  and the location for the handle to be stored.
     */
    sci_error = R_SCI_Open (SCI_CH_Serial_Term, SCI_MODE_ASYNC, &sci_config, sci_serial_term_callback,
                            &serial_term_sci_handle);

    if (SCI_SUCCESS != sci_error)
    {
        R_BSP_NOP();
    }
    R_Config_UART0_Start();

}
/**********************************************************************************************************************
 * End of function uart_config
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: uart_string_printf
 * Description  : Sends the specified character string on the set UART channel.
 * Arguments    : pString-
 *                  Pointer type log output character string.
 * Return Value : none
 *********************************************************************************************************************/
void uart_string_printf(SCI_FAR char *pString)
{
    sci_err_t sci_error;

    uint16_t retry = 0xffff;
    uint16_t str_length;
    uint16_t transmit_length;

    str_length = (uint16_t) strlen(pString);

    while ((retry > 0) && (str_length > 0))
    {

        sci_error = R_SCI_Control (serial_term_sci_handle, SCI_CMD_TX_Q_BYTES_FREE, &transmit_length);

        if (transmit_length > str_length)
        {
            transmit_length = str_length;
        }

        sci_error = R_SCI_Send (serial_term_sci_handle, (uint8_t SCI_FAR*) pString, transmit_length);

        if ((SCI_ERR_XCVR_BUSY == sci_error) || (SCI_ERR_INSUFFICIENT_SPACE == sci_error))
        {
            /* retry if previous transmission still in progress or tx buffer is insufficient. */
            retry--;
        }
        else
        {
            str_length -= transmit_length;
            pString += transmit_length;
        }

    }

    if (SCI_SUCCESS != sci_error)
    {
        R_BSP_NOP();
    }

}
/**********************************************************************************************************************
 * End of function uart_string_printf
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: putchar
 * Description  : none
 * Arguments    : none
 * Return Value : none
 *********************************************************************************************************************/
SCI_FAR_FUNC int putchar(int c)
{
    send0 ((unsigned char) c); /* 1 byte transmission */

    return c;
}
/**********************************************************************************************************************
 * End of function putchar
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: send0
 * Description  : none
 * Arguments    : none
 * Return Value : none
 *********************************************************************************************************************/
void send0(unsigned char ch)
{
    while (0 != (SSR00 & 0x20))
    {
        ;
    }

    STMK0 = 1U;
    TXD0 = ch;
    STMK0 = 0U;
    return;
}
/**********************************************************************************************************************
 * End of function send0
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: send2
 * Description  : none
 * Arguments    : none
 * Return Value : none
 *********************************************************************************************************************/
void send2(unsigned char ch)
{
    while (0 != (SSR10 & 0x20))
    {
        ;
    }

    STMK2 = 1U;
    TXD2 = ch;
    STMK2 = 0U;
    return;
}
/**********************************************************************************************************************
 * End of function send2
 *********************************************************************************************************************/
