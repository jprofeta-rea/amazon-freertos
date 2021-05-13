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
#include "platform.h"           // Located in the FIT BSP module
// changed 2020/10 start
// RX SCI FIT module include file
/////#include "r_sci_rx_if.h"        // The SCI module API interface file.
// changed 2020/10 end
// changed 2020/10 start
// RX PIN setting file
/////#include "r_pinset.h"
// changed 2020/10 end

// added 2020/10 start
//#include "R_Driver_USART.h"
#include "r_usart_cmsis_api.h"
// added 2020/10 end

/*******************************************************************************
 Macro definitions
 *******************************************************************************/
// removed 2020/10 start
// RX Setting
/*
#if !defined(MY_BSP_CFG_SERIAL_TERM_SCI)
#error "Error! Need to define MY_BSP_CFG_SERIAL_TERM_SCI in r_bsp_config.h"
#elif MY_BSP_CFG_SERIAL_TERM_SCI == (0)
#define R_SCI_PinSet_serial_term()  R_SCI_PinSet_SCI0()
#define SCI_CH_serial_term          SCI_CH0
#elif MY_BSP_CFG_SERIAL_TERM_SCI == (1)
#define R_SCI_PinSet_serial_term()  R_SCI_PinSet_SCI1()
#define SCI_CH_serial_term          SCI_CH1
#elif MY_BSP_CFG_SERIAL_TERM_SCI == (2)
#define R_SCI_PinSet_serial_term()  R_SCI_PinSet_SCI2()
#define SCI_CH_serial_term          SCI_CH2
#elif MY_BSP_CFG_SERIAL_TERM_SCI == (3)
#define R_SCI_PinSet_serial_term()  R_SCI_PinSet_SCI3()
#define SCI_CH_serial_term          SCI_CH3
#elif MY_BSP_CFG_SERIAL_TERM_SCI == (4)
#define R_SCI_PinSet_serial_term()  R_SCI_PinSet_SCI4()
#define SCI_CH_serial_term          SCI_CH4
#elif MY_BSP_CFG_SERIAL_TERM_SCI == (5)
#define R_SCI_PinSet_serial_term()  R_SCI_PinSet_SCI5()
#define SCI_CH_serial_term          SCI_CH5
#elif MY_BSP_CFG_SERIAL_TERM_SCI == (6)
#define R_SCI_PinSet_serial_term()  R_SCI_PinSet_SCI6()
#define SCI_CH_serial_term          SCI_CH6
#elif MY_BSP_CFG_SERIAL_TERM_SCI == (7)
#define R_SCI_PinSet_serial_term()  R_SCI_PinSet_SCI7()
#define SCI_CH_serial_term          SCI_CH7
#elif MY_BSP_CFG_SERIAL_TERM_SCI == (8)
#define R_SCI_PinSet_serial_term()  R_SCI_PinSet_SCI8()
#define SCI_CH_serial_term          SCI_CH8
#elif MY_BSP_CFG_SERIAL_TERM_SCI == (9)
#define R_SCI_PinSet_serial_term()  R_SCI_PinSet_SCI9()
#define SCI_CH_serial_term          SCI_CH9
#elif MY_BSP_CFG_SERIAL_TERM_SCI == (10)
#define R_SCI_PinSet_serial_term()  R_SCI_PinSet_SCI10()
#define SCI_CH_serial_term          SCI_CH10
#elif MY_BSP_CFG_SERIAL_TERM_SCI == (11)
#define R_SCI_PinSet_serial_term()  R_SCI_PinSet_SCI11()
#define SCI_CH_serial_term          SCI_CH11
#elif MY_BSP_CFG_SERIAL_TERM_SCI == (12)
#define R_SCI_PinSet_serial_term()  R_SCI_PinSet_SCI12()
#define SCI_CH_serial_term          SCI_CH12
#else
#error "Error! Invalid setting for MY_BSP_CFG_SERIAL_TERM_SCI in r_bsp_config.h"
#endif
*/
// removed 2020/10 end

// added 2020/10 start
#define UART_BUS_SPEED          (115200)              /* UART bus speed(bps) */
// added 2020/10 end


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

// changed 2020/10 start
/* Handle storage. */
/////sci_hdl_t     my_sci_handle;
// changed 2020/10 end

// changed 2020/10 start
// ** Mearge 256KB/1500KB
#ifdef RE01_256KB
    extern ARM_DRIVER_USART Driver_USART0;
    static ARM_DRIVER_USART *gsp_sci0_dev = &Driver_USART0;
#else
    extern ARM_DRIVER_USART Driver_USART4;
    static ARM_DRIVER_USART *gsp_sci4_dev = &Driver_USART4;
#endif
static void usart_callback(uint32_t event);
// changed 2020/10 end

/*****************************************************************************
* Function Name: uart_config
* Description  : prepares UART for operation
* Arguments    : none
* Return Value : none
******************************************************************************/
void uart_config(void)
{
// removed 2020/10 start
/*
    sci_cfg_t   my_sci_config;
    sci_err_t   my_sci_err;
*/
// removed 2020/10 end

    /* Initialize the I/O port pins for communication on this SCI channel.
    * This is specific to the MCU and ports chosen. For the RSKRX65-2M we will use the
    * SCI channel connected to the USB serial port emulation. */

// removed 2020/10 start
// RX Setting
/////    R_SCI_PinSet_serial_term();
// removed 2020/10 end

    /* Set up the configuration data structure for asynchronous (UART) operation. */
// removed 2020/10 start
/*
    my_sci_config.async.baud_rate    = 115200;
    my_sci_config.async.clk_src      = SCI_CLK_INT;
    my_sci_config.async.data_size    = SCI_DATA_8BIT;
    my_sci_config.async.parity_en    = SCI_PARITY_OFF;
    my_sci_config.async.parity_type  = SCI_EVEN_PARITY;
    my_sci_config.async.stop_bits    = SCI_STOPBITS_1;
    my_sci_config.async.int_priority = 3;    // 1=lowest, 15=highest
*/
// removed 2020/10 end

// changed 2020/10 start
// ** Mearge 256KB/1500KB
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

// ** Mearge 256KB/1500KB
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

// ** Mearge 256KB/1500KB
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
// ** Mearge 256KB/1500KB
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
// changed 2020/10 end

    /* OPEN ASYNC CHANNEL
    *  Provide address of the configure structure,
    *  the callback function to be assigned,
    *  and the location for the handle to be stored.*/

// changed 2020/10 start

/////    my_sci_err = R_SCI_Open(SCI_CH_serial_term, SCI_MODE_ASYNC, &my_sci_config, my_sci_callback, &my_sci_handle);

    /* If there were an error this would demonstrate error detection of API calls. */
/////    if (SCI_SUCCESS != my_sci_err)
/////    {
/////        R_BSP_NOP(); // Your error handling code would go here.
/////    }

// changed 2020/10 end

} /* End of function uart_config() */


/*****************************************************************************
* Function Name: my_sci_callback
* Description  : This is a template for an SCI Async Mode callback function.
* Arguments    : pArgs -
*                pointer to sci_cb_p_args_t structure cast to a void. Structure
*                contains event and associated data.
* Return Value : none
******************************************************************************/
// removed 2020/10 start
/////static void my_sci_callback(void *pArgs)
/////{
/////    sci_cb_args_t   *p_args;

/////    p_args = (sci_cb_args_t *)pArgs;

/////    if (SCI_EVT_RX_CHAR == p_args->event)
/////    {
        /* From RXI interrupt; received character data is in p_args->byte */
/////    	R_BSP_NOP();
/////    }
/////    else if (SCI_EVT_RXBUF_OVFL == p_args->event)
/////    {
        /* From RXI interrupt; rx queue is full; 'lost' data is in p_args->byte
           You will need to increase buffer size or reduce baud rate */
/////    	R_BSP_NOP();
/////    }
/////   else if (SCI_EVT_OVFL_ERR == p_args->event)
/////    {
        /* From receiver overflow error interrupt; error data is in p_args->byte
           Error condition is cleared in calling interrupt routine */
/////    	R_BSP_NOP();
/////    }
/////    else if (SCI_EVT_FRAMING_ERR == p_args->event)
/////    {
        /* From receiver framing error interrupt; error data is in p_args->byte
           Error condition is cleared in calling interrupt routine */
/////    	R_BSP_NOP();
/////    }
/////    else if (SCI_EVT_PARITY_ERR == p_args->event)
/////    {
        /* From receiver parity error interrupt; error data is in p_args->byte
           Error condition is cleared in calling interrupt routine */
/////    	R_BSP_NOP();
/////    }
/////    else
/////    {
        /* Do nothing */
/////    }

/////} /* End of function my_sci_callback() */

// removed 2020/10 end

void uart_string_printf(char *pString)
{
// changed 2020/10 start
    volatile uint16_t str_length = 0;
// ** Mearge 256KB/1500KB
#ifdef RE01_256KB
    volatile uint8_t sci_err;
#else
    volatile uint8_t sci_err = 0;
#endif
    volatile ARM_USART_STATUS sci_status;
    volatile uint32_t retry = 0xFFFF;

// ** Mearge 256KB/1500KB
#ifdef RE01_256KB
    str_length = (uint16_t)strlen(pString);
#else
    str_length = (uint32_t)strlen(pString);
#endif

    while ((retry > 0) && (str_length > 0))
    {

// ** Mearge 256KB/1500KB
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

// ** Mearge 256KB/1500KB
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
// changed 2020/10 end
}

// added 2020/10 start
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

// added 2020/10 end
