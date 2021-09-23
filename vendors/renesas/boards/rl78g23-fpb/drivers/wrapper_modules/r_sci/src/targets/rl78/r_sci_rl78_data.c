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
 * File Name    : r_sci_rl78_data.c
 * Version      : 0.0
 * Description  : .
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description

 *********************************************************************************************************************/

/*****************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include "platform.h"

#include "r_sci_rl78_private.h"

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

/* BAUD DIVISOR INFO */

/* Asynchronous */

#if (SCI_CFG_ASYNC_INCLUDED)
/* NOTE: diff than SCI async baud table, but should provide same results */
const SCI_FAR baud_divisor_t async_baud[NUM_DIVISORS_ASYNC]=
{
    {32, 5, 32},
    {8,  3, 8},
    {4,  2, 4},
    {2,  1, 2},
    {1,  1, 2}
};
#endif

/* Synchronous and Simple SPI */
/* BRR = (PCLK/(divisor * baud)) - 1 */
/* abcs=0, bdgm=0, divisor = 8*pow(2,2n-1) */

#if (SCI_CFG_SSPI_INCLUDED || SCI_CFG_SYNC_INCLUDED)
/* NOTE: Identical to SCI sync baud table */
const baud_divisor_t sync_baud[NUM_DIVISORS_SYNC]=
{
    /* divisor result, abcs, bgdm, n */
    {4,   0, 0, 0},
    {16,  0, 0, 1},
    {64,  0, 0, 2},
    {256, 0, 0, 3}
};
#endif


/* CHANNEL MEMORY ALLOCATIONS */
#if SCI_CFG_CH0_INCLUDED

/* channel control block */
sci_ch_ctrl_t   ch0_ctrl = {0, SCI_MODE_OFF, 0, NULL, NULL, NULL, true
                            #if (SCI_CFG_SSPI_INCLUDED || SCI_CFG_SYNC_INCLUDED)
                            , true, 0, 0, false
                            #endif
                            #if SCI_CFG_FIFO_INCLUDED
                            , false
                            , 0
                            , 0
                            , 0
                            , 0
                            #endif
                            };
#endif /* End of SCI_CFG_CH0_INCLUDED */


#if SCI_CFG_CH1_INCLUDED

/* channel control block */
sci_ch_ctrl_t   ch1_ctrl = {1, SCI_MODE_OFF, 0, NULL, NULL, NULL, true
                            #if (SCI_CFG_SSPI_INCLUDED || SCI_CFG_SYNC_INCLUDED)
                            , true, 0, 0, false
                            #endif
                            #if SCI_CFG_FIFO_INCLUDED
                            , false
                            , 0
                            , 0
                            , 0
                            , 0
                            #endif
                            };
#endif /* End of SCI_CFG_CH1_INCLUDED */


#if SCI_CFG_CH2_INCLUDED

/* channel control block */
sci_ch_ctrl_t  ch2_ctrl = {2, SCI_MODE_OFF, 0, NULL, NULL, NULL, true
                            #if (SCI_CFG_SSPI_INCLUDED || SCI_CFG_SYNC_INCLUDED)
                            , true, 0, 0, false
                            #endif
                            #if SCI_CFG_FIFO_INCLUDED
                            , false
                            , 0
                            , 0
                            , 0
                            , 0
                            #endif
                            };
#endif /* End of SCI_CFG_CH2_INCLUDED */


#if SCI_CFG_CH3_INCLUDED

/* channel control block */
sci_ch_ctrl_t   ch3_ctrl = {3, SCI_MODE_OFF, 0, NULL, NULL, NULL, true
                            #if (SCI_CFG_SSPI_INCLUDED || SCI_CFG_SYNC_INCLUDED)
                            , true, 0, 0, false
                            #endif
                            #if SCI_CFG_FIFO_INCLUDED
                            , false
                            , 0
                            , 0
                            , 0
                            , 0
                            #endif
                            };
#endif  /* End of SCI_CFG_CH3_INCLUDED */

/* SCI HANDLE-ARRAY DECLARATION */
const sci_hdl_t SCI_FAR g_handles[SCI_NUM_CH] =
{
#if SCI_CFG_CH0_INCLUDED
            &ch0_ctrl,
#else
            NULL,
#endif /**/
#if SCI_CFG_CH1_INCLUDED
            &ch1_ctrl,
#else
            NULL,
#endif /**/
#if SCI_CFG_CH2_INCLUDED
            &ch2_ctrl,
#else
            NULL,
#endif /**/
#if SCI_CFG_CH3_INCLUDED
            &ch3_ctrl,
#else
            NULL,
#endif /**/

};



void SetReg_TXD(uint8_t channel, uint8_t byte)
{

    switch(channel)
    {
        case 0:
        {
#if (SCI_CFG_CH0_INCLUDED)
            while(0 != SSR00 & 0x0020);
            TXD0 = (byte);
#endif
            break;
        }
        case 1:
        {
#if (SCI_CFG_CH1_INCLUDED)
            while(0 != SSR02 & 0x0020);
            TXD1 = (byte);
#endif
            break;
        }
        case 2:
        {
#if (SCI_CFG_CH2_INCLUDED)
            while(0 != SSR10 & 0x0020);
            TXD2 = (byte);
#endif
            break;
        }
        case 3:
        {
#if (SCI_CFG_CH3_INCLUDED)
            while(0 != SSR12 & 0x0020);
            TXD3 = (byte);
#endif
            break;
        }
        default:
            break;
    }
}


void GetReg_RXD(uint8_t channel, uint8_t *byte)
{
    switch(channel)
    {
        case 0:
        {
#if (SCI_CFG_CH0_INCLUDED)
            *byte = RXD0;
#endif
            break;
        }
        case 1:
        {
#if (SCI_CFG_CH1_INCLUDED)
            *byte = RXD1;
#endif
            break;
        }
        case 2:
        {
#if (SCI_CFG_CH2_INCLUDED)
            *byte = RXD2;
#endif
            break;
        }
        case 3:
        {
#if (SCI_CFG_CH3_INCLUDED)
            *byte = RXD3;
#endif
            break;
        }
        default:
            break;
    }
}


void SetReg_SRMKn(uint8_t channel, uint8_t flag)
{
    switch(channel)
    {
        case 0:
        {
#if (SCI_CFG_CH0_INCLUDED)
            SRMK0 = flag;
#endif
            break;
        }
        case 1:
        {
#if (SCI_CFG_CH1_INCLUDED)
            SRMK1 = flag;
#endif
            break;
        }
        case 2:
        {
#if (SCI_CFG_CH2_INCLUDED)
            SRMK2 = flag;
#endif
            break;
        }
        case 3:
        {
#if (SCI_CFG_CH3_INCLUDED)
            SRMK3 = flag;
#endif
            break;
        }
        default:
            break;
    }
}


void SetReg_SREMKn(uint8_t channel, uint8_t flag)
{
    switch(channel)
    {
        case 0:
        {
#if (SCI_CFG_CH0_INCLUDED)
            SREMK0 = flag;
#endif
            break;
        }
        case 1:
        {
#if (SCI_CFG_CH1_INCLUDED)
            SREMK1 = flag;
#endif
            break;
        }
        case 2:
        {
#if (SCI_CFG_CH2_INCLUDED)
            SREMK2 = flag;
#endif
            break;
        }
        case 3:
        {
#if (SCI_CFG_CH3_INCLUDED)
            SREMK3 = flag;
#endif
            break;
        }
        default:
            break;
    }
}


uint16_t GetReg_SSR(uint8_t channel)
{
    uint16_t err_type;
    switch(channel)
    {
        case 0:
        {
#if (SCI_CFG_CH0_INCLUDED)
            err_type = (uint8_t)(SSR01 & 0x0007U);
            SIR01 = err_type;
#endif
            break;
        }
        case 1:
        {
#if (SCI_CFG_CH1_INCLUDED)
            err_type = (uint8_t)(SSR03 & 0x0007U);
            SIR03 = err_type;
#endif
            break;
        }
        case 2:
        {
#if (SCI_CFG_CH2_INCLUDED)
            err_type = (uint8_t)(SSR11 & 0x0007U);
            SIR11 = err_type;
#endif
            break;
        }
        case 3:
        {
#if (SCI_CFG_CH3_INCLUDED)
            err_type = (uint8_t)(SSR13 & 0x0007U);
            SIR13 = err_type;
#endif
            break;
        }
        default:
            break;
    }
    return err_type;
}


