/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No 
* other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all 
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, 
* FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM 
* EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES 
* SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS 
* SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of 
* this software. By using this software, you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer 
*
* Copyright (C) 2016-2019 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/**********************************************************************************************************************
* File Name    : r_sci_rl78.c
* Description  : Functions for using SCI on the RL78 device.
*                [Note] This module is a wrapper module for the firmware update library to use SAU.(UART only)
*                       This module is a provisional implementation.
***********************************************************************************************************************
* History : DD.MM.YYYY Version Description

***********************************************************************************************************************/

/*****************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include "platform.h"

#include "r_sci_rl78_private.h"

/*****************************************************************************
Typedef definitions
******************************************************************************/

/*****************************************************************************
Macro definitions
******************************************************************************/

/*****************************************************************************
Private global variables and functions
******************************************************************************/



/*****************************************************************************
* Function Name: sci_mcu_param_check
* Description  : This function parameters check on MCU.
*                 (channel range, interrupt priority, etc...)
* Arguments    : chan -
*                    channel to check
* Return Value : SCI_SUCCESS - 
*                    parameter check all successfully
*                SCI_ERR_BAD_CHAN -
*                    channel number invalid for part
*                SCI_ERR_INVALID_ARG -
*                    interrupt priority out of range
******************************************************************************/
sci_err_t sci_mcu_param_check(uint8_t const chan)
{
    /* channel range parameter check */
    if (SCI_NUM_CH <= chan)
    {
        return SCI_ERR_BAD_CHAN;
    }

    return SCI_SUCCESS;
} /* End of function sci_mcu_param_check() */

/*****************************************************************************
* Function Name: sci_init_register
* Description  : This function initializes the register for SCI.
* Arguments    : hdl -
*                    handle for channel (ptr to chan control block)
* Return Value : none
******************************************************************************/
void sci_init_register(sci_hdl_t const hdl)
{

    return;
} /* End of function sci_init_register() */

/*****************************************************************************
* Function Name: sci_init_bit_rate
* Description  : This function determines the best possible settings for the
*                baud rate registers for the specified peripheral clock speed
*                and baud rate. Note that this does not guarantee a low bit 
*                error rate, just the best possible one. The bit rate error is
*                returned in .1% increments. If the hardware cannot support
*                the specified combination, a value of 1000 (100% error) is
*                returned.
*
* NOTE: The transmitter and receiver (TE and RE bits in SCR) must be disabled 
*       prior to calling this function.
*
*       The application must pause for 1 bit time after the BRR register
*       is loaded before transmitting/receiving to allow time for the clock
*       to settle. 
*
* Arguments    : hdl -
*                    Handle for channel (ptr to chan control block)
*                    NOTE: mode element must be already set
*                pclk -
*                    Peripheral clock speed; e.g. 24000000 for 24MHz
*                baud -
*                    Baud rate; 19200, 57600, 115200, etc.
* Return Value : bit error in .1% increments; e.g. 16 = 1.6% bit rate error
*                a value of 1000 denotes 100% error; no registers set
******************************************************************************/
int32_t sci_init_bit_rate(sci_hdl_t const  hdl,
                            uint32_t const   pclk,
                            uint32_t const   baud)
{
    uint16_t i;
    uint32_t num_divisors = 0;
    uint16_t ratio = 0;
    baud_divisor_t const __far *p_baud_info = NULL;

    uint16_t ck0n = 0;
    uint32_t sdrmn = 0;

#if SCI_CFG_PARAM_CHECKING_ENABLE
    if ((0 == pclk) || (0 == baud))
    {
        return 1000;
    }
#endif

    /* SELECT PROPER TABLE BASED UPON MODE */
    if (SCI_MODE_ASYNC == hdl->mode)
    {
#if (SCI_CFG_ASYNC_INCLUDED)
        p_baud_info = async_baud;
        num_divisors = NUM_DIVISORS_ASYNC;
#endif
    }
    else
    {
        /* SYNC or SSPI */
#if (SCI_CFG_SSPI_INCLUDED || SCI_CFG_SYNC_INCLUDED)
        p_baud_info = sync_baud;
        num_divisors = NUM_DIVISORS_SYNC;
#endif
    }

    ratio = (pclk / baud);

    for (i = 0; i < num_divisors; i++)
    {
        if (ratio > (uint32_t)(p_baud_info[i].divisor * 256))
        {
            ck0n = ck0n + p_baud_info[i].exponent;
            ratio = ratio / p_baud_info[i].denominator;
        }
    }

    sdrmn = ((ratio - 1) & 0xfe) << 8;

    switch(hdl->channel)
    {
        case 0:
            if (0 == (SMR00 & 0x8000))
            {
#if (false)
                SPS0 &= 0xf0;
                SPS0 |= ck0n;
#endif
            }
            else
            {
#if (false)
                SPS0 &= 0x0f;
                SPS0 |= ck0n << 4;
#endif
            }
            if (0 == (SMR01 & 0x8000))
            {
#if (false)
                SPS0 &= 0xf0;
                SPS0 |= ck0n;
#endif
            }
            else
            {
#if (false)
                SPS0 &= 0x0f;
                SPS0 |= ck0n << 4;
#endif
            }
            break;
        case 1:
            if (0 == (SMR02 & 0x8000))
            {
#if (false)
                SPS0 &= 0xf0;
                SPS0 |= ck0n;
#endif
            }
            else
            {
#if (false)
                SPS0 &= 0x0f;
                SPS0 |= ck0n << 4;
#endif
            }
            if (0 == (SMR03 & 0x8000))
            {
#if (false)
                SPS0 &= 0xf0;
                SPS0 |= ck0n;
#endif
            }
            else
            {
#if (false)
                SPS0 &= 0x0f;
                SPS0 |= ck0n << 4;
#endif
            }
            break;
        case 2:
            break;
        case 3:
            break;
        default:
            break;
    }

    return 0;

} /* End of function sci_init_bit_rate() */

/*****************************************************************************
* Function Name: sci_initialize_ints
* Description  : This function sets priority, clears flags, and sets 
*                interrupts in both the ICU and SCI peripheral. These include 
*                RXI, TXI, TEI, and ERI/GROUP12 interrupts.
* Arguments    : hdl - 
*                    handle for channel (ptr to chan control block)
*                priority -
*                    priority for interrupts
* Return Value : none
******************************************************************************/
void sci_initialize_ints(sci_hdl_t const hdl,
                        uint8_t const   priority)
{

    return;    
} /* End of function sci_initialize_ints() */

/*****************************************************************************
* Function Name: sci_disable_ints
* Description  : This function disable interrupts in both the ICU and SCI 
*                peripheral. These include RXI, TXI, TEI, ERI, and group
*                interrupts.
* Arguments    : hdl - 
*                    handle for channel (ptr to chan control block)
* Return Value : none
******************************************************************************/
void sci_disable_ints(sci_hdl_t const hdl)
{

    return;
} /* End of function sci_disable_ints() */



#if (SCI_CFG_ASYNC_INCLUDED)

//#if defined(__CCRL__)
//#pragma section text FWUPLIB
//#elif defined(__ICCRL78__)
//#pragma default_function_attributes = @ "FWUPLIB"
//#endif

/*****************************************************************************
* Function Name: sci_async_cmds
* Description  : This function configures non-standard UART hardware and
*                performs special software operations.
*
* Arguments    : hdl -
*                    handle for channel (ptr to chan control block)
*                cmd -
*                    command to run
*                p_args -
*                    pointer argument(s) specific to command
* Return Value : SCI_SUCCESS -
*                    Command completed successfully.
*                SCI_ERR_NULL_PTR -
*                    p_args is NULL when required for cmd
*                SCI_ERR_INVALID_ARG -
*                    The cmd value or p_args contains an invalid value.
******************************************************************************/
sci_err_t sci_async_cmds(sci_hdl_t const hdl,
                        sci_cmd_t const cmd,
                        void            *p_args)
{
    sci_err_t   err=SCI_SUCCESS;
    int32_t     bit_err;
    uint32_t    slow_baud;

#if SCI_CFG_PARAM_CHECKING_ENABLE

    /* Check parameters */
    if ((NULL == p_args)
            && ((SCI_CMD_TX_Q_BYTES_FREE == cmd) || (SCI_CMD_RX_Q_BYTES_AVAIL_TO_READ == cmd)|| (SCI_CMD_COMPARE_RECEIVED_DATA == cmd)))
    {
        return SCI_ERR_NULL_PTR;
    }

#endif

    switch(cmd)
    {
        case (SCI_CMD_EN_NOISE_CANCEL):
        {
            /* Not Supported. */
        break;
        }

        case (SCI_CMD_OUTPUT_BAUD_CLK):
        {
            /* Not Supported. */
        break;
        }

        case (SCI_CMD_START_BIT_EDGE):
        {
            /* Not Supported. */
        break;
        }

#if SCI_CFG_TEI_INCLUDED
        case (SCI_CMD_EN_TEI):  /* SCI_CMD_EN_TEI is obsolete command, but it exists only for compatibility with older version. */
        {
        break;
        }
#endif

        case (SCI_CMD_TX_Q_FLUSH):
        {
            /* Disable TXI interrupt */
            DISABLE_TXI_INT;
            R_BYTEQ_Flush(hdl->u_tx_data.que);
            ENABLE_TXI_INT;
        break;
        }

        case (SCI_CMD_RX_Q_FLUSH):
        {
            /* Disable RXI interrupt */
            DISABLE_RXI_INT;
            R_BYTEQ_Flush(hdl->u_rx_data.que);
            ENABLE_RXI_INT;
        break;
        }

        case (SCI_CMD_TX_Q_BYTES_FREE):
        {
            /* Casting pointer void* to uint16_t* type is valid */
            R_BYTEQ_Unused(hdl->u_tx_data.que, (uint16_t *) p_args);
        break;
        }

        case (SCI_CMD_RX_Q_BYTES_AVAIL_TO_READ):
        {
            /* Casting pointer void* type to uint16_t* type is valid  */
            R_BYTEQ_Used(hdl->u_rx_data.que, (uint16_t *) p_args);
        break;
        }

        case (SCI_CMD_GENERATE_BREAK):
        {
            /* Not supported. */
        break;
        }

#if SCI_CFG_DATA_MATCH_INCLUDED
        case SCI_CMD_COMPARE_RECEIVED_DATA:
        {
            hdl->rom->regs->DCCR.BIT.DFER = 0; /* Clear Match Data Framing Error Flag */
            hdl->rom->regs->DCCR.BIT.DPER = 0; /* Clear Match Data Parity Error Flag */
            hdl->rom->regs->DCCR.BIT.DCME = 1; /* Enable Data match function */
            hdl->rom->regs->CDR.BYTE.L = *((unsigned char *)p_args); /* Comparison data */
        break;
        }
#endif

        default:
        {
            err = SCI_ERR_INVALID_ARG;
        break;
        }
    }

    return err;
} /* End of function sci_async_cmds() */

//#if defined(__CCRL__)
//#pragma section
//#elif defined(__ICCRL78__)
//#pragma default_function_attributes =
//#endif

#endif /* End of SCI_CFG_ASYNC_INCLUDED */

#if (SCI_CFG_SSPI_INCLUDED || SCI_CFG_SYNC_INCLUDED)
/*****************************************************************************
* Function Name: sci_sync_cmds
* Description  : This function performs special software operations specific
*                to the SSPI and SYNC protocols.
*
* Arguments    : hdl -
*                    handle for channel (ptr to chan control block)
*                cmd -
*                    command to run
*                p_args -
*                    pointer argument(s) specific to command
* Return Value : SCI_SUCCESS -
*                    Command completed successfully.
*                SCI_ERR_NULL_PTR -
*                    p_args is NULL when required for cmd
*                SCI_ERR_INVALID_ARG -
*                    The cmd value or p_args contains an invalid value.
*                    May be due to mode channel is operating in.
******************************************************************************/
sci_err_t sci_sync_cmds(sci_hdl_t const hdl,
                        sci_cmd_t const cmd,
                        void            *p_args)
{
    sci_spi_mode_t  spi_mode;
    sci_cb_args_t   args;
    sci_err_t       err = SCI_SUCCESS;

    switch (cmd)
    {
        case (SCI_CMD_CHECK_XFER_DONE):
        {
            if (false == hdl->tx_idle)
            {
                err = SCI_ERR_XFER_NOT_DONE;
            }
        break;
        }

        case (SCI_CMD_XFER_LSB_FIRST):
        {
            hdl->rom->regs->SCR.BYTE &= (~SCI_EN_XCVR_MASK);
            SCI_SCR_DUMMY_READ;
            hdl->rom->regs->SCMR.BIT.SDIR = 0;
            SCI_IR_TXI_CLEAR;
            hdl->rom->regs->SCR.BYTE |= SCI_EN_XCVR_MASK;
        break;
        }

        case (SCI_CMD_XFER_MSB_FIRST):
        {
            hdl->rom->regs->SCR.BYTE &= (~SCI_EN_XCVR_MASK);
            SCI_SCR_DUMMY_READ;
            hdl->rom->regs->SCMR.BIT.SDIR = 1;
            SCI_IR_TXI_CLEAR;
            hdl->rom->regs->SCR.BYTE |= SCI_EN_XCVR_MASK;
        break;
        }

        case (SCI_CMD_INVERT_DATA):
        {
            hdl->rom->regs->SCR.BYTE &= (~SCI_EN_XCVR_MASK);
            SCI_SCR_DUMMY_READ;
            hdl->rom->regs->SCMR.BIT.SINV ^= 1;
            SCI_IR_TXI_CLEAR;
            hdl->rom->regs->SCR.BYTE |= SCI_EN_XCVR_MASK;
        break;
        }

        case (SCI_CMD_ABORT_XFER):
        {
            /* Disable receive interrupts in ICU and peripheral */
            DISABLE_RXI_INT;
            DISABLE_ERI_INT;

            hdl->rom->regs->SCR.BYTE &= (~(SCI_SCR_REI_MASK | SCI_SCR_RE_MASK | SCI_SCR_TE_MASK));

            hdl->tx_cnt = 0;
            hdl->tx_dummy = false;
            hdl->tx_idle = true;

            /* Do callback if available */
            if ((NULL != hdl->callback) && (FIT_NO_FUNC != hdl->callback))
            {
                args.hdl = hdl;
                args.event = SCI_EVT_XFER_ABORTED;

                /* Casting pointer to void* is valid */
                hdl->callback((void *)&args);
            }

            *hdl->rom->ir_rxi = 0;                  /* clear rxi interrupt flag */
            (*hdl->rom->icu_grp) &= (~hdl->rom->eri_ch_mask);  /* clear eri interrupt flag */

            ENABLE_ERI_INT;                         /* enable rx err interrupts in ICU */
            ENABLE_RXI_INT;                         /* enable receive interrupts in ICU */

            /* Enable receive interrupt in peripheral after rcvr or will get "extra" interrupt */
            hdl->rom->regs->SCR.BYTE |= (SCI_SCR_RE_MASK | SCI_SCR_TE_MASK);
            hdl->rom->regs->SCR.BYTE |= SCI_SCR_REI_MASK;
        break;
        }

        case (SCI_CMD_CHANGE_SPI_MODE):
        {
    #if SCI_CFG_PARAM_CHECKING_ENABLE

            if (SCI_MODE_SSPI != hdl->mode)
            {
                return SCI_ERR_INVALID_ARG;
            }

            /* Check parameters */
            if ((NULL == p_args ) || (FIT_NO_PTR == p_args))
            {
                return SCI_ERR_NULL_PTR;
            }

            /* Casting pointer void* type is valid */
            spi_mode = *((sci_spi_mode_t *)p_args);

            if ((SCI_SPI_MODE_0 != spi_mode) && (SCI_SPI_MODE_1 != spi_mode)
                    && (SCI_SPI_MODE_2 != spi_mode) && (SCI_SPI_MODE_3 != spi_mode))
            {
                return SCI_ERR_INVALID_ARG;
            }
    #endif
            hdl->rom->regs->SCR.BYTE &= (~SCI_EN_XCVR_MASK);
            SCI_SCR_DUMMY_READ;
            hdl->rom->regs->SPMR.BYTE &= 0x3F;      /* clear previous mode */
            hdl->rom->regs->SPMR.BYTE |= (*((uint8_t *)p_args));
            SCI_IR_TXI_CLEAR;
            hdl->rom->regs->SCR.BYTE |= SCI_EN_XCVR_MASK;
        break;
        }

        default:
        {
            err = SCI_ERR_INVALID_ARG;
        break;
        }
    }

    return err;
} /* End of function sci_sync_cmds() */
#endif /* End of SCI_CFG_SSPI_INCLUDED || SCI_CFG_SYNC_INCLUDED */

/*****************************************************************************
ISRs
******************************************************************************/


#if (SCI_CFG_ASYNC_INCLUDED)

/*****************************************************************************
* sciN_txiN_isr
* Description  : TXI interrupt routines for every SCI channel
******************************************************************************/

#if SCI_CFG_CH0_INCLUDED
/*******************************************************************************
 * Function Name: sci0_txi0_isr
 * Description  : TXI interrupt routines for SCI0 channel
 ******************************************************************************/
//#pragma interrupt sci0_txi0_isr(vect=INTST0)
void sci0_txi0_isr(void)
{
    txi_handler(&ch0_ctrl);
}
#endif /* End of SCI_CFG_CH0_INCLUDED */

#if SCI_CFG_CH1_INCLUDED
/*******************************************************************************
 * Function Name: sci1_txi1_isr
 * Description  : TXI interrupt routines for SCI1 channel
 ******************************************************************************/
#pragma interrupt sci1_txi1_isr(vect=INTST1)
void sci1_txi1_isr(void)
{
    txi_handler(&ch1_ctrl);
}
#endif /* End of SCI_CFG_CH1_INCLUDED */

#if SCI_CFG_CH2_INCLUDED
/*******************************************************************************
 * Function Name: sci2_txi2_isr
 * Description  : TXI interrupt routines for SCI2 channel
 ******************************************************************************/

//#if defined(__CCRL__)
//#pragma section text FWUPLIB
//#elif defined(__ICCRL78__)
//#pragma default_function_attributes = @ "FWUPLIB"
//#endif
#if (1)
#pragma interrupt sci2_txi2_isr(vect=INTST2)
#endif
void sci2_txi2_isr(void)
{
    txi_handler(&ch2_ctrl);
}

//#if defined(__CCRL__)
//#pragma section
//#elif defined(__ICCRL78__)
//#pragma default_function_attributes =
//#endif

#endif /* End of SCI_CFG_CH2_INCLUDED */

#if SCI_CFG_CH3_INCLUDED
/*******************************************************************************
 * Function Name: sci3_txi3_isr
 * Description  : TXI interrupt routines for SCI3 channel
 ******************************************************************************/
#pragma interrupt sci3_txi3_isr(vect=INTST3)
void sci3_txi3_isr(void)
{
    txi_handler(&ch3_ctrl);
}
#endif /* End of SCI_CFG_CH3_INCLUDED */

#endif /* End of SCI_CFG_ASYNC_INCLUDED */

#if SCI_CFG_TEI_INCLUDED
/*****************************************************************************
* sciN_teiN_isr
*
* Description  : TEI interrupt routines for every SCI channel.
*                BSP gets main group interrupt, then vectors to/calls these
*                "interrupts"/callbacks.
******************************************************************************/
#if (0)
#if SCI_CFG_CH0_INCLUDED
/*******************************************************************************
 * Function Name: sci0_tei0_isr
 * Description  : TEI interrupt routines for SCI0 channel.
 ******************************************************************************/
void sci0_tei0_isr(void *cb_args)
{
    tei_handler(&ch0_ctrl);
} /* End of function sci0_tei0_isr() */
#endif /* End of SCI_CFG_CH0_INCLUDED */

#if SCI_CFG_CH1_INCLUDED
/*******************************************************************************
 * Function Name: sci1_tei1_isr
 * Description  : TEI interrupt routines for SCI1 channel.
 ******************************************************************************/
void sci1_tei1_isr(void *cb_args)
{
    tei_handler(&ch1_ctrl);
} /* End of function sci1_tei1_isr() */
#endif /* End of SCI_CFG_CH1_INCLUDED */

#if SCI_CFG_CH2_INCLUDED
/*******************************************************************************
 * Function Name: sci2_tei2_isr
 * Description  : TEI interrupt routines for SCI2 channel.
 ******************************************************************************/
void sci2_tei2_isr(void *cb_args)
{
    tei_handler(&ch2_ctrl);
} /* End of function sci2_tei2_isr() */
#endif /* End of SCI_CFG_CH2_INCLUDED */

#if SCI_CFG_CH3_INCLUDED
/*******************************************************************************
 * Function Name: sci3_tei3_isr
 * Description  : TEI interrupt routines for SCI3 channel.
 ******************************************************************************/
void sci3_tei3_isr(void *cb_args)
{
    tei_handler(&ch3_ctrl);
} /* End of function sci3_tei3_isr() */
#endif /* End of SCI_CFG_CH3_INCLUDED */

#endif /* 0 */
#endif /* SCI_CFG_TEI_INCLUDED */

/*****************************************************************************
* sciN_rxiN_isr
* Description  : RXI interrupt routines for every SCI channel
******************************************************************************/

#if SCI_CFG_CH0_INCLUDED
/*******************************************************************************
 * Function Name: sci0_rxi0_isr
 * Description  : RXI interrupt routines for SCI0 channel
 ******************************************************************************/
//#pragma interrupt sci0_rxi0_isr(vect=INTSR0)
void sci0_rxi0_isr(void)
{
    rxi_handler(&ch0_ctrl);
}
#endif /* End of SCI_CFG_CH0_INCLUDED */

#if SCI_CFG_CH1_INCLUDED
/*******************************************************************************
 * Function Name: sci1_rxi1_isr
 * Description  : RXI interrupt routines for SCI1 channel
 ******************************************************************************/
#pragma interrupt sci1_rxi1_isr(vect=INTSR1)
void sci1_rxi1_isr(void)
{
    rxi_handler(&ch1_ctrl);
}
#endif /* End of SCI_CFG_CH1_INCLUDED */

#if SCI_CFG_CH2_INCLUDED
/*******************************************************************************
 * Function Name: sci2_rxi2_isr
 * Description  : RXI interrupt routines for SCI2 channel
 ******************************************************************************/

//#if defined(__CCRL__)
//#pragma section text FWUPLIB
//#elif defined(__ICCRL78__)
//#pragma default_function_attributes = @ "FWUPLIB"
//#endif
#if (1)
#pragma interrupt sci2_rxi2_isr(vect=INTSR2)
#endif
void sci2_rxi2_isr(void)
{
    rxi_handler(&ch2_ctrl);
}

//#if defined(__CCRL__)
//#pragma section
//#elif defined(__ICCRL78__)
//#pragma default_function_attributes =
//#endif

#endif /* End of SCI_CFG_CH2_INCLUDED */

#if SCI_CFG_CH3_INCLUDED
/*******************************************************************************
 * Function Name: sci3_rxi3_isr
 * Description  : RXI interrupt routines for SCI3 channel
 ******************************************************************************/
#pragma interrupt sci3_rxi3_isr(vect=INTSR3)
void sci3_rxi3_isr(void)
{
    rxi_handler(&ch3_ctrl);
}
#endif /* End of SCI_CFG_CH3_INCLUDED */

/*****************************************************************************
* sciN_eriN_isr
*
* Description  : ERI interrupt routines for every SCI channel.
*                BSP gets main group interrupt, then vectors to/calls these
*                "interrupts"/callbacks.
******************************************************************************/

#if SCI_CFG_CH0_INCLUDED
/*****************************************************************************
* Function name: sci0_eri0_isr
* Description  : ERI interrupt routines for SCI0 channel.
******************************************************************************/
//#pragma interrupt sci0_eri0_isr(vect=INTSRE0)
void sci0_eri0_isr(void)
{
    eri_handler(&ch0_ctrl);
}
#endif /* End of SCI_CFG_CH0_INCLUDED */

#if SCI_CFG_CH1_INCLUDED
/*****************************************************************************
* Function name: sci1_eri1_isr
* Description  : ERI interrupt routines for SCI1 channel.
******************************************************************************/
#pragma interrupt sci1_eri1_isr(vect=INTSRE1)
void sci1_eri1_isr(void)
{
    eri_handler(&ch1_ctrl);
}
#endif /* End of SCI_CFG_CH1_INCLUDED */

#if SCI_CFG_CH2_INCLUDED
/*****************************************************************************
* Function name: sci2_eri2_isr
* Description  : ERI interrupt routines for SCI2 channel.
******************************************************************************/

//#if defined(__CCRL__)
//#pragma section text FWUPLIB
//#elif defined(__ICCRL78__)
//#pragma default_function_attributes = @ "FWUPLIB"
//#endif
#if (1)
#pragma interrupt sci2_eri2_isr(vect=INTSRE2)
#endif
void sci2_eri2_isr(void)
{
    eri_handler(&ch2_ctrl);
}

//#if defined(__CCRL__)
//#pragma section
//#elif defined(__ICCRL78__)
//#pragma default_function_attributes =
//#endif

#endif /* End of SCI_CFG_CH2_INCLUDED */

#if SCI_CFG_CH3_INCLUDED
/*****************************************************************************
* Function name: sci3_eri3_isr
* Description  : ERI interrupt routines for SCI3 channel.
******************************************************************************/
#pragma interrupt sci3_eri3_isr(vect=INTSRE3)
void sci3_eri3_isr(void)
{
    eri_handler(&ch3_ctrl);
}
#endif /* End of SCI_CFG_CH3_INCLUDED */
