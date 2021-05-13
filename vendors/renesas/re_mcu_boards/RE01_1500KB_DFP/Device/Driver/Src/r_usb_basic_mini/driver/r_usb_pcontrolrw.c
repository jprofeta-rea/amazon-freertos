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
 * Copyright (C) 2015(2018) Renesas Electronics Corporation. All rights reserved.
 *****************************************************************************/
/******************************************************************************
 * File Name    : r_usb_pcontrolrw.c
 * Description  : USB Peripheral control transfer API code
 ******************************************************************************/
/*******************************************************************************
 * History : DD.MM.YYYY Version Description
 *         : 08.01.2014 1.00 First Release
 *         : 30.11.2018 1.10    Supporting Smart Configurator
*******************************************************************************/
/******************************************************************************
 Includes   <System Includes> , "Project Includes"
 ******************************************************************************/

#include "r_usb_basic_mini_api.h"
#include "r_usb_typedef.h"
#include "r_usb_extern.h"
#include "r_usb_bitdefine.h"
#include "r_usb_reg_access.h"

#if ((USB_CFG_MODE & USB_CFG_PERI) == USB_CFG_PERI)

/*******************************************************************************
 Macro definitions
 ******************************************************************************/

/*******************************************************************************
 Typedef definitions
 ******************************************************************************/

/*******************************************************************************
 Exported global variables (to be accessed by other files)
 ******************************************************************************/

/*******************************************************************************
 Private global variables and functions
 ******************************************************************************/
extern uint32_t g_usb_pstd_cmsis_pipe_transfer_result_size[USB_MAX_PIPE_NO + 1u];


/******************************************************************************
 Function Name   : usb_pstd_ctrl_read
 Description     : Called by R_USB_PstdCtrlRead, see it for description.
 Arguments       : uint32_t bsize    : Read size in bytes.
                 : uint8_t *table    : Start address of read data buffer.
 Return value    : uint16_t          : USB_WRITESHRT/USB_WRITE_END/USB_WRITING/USB_FIFOERROR.
 ******************************************************************************/
uint16_t usb_pstd_ctrl_read (uint32_t bsize, uint8_t *p_table)
{
    uint16_t end_flag;

    g_usb_pstd_pipe0_request = USB_ON;

    g_usb_pstd_data_cnt[USB_PIPE0] = bsize;
    gp_usb_pstd_data[USB_PIPE0] = p_table;
    g_usb_pstd_cmsis_pipe_transfer_result_size[USB_PIPE0] = bsize;

    usb_cstd_chg_curpipe((uint16_t) USB_PIPE0, (uint16_t) USB_CUSE, (uint16_t) USB_ISEL);

    /* Buffer clear */
    hw_usb_set_bclr(USB_CUSE);

    hw_usb_clear_status_bemp(USB_PIPE0);

    /* Peripheral Control sequence */
    end_flag = usb_pstd_write_data(USB_PIPE0, USB_CUSE);

    /* Peripheral control sequence */
    switch (end_flag)
    {
        /* End of data write */
        case USB_WRITESHRT :

            /* Enable not ready interrupt */
            usb_cstd_nrdy_enable((uint16_t) USB_PIPE0);

            /* Set PID=BUF */
            usb_cstd_set_buf((uint16_t) USB_PIPE0);
        break;

            /* End of data write (not null) */
        case USB_WRITEEND :

            /* Continue */
            /* Continue of data write */
        case USB_WRITING :

            /* Enable empty interrupt */
            hw_usb_set_bempenb((uint16_t) USB_PIPE0);

            /* Enable not ready interrupt */
            usb_cstd_nrdy_enable((uint16_t) USB_PIPE0);

            /* Set PID=BUF */
            usb_cstd_set_buf((uint16_t) USB_PIPE0);

        break;

            /* FIFO access error */
        case USB_FIFOERROR :
        break;
        default :
        break;
    }
    return (end_flag);     /* End or error or continue */
} /* End of function usb_pstd_ctrl_read */

/******************************************************************************
 Function Name   : usb_pstd_ctrl_write
 Description     : Called by R_USB_PstdCtrlWrite, see it for description.
 Arguments       : uint32_t bsize      : Write size in bytes.
                 : uint8_t *p_table    : Start address of write data buffer.
 Return value    : none
 ******************************************************************************/
void usb_pstd_ctrl_write (uint32_t bsize, uint8_t *p_table)
{
    g_usb_pstd_pipe0_request = USB_ON;

    g_usb_pstd_data_cnt[USB_PIPE0] = bsize;
    gp_usb_pstd_data[USB_PIPE0] = p_table;
    g_usb_pstd_cmsis_pipe_transfer_result_size[USB_PIPE0] = bsize;

    usb_cstd_chg_curpipe((uint16_t) USB_PIPE0, (uint16_t) USB_CUSE, USB_FALSE);

    /* Buffer clear */
    hw_usb_set_bclr(USB_CUSE);

    /* Interrupt enable */
    /* Enable ready interrupt */
    hw_usb_set_brdyenb((uint16_t) USB_PIPE0);

    /* Enable not ready interrupt */
    usb_cstd_nrdy_enable((uint16_t) USB_PIPE0);

    /* Set PID=BUF */
    usb_cstd_set_buf((uint16_t) USB_PIPE0);
} /* End of function usb_pstd_ctrl_write */

/******************************************************************************
 Function Name   : usb_pstd_ctrl_end
 Description     : End control transfer
 Arguments       : uint16_t status  : Transfer end status
 Return value    : none
 ******************************************************************************/
void usb_pstd_ctrl_end (uint16_t status)
{
    g_usb_pstd_pipe0_request = USB_OFF;

    /* Interrupt disable */
    /* BEMP0 disable */
    hw_usb_clear_bempenb((uint16_t) USB_PIPE0);

    /* BRDY0 disable */
    hw_usb_clear_brdyenb((uint16_t) USB_PIPE0);

    /* NRDY0 disable */
    hw_usb_clear_nrdyenb((uint16_t) USB_PIPE0);

    hw_usb_set_mbw(USB_CUSE, USB0_CFIFO_MBW);


    if ((USB_DATA_ERR == status) || (USB_DATA_OVR == status))
    {
        /* Request error */
        usb_pstd_set_stall_pipe0();
    }
    else if (USB_DATA_STOP == status)
    {
        /* Pipe stop */
        usb_cstd_set_nak((uint16_t) USB_PIPE0);
    }
    else
    {
        /* Set CCPL bit */
        hw_usb_pset_ccpl();
    }
} /* End of function usb_pstd_ctrl_end */
#endif  /* (USB_CFG_MODE & USB_CFG_PERI) == USB_CFG_REPI */

/******************************************************************************
 End  Of File
 ******************************************************************************/
