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
  ******************************************************************************/
/*******************************************************************************
 * File Name    : r_usb_preg_access.c
 * Description  : USB IP Peripheral control register access code
 *****************************************************************************/
/******************************************************************************
 * History : DD.MM.YYYY Version Description
 *         : 08.01.2014 1.00 First Release
 *         : 31.11.2018 1.10 Supporting Smart Configurator
 *****************************************************************************/
/******************************************************************************
 Includes   <System Includes> , "Project Includes"
 ******************************************************************************/
#include "r_usb_basic_mini_api.h"
#include "r_usb_typedef.h"
#include "r_usb_reg_access.h"
#include "r_usb_bitdefine.h"
#include "r_usb_extern.h"


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

/******************************************************************************
 Function Name   : hw_usb_pset_dprpu
 Description     : Set DPRPU-bit SYSCFG0 register.
                 : (Enable D+Line pullup when PeripheralController function is selected)
 Arguments       : none
 Return value    : none
 ******************************************************************************/
void hw_usb_pset_dprpu(void)
{
    USBFS->SYSCFG |= USB_DPRPU;
} /* End of function hw_usb_pset_dprpu */

/******************************************************************************
 Function Name   : hw_usb_pclear_dprpu
 Description     : Clear DPRPU-bit of the SYSCFG0 register.
                 : (Disable D+Line pullup when PeripheralController function is
                 : selected.)
 Arguments       : none
 Return value    : none
 ******************************************************************************/
void hw_usb_pclear_dprpu(void)
{
    USBFS->SYSCFG &= (~USB_DPRPU);
} /* End of function hw_usb_pclear_dprpu */

/******************************************************************************
 Function Name   : hw_usb_pset_wkup
 Description     : Set WKUP-bit DVSTCTR register.
                 : (Output Remote wakeup signal when PeripheralController function is selected)
 Arguments       : none
 Return value    : none
 ******************************************************************************/
void hw_usb_pset_wkup(void)
{
    USBFS->DVSTCTR0 |= USB_WKUP;
} /* End of function hw_usb_pset_dprpu */
/******************************************************************************
 End of function hw_usb_pset_wkup
 ******************************************************************************/

/******************************************************************************
 Function Name   : hw_usb_pset_enb_rsme
 Description     : Enable interrupt from RESUME
 Arguments       : none
 Return value    : none
 ******************************************************************************/
void hw_usb_pset_enb_rsme(void)
{
    USBFS->INTENB0 |= USB_RSME;
} /* End of function hw_usb_pset_enb_rsme */

/******************************************************************************
 Function Name   : hw_usb_pclear_enb_rsme
 Description     : Disable interrupt from RESUME
 Arguments       : none
 Return value    : none
 ******************************************************************************/
void hw_usb_pclear_enb_rsme(void)
{
    USBFS->INTENB0 &= (~USB_RSME);
} /* End of function hw_usb_pclear_enb_rsme */

/******************************************************************************
 Function Name   : hw_usb_pclear_sts_resm
 Description     : Clear interrupt status of RESUME.
 Arguments       : none
 Return value    : none
 ******************************************************************************/
void hw_usb_pclear_sts_resm(void)
{
    USBFS->INTSTS0 = (uint16_t)~USB_RESM;
} /* End of function hw_usb_pclear_sts_resm */

/******************************************************************************
 Function Name   : hw_usb_pclear_sts_valid
 Description     : Clear the Setup Packet Reception interrupt flag.
 Arguments       : none
 Return value    : none
 ******************************************************************************/
void hw_usb_pclear_sts_valid(void)
{
    USBFS->INTSTS0 = (uint16_t)~USB_VALID;
} /* End of function hw_usb_pclear_sts_valid */

/******************************************************************************
 Function Name   : hw_usb_pset_ccpl
 Description     : Enable termination of control transfer status stage.
 Arguments       : none
 Return value    : none
 ******************************************************************************/
void hw_usb_pset_ccpl(void)
{
    USBFS->DCPCTR |= USB_CCPL;
} /* End of function hw_usb_pset_ccpl */

/******************************************************************************
 Function Name   : hw_usb_pmodule_init
 Description     : 
 Arguments       : none
 Return value    : none
 ******************************************************************************/
void hw_usb_pmodule_init( void )
{

    USBFS->SYSCFG |= USB_SCKE;
    while (USB_SCKE != (USBFS->SYSCFG & USB_SCKE))
    {
        /* Wait for Set of SCKE */
    }
    USBFS->PHYSLEW = 0x5;
    USBFS->SYSCFG &= (~USB_DRPD);

    USBFS->SYSCFG |= USB_USBE;
    USBFS->CFIFOSEL  = USB0_CFIFO_MBW;
    USBFS->D0FIFOSEL = USB0_D0FIFO_MBW;
    USBFS->D1FIFOSEL = USB0_D1FIFO_MBW;
#if USB_CFG_ENDIAN == USB_CFG_BIG
    USBFS->CFIFOSEL  |= USB_BIGEND;
    USBFS->D0FIFOSEL |= USB_BIGEND;
    USBFS->D1FIFOSEL |= USB_BIGEND;
#endif  /* USB_CFG_ENDIAN == USB_CFG_BIG */
    USBFS->INTENB0 = (USB_BEMPE | USB_BRDYE | USB_VBSE | USB_DVSE | USB_CTRE);

} /* End of function hw_usb_pmodule_init */

#endif  /* (USB_CFG_MODE & USB_CFG_PERI) == USB_CFG_REPI */

/******************************************************************************
 End of file
 ******************************************************************************/

