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
 * File Name    : r_bsp_common2.c
 * Version      : 0.0
 * Description  : .
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description

 *********************************************************************************************************************/

/**********************************************************************************************************************
 Includes   <System Includes> , "Project Includes"
 *********************************************************************************************************************/
#include "r_bsp_common2.h"
#include "stdbool.h"

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

/**********************************************************************************************************************
 * function name: one_microsec_delay
 *********************************************************************************************************************/
static void one_microsec_delay(uint16_t clk)
{
    /* WAIT_LOOP */
    for(uint16_t cnt = clk; 0 < cnt; cnt--)
    {
        R_BSP_NOP();
        R_BSP_NOP();
    }
}
/**********************************************************************************************************************
 * End of function one_microsec_delay
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * function name: R_BSP_SoftwareDelay
 *********************************************************************************************************************/
bool R_BSP_SoftwareDelay(uint32_t delay, bsp_delay_units_t units)
{

    uint32_t time;
    uint32_t fclk;

    fclk = 32;//(R_BSP_GetFclkFreqHz() / 1000000);

    switch (units)
    {
        case BSP_DELAY_MICROSECS:
            time = delay;
            fclk = fclk / 20;
            break;
        case BSP_DELAY_MILLISECS:
            time = delay * 100;
            break;
        case BSP_DELAY_SECS:
            time = delay * 100000;
            break;
        default:
            break;
    }

    while(0 < time)
    {
        one_microsec_delay(fclk);
        time--;
    }

    return true;
}
/**********************************************************************************************************************
 * End of function R_BSP_SoftwareDelay
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * function name: R_BSP_SoftwareReset
 *********************************************************************************************************************/
void R_BSP_SoftwareReset(void)
{
    IAWCTL=0x80;                        // Unauthorized access detection enabled

    *(__far volatile char *)0x00=0x00;  // White at address 0x00
    R_BSP_NOP();
    while(1)
    {
        ;                               // Reset occurs before coming here
    }

}
/**********************************************************************************************************************
 * End of function R_BSP_SoftwareReset
 *********************************************************************************************************************/

/***
 * function name: R_BSP_GET_PSW
 * @return
 */
uint16_t R_BSP_GET_PSW(void)
{
    uint16_t ret;

    ret = __get_psw();

    return ret;
}
/***
 * End of function R_BSP_GET_PSW
 */


uint32_t R_BSP_CpuInterruptLevelRead (void) {
    return 0;
}

bool R_BSP_CpuInterruptLevelWrite (uint32_t level) {
    return true;
}
