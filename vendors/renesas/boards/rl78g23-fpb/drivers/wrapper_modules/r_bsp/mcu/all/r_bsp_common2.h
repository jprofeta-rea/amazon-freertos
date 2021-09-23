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
 * File Name    : r_bsp_common2.h
 * Version      : 0.0
 * Description  : .
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description

 *********************************************************************************************************************/

/**********************************************************************************************************************
 Includes   <System Includes> , "Project Includes"
 *********************************************************************************************************************/
#include "platform.h"

#ifndef R_BSP_COMMON2_H_
#define R_BSP_COMMON2_H_

/**********************************************************************************************************************
 Macro definitions
 *********************************************************************************************************************/

/* This file is for compatibility with RX. */

#define R_BSP_SET_PSW(x)
#define R_BSP_EVENACCESS_SFR

#define R_BSP_NOP()                         ( BSP_NOP() )
#define R_BSP_InterruptsDisable()           ( BSP_DISABLE_INTERRUPT() )
#define R_BSP_InterruptsEnable()            ( BSP_ENABLE_INTERRUPT() )

//#define R_BSP_ATTRIB_STATIC_INTERRUPT       (static)

#define R_BSP_GPIO_PORT(x, y)               ( P ## x ## _bit.no ## y )

/* Null argument definitions. */
#define FIT_NO_FUNC                         ((void (*)(void *))0x10000000)  /* Reserved space on RX */
#define FIT_NO_PTR                          ((void *)0x10000000)            /* Reserved space on RX */

#define BSP_ROM_SIZE_BYTES                  (768 * 1024UL)
#define BSP_RAM_SIZE_BYTES                  (48 * 1024UL)

#if defined(__CCRL__)
#define BSP_NEAR_FUNC                       __near
#elif defined(__ICCRL78__)
#define BSP_NEAR_FUNC                       __near_func
#endif

/**********************************************************************************************************************
 Global Typedef definitions
 *********************************************************************************************************************/

typedef enum e_delay_units
{
    BSP_DELAY_MICROSECS = 3,        // Requested delay amount is in microseconds
    BSP_DELAY_MILLISECS = 1000,     // Requested delay amount is in milliseconds
    BSP_DELAY_SECS = 1              // Requested delay amount is in seconds
} bsp_delay_units_t;

/**********************************************************************************************************************
 External global variables
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Exported global functions
 *********************************************************************************************************************/


/**********************************************************************************************************************
 * function name: R_BSP_SoftwareDelay
 *********************************************************************************************************************/
bool R_BSP_SoftwareDelay(uint32_t delay, bsp_delay_units_t units);

/**********************************************************************************************************************
 * function name: R_BSP_SoftwareReset
 *********************************************************************************************************************/
void R_BSP_SoftwareReset(void);

/***
 * function name: R_BSP_GET_PSW
 * @return
 */
uint16_t R_BSP_GET_PSW(void);

uint32_t R_BSP_CpuInterruptLevelRead (void);

bool R_BSP_CpuInterruptLevelWrite (uint32_t level);

#endif /* R_BSP_COMMON2_H_ */
