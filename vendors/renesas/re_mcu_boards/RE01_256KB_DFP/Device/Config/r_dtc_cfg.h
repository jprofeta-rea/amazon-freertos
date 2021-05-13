/**********************************************************************************************************************
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
* Copyright (C) 2018-2020 Renesas Electronics Corporation. All rights reserved.    
**********************************************************************************************************************/
/**********************************************************************************************************************
* File Name    : r_dtc_cfg.h
* Version      : 0.80
* Description  : HAL Driver for DTC
**********************************************************************************************************************/
/**********************************************************************************************************************
* History : DD.MM.YYYY Version  Description
*         : 20.09.2019 0.40     Based on RE01 1500KB
*         : 04.02.2020 0.80     Fixed the problem that the R_DTC_GetTransferByte function could not allocate RAM
***********************************************************************************************************************/

#ifndef R_DTC_CFG_H
#define R_DTC_CFG_H
/******************************************************************************************************************//**
 @addgroup grp_device_hal_dtc
 @{
 *********************************************************************************************************************/

/******************************************************************************
 Includes <System Includes> , "Project Includes"
 *****************************************************************************/
#ifdef  __cplusplus
extern "C"
{
#endif

/******************************************************************************
 Macro definitions
 *****************************************************************************/
#define DTC_CFG_PARAM_CHECKING_ENABLE       (1)             ///< 0:Disable Parameter Check 1:Enable Parameter Check

#define DTC_COMPLETE_PRIORITY               (3)             ///< DTC complete interrupt priority value
                                                            ///  (set to 0 to 3, 0 is highest priority.)


/******************************************************************************************************************//**
 * @name R_DTC_API_LOCATION_CONFIG
 *       Definition of R_DTC API location configuration
 *       Please select "SYSTEM_SECTION_CODE" or "SYSTEM_SECTION_RAM_FUNC".@n
 *********************************************************************************************************************/
/* @{ */
#define DTC_CFG_R_DTC_GET_VERSION           (SYSTEM_SECTION_CODE)         ///< R_DTC_GetVersion() section
#define DTC_CFG_R_DTC_OPEN                  (SYSTEM_SECTION_CODE)         ///< R_DTC_Open() section
#define DTC_CFG_R_DTC_CLOSE                 (SYSTEM_SECTION_CODE)         ///< R_DTC_Close() section
#define DTC_CFG_R_DTC_CREATE                (SYSTEM_SECTION_CODE)         ///< R_DTC_Create() section
#define DTC_CFG_R_DTC_CONTROL               (SYSTEM_SECTION_CODE)         ///< R_DTC_Control() section
#define DTC_CFG_R_DTC_INTERRUPT_ENABLE      (SYSTEM_SECTION_CODE)         ///< R_DTC_InterruptEnable() section
#define DTC_CFG_R_DTC_INTERRUPT_DISABLE     (SYSTEM_SECTION_CODE)         ///< R_DTC_InterruptDisable() section
#define DTC_CFG_R_DTC_GET_STATE             (SYSTEM_SECTION_CODE)         ///< R_DTC_GetState() section
#define DTC_CFG_R_DTC_CLEAR_STATE           (SYSTEM_SECTION_CODE)         ///< R_DTC_ClearState() section
#define DTC_CFG_R_DTC_GET_TRANSFER_BYTE     (SYSTEM_SECTION_CODE)         ///< R_DTC_GetTransferByte() section
#define DTC_CFG_DTC_COMP_INTERRUPT          (SYSTEM_SECTION_RAM_FUNC)     ///< dtc_comp_interrupt() section
/* @} */



#ifdef  __cplusplus
}
#endif
/*******************************************************************************************************************//**
 * @} (end addgroup grp_hal_drv_dtc)
 **********************************************************************************************************************/

#endif /* R_DTC_CFG_H */


/* End of file (r_dtc_cfg.h) */

