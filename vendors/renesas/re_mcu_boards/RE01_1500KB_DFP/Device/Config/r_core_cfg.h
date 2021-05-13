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
 * Copyright (C) 2018-2020 Renesas Electronics Corporation. All rights reserved.
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * File Name    : r_core_cfg.h
 * Version      : 1.20
 * Description  : "r_core" configuration file
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description
 *         : 21.08.2018 0.50     First release
 *         : 15.01.2019 0.60     OFS_SECURITR_MPU_REGISTER moved. 
 *                               In order to used OSF1 register at CMSIS-CORE,
 *                               it has moved out of "r_system_cfg.h".
 *                               Added to use OSIS, AWS macro.
 *                               Added SYSTEM_CFG_SOSC_NF_STOP macro.
 *                               Adapted to GSCE coding rules.
 *                               Changed default value of "SYSTEM_CFG_MOSC_WAIT_TIME" from 9(1021 cycle) to 5(61 cycle).
 *                               Changed default value of "SYSTEM_CFG_OFS0" and "SYSTEM_CFG_OFS1" to 0xFFFFFFFF.
 *                               Changed default value of "SYSTEM_CFG_SECMPU_XXXX" to 0xFFFFFFFF.
 *                               (However, "SYSTEM_CFG_SECMPU_CONTROL_SETTING" is excluded.) 
 *                               Changed default value of "SYSTEM_CFG_SECMPU_CONTROL_SETTING" to 0xFFFF.
 *                               Added SYSTEM_CFG_EHC_MODE macro.
 *         : 26.02.2019 0.61     Deleted bit shift operation of "SYSTEM_CFG_OFS0", "SYSTEM_CFG_OFS1"
 *                               and "SYSTEM_CFG_SECMPU_CONTROL_SETTING".
 *                               Added SYSTEM_CFG_PLL_ENABLE macro.
 *                               Changed default value of "SYSTEM_CFG_MOCO_ENABLE" to disable(=0)
 *                               Changed default value of "SYSTEM_CFG_LOCO_ENABLE" to disable(=0)
 *         : 09.04.2019 0.70     Changed default value of "SYSTEM_CFG_PLL_DIV" to division ratio(=1)
 *                               Changed default value of "SYSTEM_CFG_MOSC_DRIVE" to oscillation current large(=7)
 *         : 23.07.2019 1.00     Change the product name to RE Family
 *                                - Update comments
 *         : 10.03.2020 1.10     Added "SYSTEM_CFG_EHC_LVD1LVL" macro and "SYSTEM_CFG_EHC_LVDBATLVL"macro.
 *                               Modified the parameter checking of "SYSTEM_CFG_PLL_RANGE".
 *         : 26.05.2020 1.20     Modified the transition in the Sub clock oscillator drive capability.
 **********************************************************************************************************************/

/*******************************************************************************************************************//**
 * @addtogroup grp_cmsis_core
 * @{
 **********************************************************************************************************************/

#ifndef CORE_CFG_H
#define CORE_CFG_H

/*******************************************************************************************************************//**
 * @brief Main clock oscillator operation@n
 *        0 = MOSC is not to be used.@n
 *        1 = MOSC is to be used.@n
 **********************************************************************************************************************/
#define SYSTEM_CFG_MOSC_ENABLE            (1)

/*******************************************************************************************************************//**
 * @brief Input clock frequency in Hz (MOSC)@n
 **********************************************************************************************************************/
#define SYSTEM_CFG_MOSC_FREQUENCY_HZ      (32000000)

/*******************************************************************************************************************//**
 * @brief Main clock oscillator drive capability (MOMCR.MODRV)@n
 *        0 = Setting prohibited@n
 *        1 = Small oscillation current@n
 *        ...@n
 *        7 = Large oscillating current@n
 **********************************************************************************************************************/
#define SYSTEM_CFG_MOSC_DRIVE             (7)

/*******************************************************************************************************************//**
 * @brief Main clock oscillator clock source select (MOMCR.MOSEL)@n
 *        0 = Oscillator@n
 *        1 = External clock@n
 **********************************************************************************************************************/
#define SYSTEM_CFG_MOSC_CLOCK_SOURCE      (0)

/*******************************************************************************************************************//**
 * @brief Main clock oscillator low power enable (MOMCR.OSCLPEN)@n
 *        0 = OSCLPEN is not used.@n
 *        1 = OSCLPEN is used.@n
 **********************************************************************************************************************/
#define SYSTEM_CFG_MOSC_LOW_POWER_ENABLE  (0)

/*******************************************************************************************************************//**
 * @brief Main clock oscillation stabilization time (MOSCWTCR.MSTS)@n
 *        1 cycle = 1/(fLOCO_max[MHz]) = 1/(0.032768*1.3)@n
 *        (fLOCO_max = maximum frequency for fLOCO)@n
 *        0 = Setting prohibited@n
 *        1 =    2 cycle@n
 *        2 =    5 cycle@n
 *        3 =   13 cycle@n
 *        4 =   29 cycle@n
 *        5 =   61 cycle@n
 *        6 =  125 cycle@n
 *        7 =  253 cycle@n
 *        8 =  509 cycle@n
 *        9 = 1021 cycle@n
 **********************************************************************************************************************/
#define SYSTEM_CFG_MOSC_WAIT_TIME         (5)

/*******************************************************************************************************************//**
 * @brief High-speed on-chip oscillator operation@n
 *        0 = HOCO is not to be used.@n
 *        1 = HOCO is to be used.@n
 **********************************************************************************************************************/
#define SYSTEM_CFG_HOCO_ENABLE            (0)

/*******************************************************************************************************************//**
 * @brief High-speed on-chip oscillator frequency in Hz (HOCOMCR.HCFRQ)@n
 *        0 = HOCO oscillation frequency is 24 MHz@n
 *        1 = HOCO oscillation frequency is 32 MHz@n
 *        2 = HOCO oscillation frequency is 48 MHz@n
 *        3 = HOCO oscillation frequency is 64 MHz@n
 **********************************************************************************************************************/
#define SYSTEM_CFG_HOCO_FREQUENCY         (1)

/*******************************************************************************************************************//**
 * @brief Medium-speed on-chip oscillator operation@n
 *        0 = MOCO is not to be used.@n
 *        1 = MOCO is to be used.@n
 **********************************************************************************************************************/
#define SYSTEM_CFG_MOCO_ENABLE            (1)

/*******************************************************************************************************************//**
 * @brief Low-speed on-chip oscillator operation@n
 *        0 = LOCO is not to be used.@n
 *        1 = LOCO is to be used.@n
 **********************************************************************************************************************/
#define SYSTEM_CFG_LOCO_ENABLE            (1)

/*******************************************************************************************************************//**
 * @brief Sub clock oscillator operation@n
 *        0 = Sub clock is not to be used.@n
 *        1 = Sub clock is to be used.@n
 **********************************************************************************************************************/
#define SYSTEM_CFG_SOSC_ENABLE            (0)

/*******************************************************************************************************************//**
 * @brief Sub clock oscillator drive capability (SOMCR.SODRV)@n
 *        0 = Standard CL@n
 *        1 = Low CL6p@n
 *        2 = Low CL4p@n
 *        3 = Low CL7p@n
 **********************************************************************************************************************/
#define SYSTEM_CFG_SOSC_DRIVE             (0)

/*******************************************************************************************************************//**
 * @brief Sub clock oscillator noise filter (SOMCR.SONFSTP)@n
 *        0 = Noise filter is to be used.@n
 *        1 = Noise filter is not to be used.@n
 **********************************************************************************************************************/
#define SYSTEM_CFG_SOSC_NF_STOP           (0)

/*******************************************************************************************************************//**
 * @brief PLL circuit operation@n
 *        0 = PLL is not to be used.@n
 *        1 = PLL is to be used.@n
 **********************************************************************************************************************/
#define SYSTEM_CFG_PLL_ENABLE             (0)

/*******************************************************************************************************************//**
 * @brief PLL input frequency division ratio select(PLLCCR.PLIDIV)@n
 *        0 = /1@n
 *        1 = /2@n
 *        2 = /3@n
 *        3 = /4@n
 **********************************************************************************************************************/
#define SYSTEM_CFG_PLL_DIV                (1)

/*******************************************************************************************************************//**
 * @brief PLL frequency multiplication factor select(PLLCCR.PLLMUL)@n
 *        0 = Setting prohibited@n
 *        1 = *2@n
 *        2 = *3@n
 *        ...@n
 *        7 = *8@n
 **********************************************************************************************************************/
#define SYSTEM_CFG_PLL_MUL                (1)

/*******************************************************************************************************************//**
 * @brief PLL frequency in Hz@n
 **********************************************************************************************************************/
#if   (0 == SYSTEM_CFG_PLL_DIV)    /* SYSTEM_CFG_MOSC_FREQUENCY_HZ */
    #define SYSTEM_CFG_PLL_DIV_FREQUENCY  (SYSTEM_CFG_MOSC_FREQUENCY_HZ)
#elif (1 == SYSTEM_CFG_PLL_DIV)    /* SYSTEM_CFG_MOSC_FREQUENCY_HZ/2 */
    #define SYSTEM_CFG_PLL_DIV_FREQUENCY  (SYSTEM_CFG_MOSC_FREQUENCY_HZ/2)
#elif (2 == SYSTEM_CFG_PLL_DIV)    /* SYSTEM_CFG_MOSC_FREQUENCY_HZ/3 */
    #define SYSTEM_CFG_PLL_DIV_FREQUENCY  (SYSTEM_CFG_MOSC_FREQUENCY_HZ/3)
#elif (3 == SYSTEM_CFG_PLL_DIV)    /* SYSTEM_CFG_MOSC_FREQUENCY_HZ/4 */
    #define SYSTEM_CFG_PLL_DIV_FREQUENCY  (SYSTEM_CFG_MOSC_FREQUENCY_HZ/4)
#else
    #error "ERROR - SYSTEM_CFG_PLL_DIV value (config)"
#endif

#if   (0 == SYSTEM_CFG_PLL_MUL)
    #error "ERROR - SYSTEM_CFG_PLL_MUL value (config)"
#elif (1 == SYSTEM_CFG_PLL_MUL)    /* SYSTEM_CFG_MOSC_FREQUENCY_HZ*2 */
    #define SYSTEM_CFG_PLL_FREQUENCY      (SYSTEM_CFG_PLL_DIV_FREQUENCY*2)
#elif (2 == SYSTEM_CFG_PLL_MUL)    /* SYSTEM_CFG_MOSC_FREQUENCY_HZ*3 */
    #define SYSTEM_CFG_PLL_FREQUENCY      (SYSTEM_CFG_PLL_DIV_FREQUENCY*3)
#elif (3 == SYSTEM_CFG_PLL_MUL)    /* SYSTEM_CFG_MOSC_FREQUENCY_HZ*4 */
    #define SYSTEM_CFG_PLL_FREQUENCY      (SYSTEM_CFG_PLL_DIV_FREQUENCY*4)
#elif (4 == SYSTEM_CFG_PLL_MUL)    /* SYSTEM_CFG_MOSC_FREQUENCY_HZ*5 */
    #define SYSTEM_CFG_PLL_FREQUENCY      (SYSTEM_CFG_PLL_DIV_FREQUENCY*5)
#elif (5 == SYSTEM_CFG_PLL_MUL)    /* SYSTEM_CFG_MOSC_FREQUENCY_HZ*6 */
    #define SYSTEM_CFG_PLL_FREQUENCY      (SYSTEM_CFG_PLL_DIV_FREQUENCY*6)
#elif (6 == SYSTEM_CFG_PLL_MUL)    /* SYSTEM_CFG_MOSC_FREQUENCY_HZ*7 */
    #define SYSTEM_CFG_PLL_FREQUENCY      (SYSTEM_CFG_PLL_DIV_FREQUENCY*7)
#elif (7 == SYSTEM_CFG_PLL_MUL)    /* SYSTEM_CFG_MOSC_FREQUENCY_HZ*8 */
    #define SYSTEM_CFG_PLL_FREQUENCY      (SYSTEM_CFG_PLL_DIV_FREQUENCY*8)
#else
    #error "ERROR - SYSTEM_CFG_PLL_MUL value (config)"
#endif

/*******************************************************************************************************************//**
 * @brief PLL frequency range select(PLLCCR.FSEL0)@n
 *        0 = 32MHz - 48MHz@n
 *        1 = 48MHz - 64MHz@n
 **********************************************************************************************************************/
#if   (64000000U < SYSTEM_CFG_PLL_FREQUENCY)
    #error "ERROR - PLL frequency (config)"
#elif (48000000U < SYSTEM_CFG_PLL_FREQUENCY)     /* SYSTEM_CFG_MOSC_FREQUENCY_HZ>48MHz */
    #define SYSTEM_CFG_PLL_RANGE          (1)
#elif (32000000U <= SYSTEM_CFG_PLL_FREQUENCY)    /* SYSTEM_CFG_MOSC_FREQUENCY_HZ>=32MHz */
    #define SYSTEM_CFG_PLL_RANGE          (0)
#else
#if (1 == SYSTEM_CFG_PLL_ENABLE)
    #error "ERROR - PLL frequency (config)"
#else
    #define SYSTEM_CFG_PLL_RANGE          (0)
    #warning "WARNING - PLL frequency (config)"
#endif  /* (1 == SYSTEM_CFG_PLL_ENABLE) */
#endif

/*******************************************************************************************************************//**
 * @brief System clock source select(SCKSCR.CKSEL)@n
 *        ICLK/PCLKA:PCLKB = N:1 (N : integer number)@n
 *        0 = High-speed on-chip oscillator@n
 *        1 = Medium-speed on-chip oscillator (default)@n
 *        2 = Low-speed on-chip oscillator@n
 *        3 = Main clock oscillator@n
 *        4 = Sub clock oscillator@n
 *        5 = PLL circuit@n
 **********************************************************************************************************************/
#define SYSTEM_CFG_CLOCK_SOURCE           (3)

/*******************************************************************************************************************//**
 * @brief ICLK and PCLKA frequency division ratio select(SCKDIVCR.ICK)@n
 *        The following frequency relationship is required between ICLK/PCLKA and PCLKB@n
 *        ICLK/PCLKA:PCLKB = N:1 (N : integer number)@n
 *        0 = /1 (default)@n
 *        1 = /2@n
 *        2 = /4@n
 *        3 = /8@n
 *        4 = /16@n
 *        5 = /32@n
 *        6 = /64@n
 **********************************************************************************************************************/
#define SYSTEM_CFG_ICK_PCKA_DIV           (0)

/*******************************************************************************************************************//**
 * @brief PCLKB frequency division ratio select(SCKDIVCR.PCKB)@n
 *        The following frequency relationship is required between ICLK/PCLKA and PCLKB@n
 *        ICLK/PCLKA:PCLKB = N:1 (N : integer number)@n
 *        0 = /1 (default)@n
 *        1 = /2@n
 *        2 = /4@n
 *        3 = /8@n
 *        4 = /16@n
 *        5 = /32@n
 *        6 = /64@n
 **********************************************************************************************************************/
#define SYSTEM_CFG_PCKB_DIV               (0)

/*******************************************************************************************************************//**
 * @brief Power control mode select@n
 *        0 = Boost mode (BOOST)@n
 *        1 = Normal mode (High-speed) (NORMAL) (Default)@n
 *        2 = Normal mode (Low-speed) (NORMAL)@n
 *        3 = Normal mode (Subosc-speed) (NORMAL)@n
 *        4 = Low-Leakage-Current mode (VBB)@n
 **********************************************************************************************************************/
#define SYSTEM_CFG_POWER_CONTROL_MODE     (1)

/*******************************************************************************************************************//**
 * @brief  EHC operating mode select@n
 *         0 = Normal mode (Default)@n
 *         1 = EHC mode (EHMD=1)@n
 **********************************************************************************************************************/
#define SYSTEM_CFG_EHC_MODE               (0)

/*******************************************************************************************************************//**
 * @brief  LVD1 level select@n
 *         0 = Vdet1_0@n
 *         1 = Vdet1_1@n
 *         2 = Vdet1_3@n
 *         3 = Vdet1_5@n
 *         4 = Vdet1_7@n
 *         5 = Vdet1_9@n
 *         6 = Vdet1_B@n
 *         7 = Vdet1_D (Default)@n
 **********************************************************************************************************************/
#define SYSTEM_CFG_EHC_LVD1LVL            (7)

/*******************************************************************************************************************//**
 * @brief  LVD BAT level select@n
 *         0 = VdetBAT_5 (Default)@n
 *         1 = VdetBAT_7@n
 *         2 = VdetBAT_8@n
 *         3 = VdetBAT_B@n
 *         4 = VdetBAT_D@n
 **********************************************************************************************************************/
#define SYSTEM_CFG_EHC_LVDBATLVL          (0)

/*******************************************************************************************************************//**
 * @name OFS_SECURITR_MPU_REGISTER
 **********************************************************************************************************************/
/* @{ */
#define SYSTEM_CFG_OFS0                     (0xFFFFFFFF)    /* OFS0 register          */
#define SYSTEM_CFG_OFS1                     (0xFFFFFFFF)    /* OFS1 register          */
#define SYSTEM_CFG_SECMPU_PC0_START         (0xFFFFFFFF)    /* SECMPU PC0 START       */
#define SYSTEM_CFG_SECMPU_PC0_END           (0xFFFFFFFF)    /* SECMPU PC0 END         */
#define SYSTEM_CFG_SECMPU_PC1_START         (0xFFFFFFFF)    /* SECMPU PC1 START       */
#define SYSTEM_CFG_SECMPU_PC1_END           (0xFFFFFFFF)    /* SECMPU PC1 END         */
#define SYSTEM_CFG_SECMPU_REGION0_START     (0xFFFFFFFF)    /* SECMPU REGION0 START   */
#define SYSTEM_CFG_SECMPU_REGION0_END       (0xFFFFFFFF)    /* SECMPU REGION0 END     */
#define SYSTEM_CFG_SECMPU_REGION1_START     (0xFFFFFFFF)    /* SECMPU REGION1 START   */
#define SYSTEM_CFG_SECMPU_REGION1_END       (0xFFFFFFFF)    /* SECMPU REGION1 END     */
#define SYSTEM_CFG_SECMPU_REGION2_START     (0xFFFFFFFF)    /* SECMPU REGION2 START   */
#define SYSTEM_CFG_SECMPU_REGION2_END       (0xFFFFFFFF)    /* SECMPU REGION2 END     */
#define SYSTEM_CFG_SECMPU_CONTROL_SETTING   (0xFFFF)        /* SECMPU CONTROL SETTING */
/* @} */

/*******************************************************************************************************************//**
 * @name OSIS_REGISTER
 **********************************************************************************************************************/
/* @{ */
#define SYSTEM_CFG_ID_CODE_PROTECTION_1      (0xFFFFFFFF)    /* ID CODE PROTECTION 1 */
#define SYSTEM_CFG_ID_CODE_PROTECTION_2      (0xFFFFFFFF)    /* ID CODE PROTECTION 2 */
#define SYSTEM_CFG_ID_CODE_PROTECTION_3      (0xFFFFFFFF)    /* ID CODE PROTECTION 3 */
#define SYSTEM_CFG_ID_CODE_PROTECTION_4      (0xFFFFFFFF)    /* ID CODE PROTECTION 4 */
/* @} */

/*******************************************************************************************************************//**
 * @name AWS_REGISTER
 **********************************************************************************************************************/
/* @{ */
#define SYSTEM_CFG_AWS                       (0xFFFFFFFF)    /* AWS register */
/* @} */

#endif /* CORE_CFG_H */

/*******************************************************************************************************************//**
 * @} (end addtogroup grp_cmsis_core)
 **********************************************************************************************************************/

/* End of file (r_core_cfg.h) */
