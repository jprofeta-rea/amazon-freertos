/*********************************************************************************************************************
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
#include "RE01_1500KB.h"
#include "r_core_cfg.h"
#include "r_system_api.h"
#include "r_lpm_api.h"

/* Start user code for include. */

/* End user code.*/

/* Start user code for global.*/

/* End user code. */

/* Functions */
void NMI_Handler( void )  __attribute__ ((section(".ramfunc"))) ;     /* This is NMI Handler for User template*/
void LVD_for_EHC( void ) __attribute__ ((section(".ramfunc"))) ;   /* This is  User template code of EHC initialization in NMI Handler*/


/***********************************************************************************************************************
* Function Name: main
* Revision     : 1.30
* Description  : main function. Please add the code for your system.
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/


int main()
{
  uint32_t data;

  /**** It recommends use this part as template this device's start-up code. ****/
  /* If the API functions are allocated to RAM, this function must be executed first */
  /* It copies Driver codes, which change the allocation area from ROM to RAM, by using r_[module]_cfg.h */ 
  /* Attention: Every code Pin.c is always allocated to RAM. These functions are called from API.  */
  R_SYS_CodeCopy();
  /* Initialize System Function Driver. */
  /* This function needs to be called after R_SYS_CodeCopy function. */
  R_SYS_Initialize();
  /* Initialize LPM Function Driver. */
  /* This function needs to be called before call R_LPM_IOPowerSupplyModeSet function. */
  R_LPM_Initialize();
  /* Set Power Supply Open Control Register (VOCR register)                                      */
  /* If you doesn't clear bits which corresponding your using port domains, MCU cannot read      */
  /* correctly value of input signal.                                                            */
  /* The VOCR register is used for control such that, when power is not being supplied to        */
  /* the AVCC0, AVCC1, IOVCC0,IOVCC1, IOVCC2, IOVCC3, USB power supply, or MTDV power supply pin,*/
  /* the operation of circuits that operate with the pins to which power is not being supplied   */
  /* does not affect the circuits that operate with the pins to which power is being supplied.   */
  /* This feature is mainly used when the device is Energy Harvesting start-up.                  */
  /* Default value is 0xFF which is not propagated signal into internal device.                  */
  /* Please change the value of argument with your target board                                  */
  R_LPM_IOPowerSupplyModeSet((uint8_t)LPM_IOPOWER_SUPPLY_NONE);
  /**** End of template code.   ****/

  /*******************************************************************/
  /**** Write user code for user init and system operations here. ****/
  /*******************************************************************/



  while(1)
  {
    data++;
  }
  return 0;
}

/***********************************************************************************************************************
* Function Name: BoardInit
* Description  : Configure board initial setting.
*                This function is called by SystemInit() function in system_RE01_1500KB.c file.
*                This is reference to perform BoardInit process. Sample code of target is Evaluation Kit RE01_1500KB
*                 on Renesas. Please modify this function to suit your board.
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
void BoardInit(void)
{
    /*****   This function performs at beginning of start-up after released reset pin *****/
    /***** Please set pins here if your board is needed pins setting at the device start-up. ****/
    /*****   This function is suiting Evaluation Kit RE01_1500KB. Please change to your board pin setting *****/
    
    /* Handling of Unused ports (IOVCC domain) */
    /* PORT4 Setting: Evaluation Kit RE01_1500KB has DCDCs. Those are connect P404 and P405. */
    /* This perform to disable those output.                                     */
    /* Those are needed to enable when using EHC start up of this device. */ 
    /* Set P404 and P405 not to be used as DCDC_EN (output low: Disable) */

    /* PODR - Port Output Data
       b15-b0   PODR15 - PODR00     - Output Low Level */
    PORT4->PODR = 0x0000;

    /* PDR - Port Direction
       b15-b6   PDR15 - PDR06       - Input
       b5 -b4   PDR05 - PDR04       - Output
       b3 -b0   PDR03 - PDR00       - Input */
    PORT4->PDR  = 0x0030;

    /* Handling of Unused pors (AVCC1 domain) */

    /* PORT0 Setting */
    /* Set P009, P008 and P007 as LEDs (output high) */

    /* PODR - Port Output Data
       b15-b10  PODR15 - PODR10     - Output Low Level
       b9 -b7   PODR09 - PODR07     - Output High Level
       b6 -b0   PODR06 - PODR00     - Output Low Level */
    PORT0->PODR = 0x0380;

    /* PDR - Port Direction
       b15-b10  PDR15 - PDR10       - Input PORT
       b9 -b7   PDR09 - PDR07       - Output PORT
       b6 -b0   PDR06 - PDR00       - Input PORT */
    PORT0->PDR = 0x0380;

} /* End of function BoardInit */

/**********************************************************************************************************************
* Function Name: NMI_Handler
* Description  : NMI handler for User for template code when using EHC start-up.
*                This function is called when NMI interrupt is happen such as LVD, WDT and Oscillation stop detection.
*                This is must need to create when User uses Energy harvesting start-up(EHC).
*                (EHC is enable when SYSTEM_CFG_EHC_MODE of definition in r_core_cfg.h set "1".)
*                If use uses EHC, User must need to create process to initialize EHC circuit in this device
*                when detect voltage drop using LVD1.
*                This main.c is providing such a NMI Handler template code as default.
*
* Arguments    : none
* Return Value : none
**********************************************************************************************************************/
  void NMI_Handler( void ) 
{
  volatile uint16_t f_status;
  /* Read status flag for NMI interrupts */
  f_status = ICU->NMISR;

  if(0x0004 == (f_status & 0x0004))
  {
    /* When detect LVD1 */
    LVD_for_EHC();
    return; /* This return is executed when LVD1 detection was misdetection by any noise. */
  }



} /* End of function NMI_Handler() */

/**********************************************************************************************************************
* Function Name: LVD_for_EHC
* Description  : This function performs EHC initialization when it checks No misdetection of LVD.
*                This function checks this LVD is misdetection or not.
*                When this is not misdetection, it performs initialization of EHC.
*
* Arguments    : none
* Return Value : -1 : Error : This LVD detection is misdetection.
*                -  : When Properly , this is perform while loop after cut of power supply.
**********************************************************************************************************************/
void LVD_for_EHC()
{
  volatile uint16_t i;
#if SYSTEM_CFG_EHC_MODE == (1)
   /* Disable protect for LVD */
   R_SYS_RegisterProtectDisable(SYSTEM_REG_PROTECT_LVD);
   /* Clear LVD1 detection flag */
   SYSTEM->LVD1SR_b.DET = 0;            
   /* Clear LVD1 NMI interrpt flag */
   ICU->NMICLR_b.LVD1CLR = 1;            
   while(ICU->NMISR_b.LVD1ST != 0);
   /* Enable protect for LVD */
   R_SYS_RegisterProtectEnable(SYSTEM_REG_PROTECT_LVD);
   /**************************************************************************/
   /* Write user's code if you have any process to do before device power off.*/
   /**************************************************************************/
   /* Wait 100m second for stabilizetion of noise */
   R_SYS_SoftwareDelay(100, SYSTEM_DELAY_UNITS_MILLISECONDS);
   /* Check if LVD detected due to noise */ 
   for(i = 0; i < 3 ; i++)
   {
      if(0 != SYSTEM->LVD1SR_b.MON)
      {
        /* This LVD detection is misdetection. */
        /* VCC level is higher than LVD1det level. */
        /* So, This detection is error by any noise */
          return ;
       }
    }

    /*--------------------------------------------------------------------------
     * Set the protect function of the register
     *------------------------------------------------------------------------*/
    SYSTEM->PRCR = 0xA503U;
    
     /*--------------------------------------------------------------------------
     * Start the LOCO operation
     *------------------------------------------------------------------------*/
    SYSTEM->LOCOCR = 0x00U;     /* Start LOCO */

    /*--------------------------------------------------------------------------
     * Wait the LOCO clock stabilization
     *------------------------------------------------------------------------*/
    while(0x00 != SYSTEM->LOCOCR)
    {
        /* Retry the writing */
        SYSTEM->LOCOCR = 0x00U;
    }
    
    /*--------------------------------------------------------------------------
     * Set the system clock source
     * b2-b0 : [   CKSEL] System clock source select
     *                      - [010] LOCO selection
     *------------------------------------------------------------------------*/
    SYSTEM->SCKSCR = 0x02U;     /* Clock source : LOCO */
    while(0x02U != SYSTEM->SCKSCR)
    {
        /* Retry the writing */
        SYSTEM->SCKSCR = 0x02U;
    }
    
    /**** Enable Stop module clock for DTC/DMAC */
    SYSTEM->MSTPCRA_b.MSTPA22 = 1U;     /* Stop module clock for DTC/DMAC */
    
    /* Enable EHC Charging Capacitor Quick Wake-up function */
    /* This is must need for initializing EHC Circuit when using EHC mode. */
    SYSTEM->EHCCR1_b.QUICKMODE = 0U;
    
    while(1)
    {
        ;    /* loop */
    }
#endif /* SYSTEM_CFG_EHC_MODE == (1) */

} /*End of function of LVD_for_EHC() */


/* End of File */