/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
* No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY
* LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR
* ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability 
* of this software. By using this software, you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2011, 2020 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : r_cg_serial_user.c
* Version      : CodeGenerator for RL78/G14 V2.05.05.01 [25 Nov 2020]
* Device(s)    : R5F104ML
* Tool-Chain   : CCRL
* Description  : This file implements device driver for Serial module.
* Creation Date: 2021/07/19
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver_sensor.h"
#include "r_cg_serial_sensor.h"
/* Start user code for include. Do not edit comment generated here */
#include "sw_delay.h"
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine_sensor.h"

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
#pragma interrupt r_iica0_interrupt(vect=INTIICA0)
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
extern volatile uint8_t   g_iica0_master_status_flag;  /* iica0 master flag */ 
extern volatile uint8_t   g_iica0_slave_status_flag;   /* iica0 slave flag */
extern volatile uint8_t * gp_iica0_rx_address;         /* iica0 receive buffer address */
extern volatile uint16_t  g_iica0_rx_cnt;              /* iica0 receive data length */
extern volatile uint16_t  g_iica0_rx_len;              /* iica0 receive data count */
extern volatile uint8_t * gp_iica0_tx_address;         /* iica0 send buffer address */
extern volatile uint16_t  g_iica0_tx_cnt;              /* iica0 send data count */
/* Start user code for global. Do not edit comment generated here */
void iica0_callback(MD_STATUS flag);
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: r_iica0_interrupt
* Description  : This function is INTIICA0 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void __near r_iica0_interrupt(void)
{
    if ((IICS0 & _80_IICA_STATUS_MASTER) == 0x80U)
    {
        iica0_master_handler();
    }
}

/***********************************************************************************************************************
* Function Name: iica0_master_handler
* Description  : This function is IICA0 master handler.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void iica0_master_handler(void)
{
    /* Control for communication */
    if ((0U == IICBSY0) && (g_iica0_tx_cnt != 0U))
    {
        r_iica0_callback_master_error(MD_SPT);
    }
    /* Control for sended address */
    else
    {
        if ((g_iica0_master_status_flag & _80_IICA_ADDRESS_COMPLETE) == 0U)
        {
            if (1U == ACKD0)
            {
                g_iica0_master_status_flag |= _80_IICA_ADDRESS_COMPLETE;
                
                if (1U == TRC0)
                {
                    WTIM0 = 1U;
                    
                    if (g_iica0_tx_cnt > 0U)
                    {
                        IICA0 = *gp_iica0_tx_address;
                        gp_iica0_tx_address++;
                        g_iica0_tx_cnt--;
                    }
                    else
                    {
                        r_iica0_callback_master_sendend();
                    }
                }
                else
                {
                    ACKE0 = 1U;
                    WTIM0 = 0U;
                    WREL0 = 1U;
                }
            }
            else
            {
                r_iica0_callback_master_error(MD_NACK);
            }
        }
        else
        {
            /* Master send control */
            if (1U == TRC0)
            {
                if ((0U == ACKD0) && (g_iica0_tx_cnt != 0U))
                {
                    r_iica0_callback_master_error(MD_NACK);
                }
                else
                {
                    if (g_iica0_tx_cnt > 0U)
                    {
                        IICA0 = *gp_iica0_tx_address;
                        gp_iica0_tx_address++;
                        g_iica0_tx_cnt--;
                    }
                    else
                    {
                        r_iica0_callback_master_sendend();
                    }
                }
            }
            /* Master receive control */
            else
            {
                if (g_iica0_rx_cnt < g_iica0_rx_len)
                {
                    *gp_iica0_rx_address = IICA0;
                    gp_iica0_rx_address++;
                    g_iica0_rx_cnt++;
                    
                    if (g_iica0_rx_cnt == g_iica0_rx_len)
                    {
                        ACKE0 = 0U;
                        WTIM0 = 1U;
                        WREL0 = 1U;
                    }
                    else
                    {
                        WREL0 = 1U;
                    }
                }
                else
                {
                    r_iica0_callback_master_receiveend();
                }
            }
        }
    }
}

/***********************************************************************************************************************
* Function Name: r_iica0_callback_master_error
* Description  : This function is a callback function when IICA0 master error occurs.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void r_iica0_callback_master_error(MD_STATUS flag)
{
    /* Start user code. Do not edit comment generated here */
    iica0_callback(flag);
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_iica0_callback_master_receiveend
* Description  : This function is a callback function when IICA0 finishes master reception.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void r_iica0_callback_master_receiveend(void)
{
    /* Start user code. Do not edit comment generated here */
    iica0_callback(MD_OK);
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_iica0_callback_master_sendend
* Description  : This function is a callback function when IICA0 finishes master transmission.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void r_iica0_callback_master_sendend(void)
{
    /* Start user code. Do not edit comment generated here */
    iica0_callback(MD_OK);
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
volatile static uint8_t busy;
static MD_STATUS status;
static uint8_t send_stop;

/*******************************************************************************************************************
* Name:       i2c_master_callback
* Function:   I2C callback routine
* Parameters: None
* Return:     None
*******************************************************************************************************************/
void iica0_callback(MD_STATUS flag)
{
	busy = FALSE;
	status = flag;
	if(TRUE == send_stop)
	{
		//R_IICA0_StopCondition();
	}
}

uint8_t write_device(uint8_t addr, uint8_t* data, uint8_t numBytes, uint8_t delay)
{
	status = MD_OK;

	/* Wait for previous transmission to complete */
	while(TRUE == busy);

	send_stop = TRUE;

    /* Send out the data */
	R_IICA0_Master_Send(addr, NULL, 0, delay);
	if(MD_OK == status)
	{
		busy = TRUE;
        while(TRUE == busy);
	}

    return(status);
}

uint8_t read_device(uint8_t addr, uint8_t* data, uint8_t numBytes, uint8_t delay)
{
	/* Wait for previous transmission to complete */
	while(TRUE == busy);

	send_stop = TRUE;

	addr = addr & 0xFFU;

  /* Read the data */
	R_IICA0_Master_Receive(addr, (uint8_t*)&data[0], (uint16_t)numBytes, delay);
	if(MD_OK == status)
	{
		busy = TRUE;
		while(TRUE == busy);
	}

	return(status);
}

int8_t write(uint8_t addr, uint8_t startReg, uint8_t* data, uint8_t numBytes)
{
	uint8_t writeRegister_writeData[64];
	uint16_t itr = 0;

	/* Wait for previous transmission to complete */
	while(TRUE == busy);

	send_stop = TRUE;

	/*writes data from an array beginning at the startReg*/
    writeRegister_writeData[0] = startReg;

    for(itr = 0; itr < numBytes; itr++)
    {
      writeRegister_writeData[itr + 1] = data[itr];
    }

    /* Send out the data */
	R_IICA0_Master_Send(addr, &writeRegister_writeData[0], ((uint8_t)numBytes+1), 1);
	busy = TRUE;
	while(TRUE == busy);

    return(status);
}

int8_t read(uint8_t addr, uint8_t startReg, uint8_t* data, uint8_t numBytes)
{
	uint8_t readBlock_writeData[64];
	status = MD_OK;

	/* Wait for previous transmission to complete */
	while(TRUE == busy);

	/* Don't send the stop condition */
	send_stop = FALSE;

	addr = addr & 0xFFU;

	readBlock_writeData[0] = startReg;

	/* Send out the Register to start from */
	R_IICA0_Master_Send(addr, &readBlock_writeData[0], 1, 1);
	if(MD_OK == status)
	{
		busy = TRUE;
		while(TRUE == busy);
	}

	if(MD_OK != status)
	{
		return status;
	}

	/* Read the data */
	R_IICA0_Master_Receive(addr, (uint8_t*)&data[0], (uint16_t)numBytes, 10);
	if(MD_OK == status)
	{
		busy = TRUE;
		while(TRUE == busy);
		//R_IICA0_StopCondition();
	}

	/* Allow send*/
	send_stop = TRUE;

	return(status);
}
/* End user code. Do not edit comment generated here */
