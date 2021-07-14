
#include <stdio.h>
#include <string.h>

#include "aws_application_version.h"
#include "bg96_common.h"
#include "bg96_driver.h"
#include "RE01_1500KB.h"

/***** define *****/

/***** Type *****/

/***** Functions *****/
void bg96_SysCondition_pwrkey_H (void);
void bg96_SysCondition_pwrkey_L (void);

/*******************************************************************************
 * Description  : BG96 PWRKEY H
 * Arguments    : None
 * Return Value : None
 *******************************************************************************/
void bg96_SysCondition_pwrkey_H (void)
{
	PORT3->PODR = 0x0000;
	PORT3->PDR = 0x0020;
}

/*******************************************************************************
 * Description  : BG96 PWRKEY L
 * Arguments    : None
 * Return Value : None
 *******************************************************************************/
void bg96_SysCondition_pwrkey_L (void)
{
	PORT3->PODR = 0x0020;
	PORT3->PDR = 0x0020;
}
