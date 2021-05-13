
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
// changed 2020/10 start
// BG96 への接続用に PE01 汎用ポートを割り当てる場合、修正が必要
// 現行は、コメントアウト
/*
    PORTE.PODR.BIT.B0 = 0;  // PWRKEY H
    PORTE.PDR.BIT.B0 = 1;   // OUT
*/
// changed 2020/10 end
	PORT3->PODR = 0x0000;
	PORT3->PDR = 0x0020;
//	PORT3->PODR.BIT.B5 = 0;
//	PORT3->PDR.BIT.B5 = 1;
}

/*******************************************************************************
 * Description  : BG96 PWRKEY L
 * Arguments    : None
 * Return Value : None
 *******************************************************************************/
void bg96_SysCondition_pwrkey_L (void)
{
// changed 2020/10 start
// BG96 への接続用に PE01 汎用ポートを割り当てる場合、修正が必要
// 現行は、コメントアウト
/*
    PORTE.PODR.BIT.B0 = 1;  // PWRKEY L
    PORTE.PDR.BIT.B0 = 1;   // OUT
*/
// changed 2020/10 end
//	PORT3->PODR->BIT.B5 = 1;
//	PORT3->PDR.BIT.B5 = 1;
	PORT3->PODR = 0x0020;
	PORT3->PDR = 0x0020;
}
