/**
  ******************************************************************************
  * @file    entropy_hardware_poll.c
  *
  ******************************************************************************
  */

#include <string.h>
#include "platform.h"   // __LIT for all compilers
#ifdef RE01_256KB
    #include "RE01_256KB.h"
#else
    #include "RE01_1500KB.h"
#endif
#include "r_adc_api.h"
#include "mbedtls/entropy_poll.h"

void get_random_number(uint8_t *data, uint32_t len);

extern DRIVER_S14AD Driver_S14AD;
DRIVER_S14AD *gsp_adc_dev = &Driver_S14AD;

/******************************************************************************
Functions : hardware entropy collector(repeatedly called until enough gathered)
******************************************************************************/
int mbedtls_hardware_poll( void *data,
                           unsigned char *output, size_t len, size_t *olen )
{
    uint32_t random_number = 0;

    get_random_number((uint8_t *)&random_number, sizeof(uint32_t));
    *olen = 0;

    memcpy(output, &random_number, sizeof(uint32_t));
    *olen = sizeof(uint32_t);

    return 0;
}

/******************************************************************************
Functions : random number generator(XorShift method)
WARNING: The random number generation solution presented in this application is
    for demonstration purposes only. It is not recommended to go into production with
    the logic presented here. The current solution takes entropy from the a
    temperature sensor on the board and from the current system time. For
    production development, Renesas RX65x customers are recommended to use the
    TRNG included in the Trusted Secure IP Driver. Please see the following for more information
    on the Trusted Secure IP Driver: https://www.renesas.com/us/en/products/software-tools/software-os-middleware-driver/security-crypto/trusted-secure-ip-driver.html
******************************************************************************/
void get_random_number(uint8_t *data, uint32_t len)
{
    static uint32_t y = 2463534242;
    uint32_t res;
    uint32_t lp;
    uint8_t *bPtr;
    uint16_t temperature_data;
    st_adc_pins_t scanset_pin;

    gsp_adc_dev->Open(ADC_SINGLE_SCAN, 0x10, NULL);
    gsp_adc_dev->Control(AD_CMD_SET_SAMPLING_TEMP, (uint8_t*) 240);

    /** Channel Select */
    scanset_pin.an_chans = 0;
    scanset_pin.sensor   = ADC_MSEL_TEMP;
    gsp_adc_dev->ScanSet(ADC_GROUP_A, scanset_pin, ADC_TRIGER_SOFT);

    /* Start module */
    R_LPM_ModuleStart(LPM_MSTP_TEMPS);

#ifdef RE01_256KB
    /* enable temps */
    TSN->TSCR = 0x80;
#else
    /* enable temps */
    TEMPS->TSCR = 0x80;
#endif



    /** A/D Start */
    gsp_adc_dev->Start();
    R_SYS_SoftwareDelay(100, SYSTEM_DELAY_UNITS_MILLISECONDS);
    gsp_adc_dev->Read(ADC_SSEL_TEMP, &temperature_data);
    y += temperature_data;  /* randomness from internal temperature sensor */
    y += xTaskGetTickCount();   /* randomness from system timer */

    res = len / 4;
    for (lp = 0; lp < res; lp++)
    {
        y = y ^ (y << 13);
        y = y ^ (y >> 17);
        y = y ^ (y << 5);
        bPtr = (uint8_t*) & y;
#if __LIT
        *((uint32_t *)data) = (uint32_t)((*(bPtr + 3) << 24) | (*(bPtr + 2) << 16) | (*(bPtr + 1) << 8) | *(bPtr + 0));
#else
        *((uint32_t *)data) = y;
#endif
        data += 4;
    }
    y = y ^ (y << 13);
    y = y ^ (y >> 17);
    y = y ^ (y << 5);
    res = (uint32_t)len % 4;
    bPtr = (uint8_t*) & y;
    switch (res)
    {
        case 3:
#if __LIT
            *data++ = bPtr[3];
            *data++ = bPtr[2];
            *data++ = bPtr[1];
#else
            *data++ = bPtr[0];
            *data++ = bPtr[1];
            *data++ = bPtr[2];
#endif
            break;

        case 2:
#if __LIT
            *data++ = bPtr[3];
            *data++ = bPtr[2];
#else
            *data++ = bPtr[0];
            *data++ = bPtr[1];
#endif
            break;

        case 1:
#if __LIT
            *data++ = bPtr[3];
#else
            *data++ = bPtr[0];
#endif
            break;

        default:
            /* no op */
            break;
    }
}
