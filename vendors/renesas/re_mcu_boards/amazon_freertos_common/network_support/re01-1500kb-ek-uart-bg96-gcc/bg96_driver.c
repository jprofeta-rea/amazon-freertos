#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "FreeRTOSIPConfig.h"
#include "platform.h"

#include "r_byteq_if.h"
#include "bg96_driver.h"
#include "bg96_common.h"

#include "r_system_api.h"
#include "R_Driver_USART.h"

#define SCI_TX_BUSIZ_DEFAULT                    (1460)
#define UART_BUS_CHANGE_SPEED          (115200)              /* UART bus speed(bps) */

const uint8_t bg96_return_text_ok[]          = BG96_RETURN_TEXT_OK;
const uint8_t bg96_return_text_error[]       = BG96_RETURN_TEXT_ERROR;
const uint8_t bg96_return_text_ready[]       = BG96_RETURN_TEXT_READY;
const uint8_t bg96_return_text_ok_go_send[]  = BG96_RETURN_TEXT_OK_GO_SEND;
const uint8_t bg96_return_text_send_byte[]   = BG96_RETURN_TEXT_SEND_BYTE;
const uint8_t bg96_return_text_send_ok[]     = BG96_RETURN_TEXT_SEND_OK;
const uint8_t bg96_return_text_send_fail[]   = BG96_RETURN_TEXT_SEND_FAIL;

const uint8_t bg96_socket_status_closed[]       = BG96_SOCKET_STATUS_TEXT_CLOSED;
const uint8_t bg96_socket_status_socket[]       = BG96_SOCKET_STATUS_TEXT_SOCKET;
const uint8_t bg96_socket_status_bound[]        = BG96_SOCKET_STATUS_TEXT_BOUND;
const uint8_t bg96_socket_status_listen[]       = BG96_SOCKET_STATUS_TEXT_LISTEN;
const uint8_t bg96_socket_status_connected[]    = BG96_SOCKET_STATUS_TEXT_CONNECTED;

const uint8_t bg96_return_dummy[]   = "";

const uint8_t * const bg96_result_code[BG96_RETURN_ENUM_MAX][BG96_RETURN_STRING_MAX] =
{
	/* text mode*/                  /* numeric mode */
	{bg96_return_text_ok,},
	{bg96_return_text_error,},
	{bg96_return_text_ready,},
	{bg96_return_text_ok_go_send,},
	{bg96_return_text_send_byte,},
	{bg96_return_text_send_ok,},
	{bg96_return_text_send_fail,},
};

const uint8_t * const bg96_socket_status[BG96_SOCKET_STATUS_MAX] =
{
	bg96_socket_status_closed,
	bg96_socket_status_socket,
	bg96_socket_status_bound,
	bg96_socket_status_listen,
	bg96_socket_status_connected,
};

bg96_system_status_t g_bg96_system_state = BG96_SYSTEM_CLOSE;

volatile uint8_t current_socket_index;

uint8_t buff[1000];
uint8_t recvbuff[2048+20];

uint8_t   g_bg96_sci_err;

volatile uint32_t g_bg96_uart_teiflag[CREATEABLE_SOCKETS];
static uint8_t timeout_overflow_flag[CREATEABLE_SOCKETS];

static TickType_t starttime[CREATEABLE_SOCKETS], thistime[CREATEABLE_SOCKETS], endtime[CREATEABLE_SOCKETS], remaintime[CREATEABLE_SOCKETS];
static TickType_t startbytetime[CREATEABLE_SOCKETS], thisbytetime[CREATEABLE_SOCKETS], endbytetime[CREATEABLE_SOCKETS];
static uint8_t byte_timeout_overflow_flag[CREATEABLE_SOCKETS];

uint8_t g_bg96_return_mode;
static uint8_t flag_ = true;

byteq_hdl_t socket_byteq_hdl[CREATEABLE_SOCKETS];

uint32_t socket_recv_error_count[CREATEABLE_SOCKETS];
extern bg96_socket_t g_bg96_socket[WIFI_CFG_CREATABLE_SOCKETS];

uint8_t debug_out_crlf;

static void bg96_uart_callback_command_port(uint32_t event);

static void bg96_uart_callback_data_port(void *pArgs);
static void timeout_init(uint8_t socket_no, uint16_t timeout_ms);
static void bytetimeout_init(uint8_t socket_no, uint16_t timeout_ms);
static int32_t check_timeout_and_remain(uint8_t socket_no, int32_t rcvcount, uint16_t timeout_ms);
static int32_t check_timeout(uint8_t socket_no, int32_t rcvcount);
static int32_t check_bytetimeout(uint8_t socket_no, int32_t rcvcount);
static void timeout_only_wait(uint8_t socket_no, uint16_t timeout_ms);
static int32_t bg96_serial_open(uint32_t);

static int32_t bg96_serial_data_port_open(void);
static int32_t bg96_serial_send_basic(uint8_t serial_ch_id, uint8_t *ptextstring, uint16_t response_type, uint16_t timeout_ms, bg96_return_code_t expect_code);
static int32_t bg96_change_socket_index(uint8_t socket_no);
static TickType_t g_sl_bg96_tcp_recv_timeout = 3000;		/* ## slowly problem ## unit: 1ms */
static int32_t bg96_take_mutex(uint8_t mutex_flag);
static void bg96_give_mutex(uint8_t mutex_flag);
static int32_t SetupCommunication(int32_t Change_Baudrate);
static int32_t bg96_serial_send_recv_test(uint8_t serial_ch_id, uint8_t *ptextstring, uint16_t response_type, uint16_t timeout_ms, bg96_return_code_t expect_code);
static int32_t bg96_serial_response_test(int32_t recvcnt, bg96_return_code_t expect_code);
uint8_t g_wifi_cleateble_sockets = WIFI_CFG_CREATABLE_SOCKETS;
#define MUTEX_TX (1 << 0)
#define MUTEX_RX (1 << 1)

/**
 * @brief The global mutex to ensure that only one operation is accessing the
 * g_bg96_semaphore flag at one time.
 */
static SemaphoreHandle_t g_bg96_semaphore = NULL;
static SemaphoreHandle_t g_bg96_tx_semaphore = NULL;
static SemaphoreHandle_t g_bg96_rx_semaphore = NULL;

/**
 * @brief Maximum time in ticks to wait for obtaining a semaphore.
 */
static const TickType_t xMaxSemaphoreBlockTime = pdMS_TO_TICKS( 60000UL );

static uint8_t g_bg96_cgatt_flg;
extern ARM_DRIVER_USART Driver_USART5;
static ARM_DRIVER_USART *gsp_sci5_dev = &Driver_USART5;
wifi_err_t R_CELLULAR_BG96_SocketShutdown (int32_t socket_no);
uint32_t dummy_len = 0;
void reset_button(void);

int32_t bg96_wifi_init(void)
{
	int32_t ret;
	int32_t tries_connect;
	uint32_t cgatt_cnt = 0;
    uint32_t recvlen = 0;
	uint8_t atcmd[128];

	if( BG96_SYSTEM_CLOSE != g_bg96_system_state)
	{
		return WIFI_ERR_ALREADY_OPEN;
	}
    g_bg96_cgatt_flg = 0;

    ret = SetupCommunication(UART_BUS_CHANGE_SPEED);

    if (ret !=0)
    {
    	return ret;
    }

    // APN set
    memset(buff, 0x00, sizeof(buff));
	sprintf(buff, "AT+QICSGP=1,1,\"%s\",\"%s\",\"%s\",1\r", CELLULAR_APN, CELLULAR_APN_USERID, CELLULAR_APN_PASSWORD);
	ret = bg96_serial_send_basic(BG96_UART_COMMAND_PORT, buff, 6, 4000, BG96_RETURN_OK);
    cgatt_cnt = 0;
    while(1)
    {

    	ret = bg96_serial_send_basic(BG96_UART_COMMAND_PORT, "AT+CGATT?\r", 19, 400, BG96_RETURN_OK);

            recvlen = strlen((const char *)recvbuff);
            if (recvlen > 13)
            {
                if (0 == memcmp((const char *)recvbuff, "\r\n+CGATT: 1\r\n", 13))
                {
                    break;
                }
                else
                {
                    cgatt_cnt++;
                }
            }
            else
            {
                cgatt_cnt++;
            }



        // timeout
        if (cgatt_cnt > BG96_RETRY_GATT)
        {
            ret = -1;
            break;
        }

        timeout_only_wait(1, 500);
    }
    if(ret != 0)
    {
        bg96_power_down(0);

        return ret;
    }

    bg96_socket_init();

    ret = bg96_serial_send_basic(BG96_UART_COMMAND_PORT, "AT+QIACT=1\r", 6, 4000, BG96_RETURN_OK);

    vStart_bg96_recv_task();

    g_bg96_cgatt_flg = 1;

    g_bg96_system_state = BG96_SYSTEM_CONNECT;

    // error handling T.B.D.

    return ret;
}

int32_t R_CELLULAR_BG96_SocketCreate(uint32_t type, uint32_t ipversion)
{
	int32_t i;
	int32_t ret = WIFI_ERR_SOCKET_CREATE;
	uint8_t mutex_flag;
	wifi_err_t api_ret = WIFI_SUCCESS;

	if(type != WIFI_SOCKET_IP_PROTOCOL_TCP || ipversion != WIFI_SOCKET_IP_VERSION_4)
	{
		return WIFI_ERR_PARAMETER;
	}
	if( 0 != R_CELLULAR_BG96_IsConnected())
	{
		return WIFI_ERR_NOT_CONNECT;
	}

    mutex_flag = (MUTEX_TX | MUTEX_RX);
    if(0 != bg96_take_mutex(mutex_flag))
	{
    	api_ret = WIFI_ERR_TAKE_MUTEX;
	}

    if(WIFI_SUCCESS == api_ret)
    {
        for(i=0;i<g_wifi_cleateble_sockets;i++)
        {
            if(g_bg96_socket[i].socket_create_flag == 0)
            {
                break;
            }
        }
        if(i >=g_wifi_cleateble_sockets)
        {
            api_ret = WIFI_ERR_SOCKET_NUM;
        }
        if(WIFI_SUCCESS == api_ret)
        {
            g_bg96_socket[i].socket_create_flag = 1;
            g_bg96_socket[i].ipversion = ipversion;
            g_bg96_socket[i].protocol = type;
            g_bg96_socket[i].socket_status = WIFI_SOCKET_STATUS_SOCKET;
            g_bg96_socket[i].ssl_flag = 0;
            g_bg96_socket[i].ssl_type = 0;
            R_BYTEQ_Flush(g_bg96_socket[i].socket_byteq_hdl);
            g_bg96_socket[i].qiurc_rcv_flg = 0;
            ret = i;
        }
        else
        {
            ret = WIFI_ERR_SOCKET_CREATE;
        }

        /* Give back the socketInUse mutex. */
        bg96_give_mutex(mutex_flag);
    }

    return ret;
}


int32_t bg96_tcp_connect(uint8_t socket_no, uint32_t ipaddr, uint16_t port)
{
	int32_t ret = -1;
    uint8_t mutex_flag;
	wifi_err_t api_ret = WIFI_SUCCESS;

	if( (0 == ipaddr) || (0 == port) )
	{
		return  WIFI_ERR_PARAMETER;
	}
	if( (socket_no >= WIFI_CFG_CREATABLE_SOCKETS) || (socket_no < 0) || (g_bg96_socket[socket_no].socket_status != WIFI_SOCKET_STATUS_SOCKET) )
	{
		return  WIFI_ERR_SOCKET_NUM;
	}
	if( 0 != R_CELLULAR_BG96_IsConnected())
	{
		return WIFI_ERR_NOT_CONNECT;
	}

    mutex_flag = (MUTEX_TX | MUTEX_RX);
    if(0 != bg96_take_mutex(mutex_flag))
	{
		api_ret = WIFI_ERR_TAKE_MUTEX;
	}
	if(WIFI_SUCCESS == api_ret)
	{
		if(	g_bg96_socket[socket_no].socket_create_flag == 0)
		{
			/* Give back the socketInUse mutex. */
			bg96_give_mutex(mutex_flag);
			return -1;
		}

        strcpy((char *)buff, "AT+QIOPEN=1,");
        sprintf((char *)buff + strlen((char *)buff), "%d,\"TCP\",\"%d.%d.%d.%d\",%d,0,0\r", socket_no, (uint8_t)(ipaddr >> 24), (uint8_t)(ipaddr >> 16), (uint8_t)(ipaddr >> 8), (uint8_t)(ipaddr), port);
		ret = bg96_serial_send_with_recvtask(BG96_UART_COMMAND_PORT, buff, 3, 4000, BG96_RETURN_OK, BG96_SET_CIPSTART, socket_no, &dummy_len, 0);

		/* Give back the socketInUse mutex. */
        bg96_give_mutex(mutex_flag);
    }
    else
    {
    	return -1;
    }
    if (ret == 0)
    {
		g_bg96_socket[socket_no].socket_status = WIFI_SOCKET_STATUS_CONNECTED;
#if DEBUGLOG == 1
		R_BSP_CpuInterruptLevelWrite (14);
		printf("connected(%d)\r\n", socket_no);
		R_BSP_CpuInterruptLevelWrite (0);
#endif
    }

	return ret;

}

int32_t bg96_tcp_send(uint8_t socket_no, uint8_t *pdata, int32_t length, uint32_t timeout_ms)
{
	int32_t timeout;
	volatile int32_t sended_length;
	int32_t ret;
	uint8_t ercd;
	uint8_t result;
    uint32_t value;
    uint8_t mutex_flag;
    uint32_t lenghttmp1;

    ARM_USART_STATUS sci_status_tx;

	if( (pdata == NULL) || (length < 0 ))
	{
		return  WIFI_ERR_PARAMETER;
	}

	if( 0 != R_CELLULAR_BG96_IsConnected())
	{
		return WIFI_ERR_NOT_CONNECT;
	}

	if( (socket_no >= WIFI_CFG_CREATABLE_SOCKETS) || (socket_no < 0) || (0 == g_bg96_socket[socket_no].socket_create_flag) || (g_bg96_socket[socket_no].socket_status != WIFI_SOCKET_STATUS_CONNECTED) )
	{
		return  WIFI_ERR_SOCKET_NUM;
	}

    mutex_flag = MUTEX_TX;
    if( 0 == bg96_take_mutex(mutex_flag) )
    {
    	if(0 == g_bg96_socket[socket_no].socket_create_flag)
    	{
			/* Give back the socketInUse mutex. */
			bg96_give_mutex(mutex_flag);
			return -10;
    	}
		sended_length = 0;
		lenghttmp1 = SCI_TX_BUSIZ_DEFAULT;

		while(sended_length < length)
		{
			if((length - sended_length) > SCI_TX_BUSIZ_DEFAULT)
			{
				lenghttmp1 = SCI_TX_BUSIZ_DEFAULT;
			}
			else
			{
				lenghttmp1 = (length - sended_length);
			}

			timeout_init(socket_no, timeout_ms);

            memset(buff, 0x00, sizeof(buff));
			strcpy((char *)buff,"AT+QISEND=");
			sprintf((char *)buff+strlen((char *)buff),"%d,%d\r",socket_no,lenghttmp1);
			ret = bg96_serial_send_with_recvtask(BG96_UART_COMMAND_PORT, buff, 3, remaintime[socket_no], BG96_RETURN_OK_GO_SEND, BG96_SET_CIPSEND, socket_no, &dummy_len, 0);
			if(ret != 0)
			{
				/* Give back the socketInUse mutex. */
				bg96_give_mutex(mutex_flag);
				__NOP();
				return -1;
			}

			timeout_init(socket_no, 3000);
			timeout = 0;

            bg96_response_set_queue( BG96_SET_CIPSEND_END, socket_no );
			g_bg96_uart_teiflag[BG96_UART_COMMAND_PORT] = 0;
			ercd = gsp_sci5_dev->Send(pdata+sended_length, lenghttmp1);

			if(ARM_DRIVER_OK != ercd)
			{
				/* Give back the socketInUse mutex. */
				bg96_give_mutex(mutex_flag);
				return -2;
			}
			sci_status_tx = gsp_sci5_dev->GetStatus();

			while (sci_status_tx.tx_busy == 1 )
			{
			    sci_status_tx = gsp_sci5_dev->GetStatus();
				if(-1 == check_timeout(socket_no, 0))
				{
					/* Give back the socketInUse mutex. */
					bg96_give_mutex(mutex_flag);
					return 0;
				}
			}
			while(1)
			{
				if(0 != g_bg96_uart_teiflag[BG96_UART_COMMAND_PORT])
				{
					break;
				}
				if(-1 == check_timeout(socket_no, 0))
				{
					timeout = 1;
					break;
				}
			}
			if(timeout == 1 )
			{
				/* Give back the socketInUse mutex. */
				bg96_give_mutex(mutex_flag);
				return 0;
			}

			timeout_init(socket_no, 4000);

			while(1)
			{
                ercd = bg96_response_get_queue( BG96_SET_CIPSEND_END, socket_no, &result, &value);
				if(0 == ercd)
				{
					break;
				}
				if(-1 == check_timeout(socket_no, 0))
				{
					timeout = 1;
					break;
				}
			}

			if(result != BG96_RETURN_SEND_OK)
			{
				/* Give back the socketInUse mutex. */
				bg96_give_mutex(mutex_flag);
				__NOP();
				return -4;
			}

			sended_length += lenghttmp1;

		}
		/* Give back the socketInUse mutex. */
		bg96_give_mutex(mutex_flag);
    }
    else
    {
    	return -5;
    }
	return sended_length;
}

int32_t bg96_tcp_recv(uint8_t socket_no, uint8_t *pdata, int32_t length, uint32_t timeout_ms)
{
    uint8_t ercd;
    uint32_t recvcnt = 0;
    int scanf_ret;
    uint32_t recv_socket_id;
    uint32_t recv_length;
    uint32_t stored_len;
    uint32_t i;
    volatile int32_t timeout;
    int32_t ret;
    byteq_err_t byteq_ret;
    int32_t recvlen;
    uint32_t len;
    int32_t allrecv_len = 0;
    uint16_t tmpcnt = 0;
    uint8_t bytedata;
    uint8_t pdata_l[32];

    uint8_t mutex_flag;

	if( (pdata == NULL) || (length <= 0 ))
	{
		return  WIFI_ERR_PARAMETER;
	}

	if( 0 != R_CELLULAR_BG96_IsConnected())
	{
		return WIFI_ERR_NOT_CONNECT;
	}

	if( (socket_no >= WIFI_CFG_CREATABLE_SOCKETS) || (socket_no < 0) || (0 == g_bg96_socket[socket_no].socket_create_flag) ||  (g_bg96_socket[socket_no].socket_status != WIFI_SOCKET_STATUS_CONNECTED) )
	{
		return  WIFI_ERR_SOCKET_NUM;
	}
        /*Buffer Access Mode*/
    while(length > allrecv_len)
    {
    	while (1)
        {
            byteq_ret = R_BYTEQ_Get(g_bg96_socket[socket_no].socket_byteq_hdl, &bytedata);

            if(BYTEQ_SUCCESS == byteq_ret)
            {
                *(pdata + allrecv_len) = bytedata;
                allrecv_len++;

                if (allrecv_len >= length)
                {
                    break;
                }
            }
            else
            {
                break;
            }

        }

        if (allrecv_len >= length)
        {
            break;
        }
        timeout_init(socket_no, timeout_ms);
        timeout = 0;
        while(1)
        {
            if (g_bg96_socket[socket_no].qiurc_rcv_flg == 1)
            {
                break;
            }
            if(-1 == check_timeout_and_remain(socket_no, 0, timeout_ms))
            {
                timeout = 1;
                break;
            }
            vTaskDelay( 1 );
        }

        if (timeout == 1)
        {
            return allrecv_len;
        }

        mutex_flag = MUTEX_RX;
        if( 0 == bg96_take_mutex(mutex_flag) )
        {
            memset(pdata_l, 0x00, sizeof(pdata_l));
            tmpcnt++;
            recvlen = length - allrecv_len;
            if(recvlen > 1500)
            {
                recvlen = 1500;
            }

            memset(buff, 0x00, sizeof(buff));
            sprintf((char *)buff,"AT+QIRD=%d,%d\r",socket_no, recvlen);
            len = 0;
            ret = bg96_serial_send_with_recvtask(BG96_UART_COMMAND_PORT, buff, 3, remaintime[socket_no], BG96_RETURN_OK, BG96_SET_QIRD, socket_no, &len, 0);

            if(ret != 0)
            {
                /* Give back the socketInUse mutex. */
                bg96_give_mutex(mutex_flag);
                __NOP();
                if (ret == -2)
                {
                    ret = 0;
                }
                return ret;
            }

            if (len == 0)
            {

            }
            else
            {
                timeout_init(socket_no, 4000); //[RE01-test] change 3000 to 4000
                timeout = 0;

                stored_len = 0;
                while(1)
                {
                    byteq_ret = R_BYTEQ_Get(g_bg96_socket[socket_no].socket_byteq_hdl, &bytedata);

                    if(BYTEQ_SUCCESS == byteq_ret)
                    {
                        *(pdata + stored_len + allrecv_len) = bytedata;
                        stored_len++;

                        if (stored_len >= len)
                        {
                            break;
                        }
                        else if (stored_len >= recvlen)
                        {
                    	    break;
                        }
                    }
                    if(-1 == check_timeout(socket_no, 0))
                    {
                        timeout = 1;
                        break;
                    }
                }

                if(timeout == 1)
                {
                    /* Give back the socketInUse mutex. */
                    bg96_give_mutex(mutex_flag);
                    break;
                }
                allrecv_len += stored_len;
            }

            /* Give back the socketInUse mutex. */
            bg96_give_mutex(mutex_flag);
        }
        else
        {
            // mutex error
            return -1;
        }
    }

    return allrecv_len;
}

int32_t bg96_serial_tcp_recv_timeout_set(uint8_t socket_no, TickType_t timeout_ms)
{
	g_sl_bg96_tcp_recv_timeout = timeout_ms;
	return 0;
}

int32_t bg96_tcp_disconnect(uint8_t socket_no)
{
	int32_t ret = 0;
    uint8_t mutex_flag;
    wifi_err_t api_ret = WIFI_SUCCESS;

    mutex_flag = (MUTEX_TX | MUTEX_RX);
    if(0 != bg96_take_mutex(mutex_flag))
	{
		api_ret = WIFI_ERR_TAKE_MUTEX;
	}
	if(WIFI_SUCCESS == api_ret)
	{
		if(1 == g_bg96_socket[socket_no].socket_create_flag)
		{
            memset(buff, 0x00, sizeof(buff));
			sprintf((char *)buff,"AT+QICLOSE=%d\r",socket_no);
			ret = bg96_serial_send_with_recvtask(BG96_UART_COMMAND_PORT, buff, 3, 10000, BG96_RETURN_OK, BG96_SET_CIPCLOSE, socket_no, &dummy_len, 0);
			if(0 == ret)
			{
				g_bg96_socket[socket_no].socket_create_flag = 0;
			}
			/* Give back the socketInUse mutex. */
			bg96_give_mutex(mutex_flag);
		}
		else
		{
			/* Give back the socketInUse mutex. */
			bg96_give_mutex(mutex_flag);
			return -1;
		}
    }
    else
    {
    	return -1;
    }
	return ret;

}

int32_t bg96_dns_query(uint8_t *ptextstring, uint32_t *ulipaddr)
{
	uint32_t result;
	int32_t func_ret;
	int32_t scanf_ret;
    uint8_t mutex_flag;
    wifi_err_t api_ret = WIFI_SUCCESS;

    mutex_flag = (MUTEX_TX | MUTEX_RX);
    if(0 != bg96_take_mutex(mutex_flag))
	{
		api_ret = WIFI_ERR_TAKE_MUTEX;
	}
	if(WIFI_SUCCESS == api_ret)
	{
        memset(buff, 0x00, sizeof(buff));
        strcpy((char *)buff,"AT+QIDNSGIP=1,\"");
        sprintf((char *)buff+strlen((char *)buff),"%s\"\r",ptextstring);
        func_ret = bg96_serial_send_with_recvtask(BG96_UART_COMMAND_PORT, buff, 3, 20000, BG96_RETURN_OK, BG96_SET_CIPDOMAIN, 0xff, &dummy_len, 0);
        if(func_ret != 0)
        {
            return -1;
        }
        *ulipaddr = (((uint32_t)dnsaddress[0]) << 24) | (((uint32_t)dnsaddress[1]) << 16) | (((uint32_t)dnsaddress[2]) << 8) | ((uint32_t)dnsaddress[3]);

		bg96_give_mutex(mutex_flag);
    }

	return 0;
}


static int32_t bg96_serial_send_basic(uint8_t serial_ch_id, uint8_t *ptextstring, uint16_t response_type, uint16_t timeout_ms, bg96_return_code_t expect_code)
{
	volatile int32_t timeout;
	volatile uint32_t ercd;
	volatile ARM_USART_STATUS sci_status_tx;
	volatile ARM_USART_STATUS sci_status_rx;
	volatile uint32_t recvcnt = 0;
	volatile int32_t ret;
	memset(recvbuff,0,sizeof(recvbuff));
	timeout_init(serial_ch_id, timeout_ms);
	if(ptextstring != NULL)
	{
		timeout = 0;
		recvcnt = 0;
		g_bg96_uart_teiflag[serial_ch_id] = 0;
		ercd = gsp_sci5_dev->Send(ptextstring, strlen((const char *)ptextstring));
		if(ARM_DRIVER_OK != ercd)
		{
			return -1;
		}

        sci_status_tx = gsp_sci5_dev->GetStatus();

        while (sci_status_tx.tx_busy == 1 )
        {
            sci_status_tx = gsp_sci5_dev->GetStatus();

            if(-1 == check_timeout(serial_ch_id, recvcnt))
            {
            	return -1;
            }
        }
		while(1)
		{
			if(0 != g_bg96_uart_teiflag[serial_ch_id])
			{
				break;
			}
			if(-1 == check_timeout(serial_ch_id, recvcnt))
			{
				timeout = 1;
				break;
			}
		}
		if(timeout == 1)
		{
			return -1;
		}
	}
	while(1)
	{
		ercd = gsp_sci5_dev->Receive(&recvbuff[recvcnt], response_type);
		if(ARM_DRIVER_OK == ercd)
		{
			bytetimeout_init(serial_ch_id, response_type);
			sci_status_rx = gsp_sci5_dev->GetStatus();
			while (sci_status_rx.rx_busy == 1)
			{
				if(-1 == check_timeout(serial_ch_id, 0))
					{
						flag_ = false;
					}
				if(-1 == check_bytetimeout(serial_ch_id, recvcnt))
				{
					ret = -1;
					flag_ = true;
					break;
//			    	return -1;break
				}
		    }
			recvcnt = (uint32_t )response_type;
		}
		else
		{
			return -1;
		}

		if (recvcnt >= (uint32_t )response_type)
		{
			break;
		}
	}
	if(timeout == 1)
	{
		return -1;
	}

	/* Response data check */
	ret = -1;
	recvbuff[recvcnt] = '\0';
	if(recvcnt >= strlen((const char *)bg96_result_code[expect_code][0]))
	{
		if(0 == strncmp((const char *)&recvbuff[recvcnt - strlen((const char *)bg96_result_code[expect_code][0]) ],
				(const char *)bg96_result_code[expect_code][0],
				strlen((const char *)bg96_result_code[expect_code][0])))
		{
			ret = 0;
		}
	}

	return ret;
}
/* Setup  */
static int32_t SetupCommunication(int32_t Change_Baudrate)
{
	int32_t tries, ret,index;

	uint32_t k,state;
	uint32_t baudrate;
	/* */
	const uint32_t BR[] = {115200,460800,921600,9600,110,300,600,1200,2400,4800,9600,14400,19200,38400,57600,230400};
	k     =  0;
	state =  0;
	tries = 0;

	baudrate = BR[k];

	while(k <= (sizeof(BR)/sizeof(BR[0])))
	{
	  switch (state)
	  {
	  case 0:
	  case 1:
	  case 2:
	  {
		  /*Reset BG96 Dragino module */
		  reset_button();
		  /*Initialize uart RE01 1.5MB */
		  ret = bg96_serial_open(baudrate);
		  if(ret != 0)
		  	{
		  		return ret;
		  	}
		  while (1)
		  {
			  if (-1 == check_timeout(1, 0))
			  {
				  ret = 0;
				  break;
			  }
		  }
		  /*Test the sending and receving with BG96 Dragino */
		  ret = bg96_serial_send_recv_test(BG96_UART_COMMAND_PORT, "ATE0\r", 6, 400, BG96_RETURN_OK);
		 	break;
	  }
	  default:
		  	  ret = -1;
	          break;

	  }
	  /*Check result of the testing for the sending and receving with BG96 Dragino */
	  if (ret == 0)
		  {
			  vTaskDelay(500);
			  /*Check response from BG96 Dragino */
			  ret = bg96_serial_response_test(6, BG96_RETURN_OK);
			  /*Response from BG96 Dragino is OK. That means the connection between RE01 1.5MB and Draguino is successful */
			  if (ret == 0)
			  {
				  /*Check recent baudrate and changing baudrate */
				  if(Change_Baudrate != BR[k])
				  {
					  /*The baudrate is not same -> send AT cmd to change baudrate */
					  memset(buff, 0x00, sizeof(buff));
					  sprintf((char *)buff,"AT+IPR=%d;&W\r",Change_Baudrate);
					  /*Send AT cmd to change baudrate */
					  ret = bg96_serial_send_basic(BG96_UART_COMMAND_PORT, buff, 6, 400, BG96_RETURN_OK);
					  /* Return state 2 */
					  state = 2;
					  /*Check the sending AT cmd to change baudrate */
					  if (ret == 0)
					  {
						  /*Verify changed baudrate to re-initialize uart RE01 1.5MB*/
						  baudrate = Change_Baudrate;
						  for(index = 0; index < (sizeof(BR)/sizeof(BR[0]));index++ )
						  {
							  if (BR[index]==Change_Baudrate)
							  {
								  k = index;
								  break;
							  }
						  }

					  }
					  else
					  /*Check the sending AT cmd to change baudrate is failed. Test next baudrate in array BR[]*/
					  {
						  k++;
						  baudrate = BR[k];

					  }
					  bg96_serial_close();


				  }
				  else
				  {
					  /*Recent baudrate and changing baudrate are same, exit loop*/
					  return ret;
				  }

			  }
			  else
			  {
				  /*Response from BG96 Dragino is failed. Try 3 times to get send and get respone*/
				  tries ++;
				  state = 2;
				  if (tries > 3)
				  {
					  ret = -1;
					  break;
				  }
			  }
		  }
		  else
		  {
			  /* Initialize k-th baudrate in array BR  */
			  state = 2;
			  k++;
			  baudrate = BR[k];
			  bg96_serial_close();
		  }
	  }
	return ret;
}

int32_t bg96_serial_send_with_recvtask(uint8_t serial_ch_id, uint8_t *ptextstring, uint16_t response_type, uint16_t timeout_ms, bg96_return_code_t expect_code,  uint8_t command, uint8_t socket_no, uint32_t *length, uint8_t delay_flag)
{
	volatile int32_t timeout;
	uint8_t ercd;
	ARM_USART_STATUS sci_status_tx;
	uint32_t recvcnt = 0;
	int32_t ret;
	uint8_t result;
    uint32_t len;

    bytetimeout_init(serial_ch_id, timeout_ms);

	if(ptextstring != NULL)
	{
		timeout = 0;
		recvcnt = 0;

		bg96_response_set_queue( command, socket_no );
		g_bg96_uart_teiflag[serial_ch_id] = 0;
		ercd = gsp_sci5_dev->Send(ptextstring, strlen((const char *)ptextstring));
	    if(ARM_DRIVER_OK != ercd)
		{
			return -1;
		}
	    sci_status_tx = gsp_sci5_dev->GetStatus();

	    while (sci_status_tx.tx_busy == 1 )
	    {
	        sci_status_tx = gsp_sci5_dev->GetStatus();
	        if(-1 == check_bytetimeout(serial_ch_id, 1))
	        {
	            return -2;
	        }
	    }
		while(1)
		{
			if(0 != g_bg96_uart_teiflag[serial_ch_id])
			{
				break;
			}
			if(-1 == check_bytetimeout(serial_ch_id, 1))
			{
				timeout = 1;
				break;
			}
		}
		if(timeout == 1)
		{
			return -2;
		}
	}
    len = 0;
	while(1)
	{
        ercd = bg96_response_get_queue( command, socket_no, &result, &len );
        if (length != NULL)
        {
            *length = len;
        }

		if(0 == ercd )
		{
			break;
		}
		if(-1 == check_bytetimeout(serial_ch_id, 1))
		{
			timeout = 1;
			break;
		}
        if (delay_flag == 1)
        {
		    vTaskDelay( 1 );
        }
	}
	if(timeout == 1)
	{
		return -2;
	}

	ret = -1;
	if(result == expect_code)
	{
		ret = 0;
	}
	return ret;
}


static void timeout_init(uint8_t socket_no, uint16_t timeout_ms)
{
	starttime[socket_no] = xTaskGetTickCount();
	endtime[socket_no] = starttime[socket_no] + timeout_ms;
    remaintime[socket_no] = timeout_ms;
	if((starttime[socket_no] + endtime[socket_no]) < starttime[socket_no])
	{
		/* overflow */
		timeout_overflow_flag[socket_no] = 1;
	}
	else
	{
		timeout_overflow_flag[socket_no] = 0;
	}
}


static int32_t check_timeout_and_remain(uint8_t socket_no, int32_t rcvcount, uint16_t timeout_ms)
{
	if(0 == rcvcount)
	{
		thistime[socket_no] = xTaskGetTickCount();
		if(timeout_overflow_flag[socket_no] == 0)
		{
			if(thistime[socket_no] >= endtime[socket_no] || thistime[socket_no] < starttime[socket_no])
			{
				return -1;
			}
            else
            {
                remaintime[socket_no] = endtime[socket_no] - thistime[socket_no];
            }
            
		}
		else
		{
			if(thistime[socket_no] < starttime[socket_no] && thistime[socket_no] <= endtime[socket_no])
			{
				/* Not timeout  */
				return -1;
			}
            else
            {
                if (thistime[socket_no] < starttime[socket_no])
                {
                    remaintime[socket_no] = endtime[socket_no] - thistime[socket_no];
                }
                else
                {
                    remaintime[socket_no] = timeout_ms - (thistime[socket_no] - starttime[socket_no]);
                }
            }
		}
	}
	/* Not timeout  */
	return 0;
}

static int32_t check_timeout(uint8_t socket_no, int32_t rcvcount)
{
	if(0 == rcvcount)
	{
		thistime[socket_no] = xTaskGetTickCount();
		if(timeout_overflow_flag[socket_no] == 0)
		{
			if(thistime[socket_no] >= endtime[socket_no] || thistime[socket_no] < starttime[socket_no])
			{
				return -1;
			}
		}
		else
		{
			if(thistime[socket_no] < starttime[socket_no] && thistime[socket_no] <= endtime[socket_no])
			{
				/* Not timeout  */
				return -1;
			}
		}
	}
	/* Not timeout  */
	return 0;
}

static void bytetimeout_init(uint8_t socket_no, uint16_t timeout_ms)
{
	startbytetime[socket_no] = xTaskGetTickCount();
	endbytetime[socket_no] = startbytetime[socket_no] + timeout_ms;
	if((startbytetime[socket_no] + endbytetime[socket_no]) < startbytetime[socket_no])
	{
		/* overflow */
		byte_timeout_overflow_flag[socket_no] = 1;
	}
	else
	{
		byte_timeout_overflow_flag[socket_no] = 0;
	}
}

static int32_t check_bytetimeout(uint8_t socket_no, int32_t rcvcount)
{
	if(0 != rcvcount)
	{
		thisbytetime[socket_no] = xTaskGetTickCount();
		if(byte_timeout_overflow_flag[socket_no] == 0)
		{
			if(thisbytetime[socket_no] >= endbytetime[socket_no] || thisbytetime[socket_no] < startbytetime[socket_no])
			{
				return -1;
			}
		}
		else
		{
			if(thisbytetime[socket_no] < startbytetime[socket_no] && thisbytetime[socket_no] <= endbytetime[socket_no])
			{
				/* Not timeout  */
				return -1;
			}
		}
	}
	if (flag_ == false){
		return -1;
	}
	/* Not timeout  */
	return 0;
}

static void timeout_only_wait(uint8_t socket_no, uint16_t timeout_ms)
{
    timeout_init(socket_no, timeout_ms);
    while(1)
    {
        if (-1 == check_timeout(socket_no, 0))
        {
            break;
        }
    }

    return;
}

static int32_t bg96_serial_open(uint32_t baudrate)
{
	uint8_t   my_sci_err;
    if (ARM_DRIVER_OK != gsp_sci5_dev->Initialize(bg96_uart_callback_command_port))
    {
        return -1;
    }
    if (ARM_DRIVER_OK != gsp_sci5_dev->PowerControl(ARM_POWER_FULL))
    {
        return -1;
    }
    if (ARM_DRIVER_OK != gsp_sci5_dev->Control((ARM_USART_MODE_ASYNCHRONOUS |    /* UART (Asynchronous) */
                                                ARM_USART_DATA_BITS_8       |    /* 8 Data bits */
                                                ARM_USART_PARITY_NONE       |    /* No Parity */
                                                ARM_USART_STOP_BITS_1       |    /* 1 Stop bit */
                                                ARM_USART_FLOW_CONTROL_NONE)     /* No Flow Control */
                                               ,baudrate))
    {
        return -1;
    }

    /** enable transmit and receive */
    if (ARM_DRIVER_OK != gsp_sci5_dev->Control(ARM_USART_CONTROL_TX_RX,1))
    {
        return -1;
    }
    return 0;
}

int32_t bg96_serial_close(void)
{

    if (ARM_DRIVER_OK != gsp_sci5_dev->PowerControl(ARM_POWER_OFF))
    {
        while(1)
        {
            ;   /* Intentionally empty braces. */
        }
    }
    gsp_sci5_dev->Uninitialize();
    return 0;
}

int32_t bg96_socket_init(void)
{
	int i;
	for(i = 0;i<CREATEABLE_SOCKETS; i++)
	{
		if(BYTEQ_SUCCESS != R_BYTEQ_Open(g_bg96_socket[i].socket_recv_buff, sizeof(g_bg96_socket[i].socket_recv_buff), &g_bg96_socket[i].socket_byteq_hdl))
		{
			return -1;
		}
	}

    if (g_bg96_semaphore != NULL)
    {
        vSemaphoreDelete(g_bg96_semaphore);
    }
    g_bg96_semaphore = xSemaphoreCreateMutex();

    if( g_bg96_semaphore == NULL )
    {
    	return -1;
    }

    if (g_bg96_tx_semaphore != NULL)
    {
        vSemaphoreDelete(g_bg96_tx_semaphore);
    }
    g_bg96_tx_semaphore = xSemaphoreCreateMutex();

    if( g_bg96_tx_semaphore == NULL )
    {
    	return -1;
    }

    if (g_bg96_rx_semaphore != NULL)
    {
        vSemaphoreDelete(g_bg96_rx_semaphore);
    }
    g_bg96_rx_semaphore = xSemaphoreCreateMutex();

    if( g_bg96_rx_semaphore == NULL )
    {
    	return -1;
    }

	/* Success. */
	return 0;

}

int effl;

static void bg96_uart_callback_command_port(uint32_t event)
{
    /* Check event */
    switch( event )
    {
        case ARM_USART_EVENT_SEND_COMPLETE:
            {
            ;   /* Describe the process when sending is completed */
        	    g_bg96_uart_teiflag[BG96_UART_COMMAND_PORT] = 1;
            }
        break;

        case ARM_USART_EVENT_RECEIVE_COMPLETE:
            {
            ;   /* Describe processing when receiving is completed */
                __NOP();
            }
        break;
        case ARM_USART_EVENT_RX_OVERFLOW:
            {
                __NOP();

                effl = 1;
            }
        break;

        case ARM_USART_EVENT_RX_FRAMING_ERROR:
            {
                __NOP();

                effl = 2;
            }
        break;

        case ARM_USART_EVENT_RX_PARITY_ERROR:
            {
                __NOP();

                effl = 3;
            }
        break;
        default:
            {
            /* Resume reception when a reception error occurs */
            }
        break;
    }
} /* End of function my_sci_callback() */

uint8_t bg96_get_time (bg96_datetime_t *p_time, uint16_t str_size)
{
    int32_t func_ret;
    uint8_t ret = BG96_RETURN_ERROR;
    uint8_t mutex_flag;

    mutex_flag = MUTEX_TX|MUTEX_RX;
    if( 0 == bg96_take_mutex(mutex_flag) )
    {

		if (g_bg96_cgatt_flg == 1)
		{
			memset(buff, 0x00, sizeof(buff));
			strcpy((char *)buff,"AT+QLTS=2\r");
			func_ret = bg96_serial_send_with_recvtask(BG96_UART_COMMAND_PORT, buff, 3, 300, BG96_RETURN_OK, BG96_GET_CIPSNTPTIME, 0xff, &dummy_len, 0);
			if(func_ret != 0)
			{
				/* Give back the socketInUse mutex. */
				bg96_give_mutex(mutex_flag);
				return ret;
			}

			if ((  strlen(g_bg96_time.year)
				 | strlen(g_bg96_time.month)
				 | strlen(g_bg96_time.day)
				 | strlen(g_bg96_time.hour)
				 | strlen(g_bg96_time.min)
				 | strlen(g_bg96_time.sec)
				) == 0)
			{
				/* Give back the socketInUse mutex. */
				bg96_give_mutex(mutex_flag);
				return ret;
			}
			else
			{
				memcpy(p_time->year,  g_bg96_time.year,  str_size+2);
				memcpy(p_time->month, g_bg96_time.month, str_size);
				memcpy(p_time->day,   g_bg96_time.day,   str_size);
				memcpy(p_time->hour,  g_bg96_time.hour,  str_size);
				memcpy(p_time->min,   g_bg96_time.min,   str_size);
				memcpy(p_time->sec,   g_bg96_time.sec,   str_size);
				ret = BG96_RETURN_OK;

			}
		}
    }
	/* Give back the socketInUse mutex. */
	bg96_give_mutex(mutex_flag);
    return ret;
}


// mode 0: Only 3.8v off
//      1: 0 + ATI+QPOWD
void bg96_power_down (uint8_t mode)
{
    if (mode == 1) {
        bg96_serial_send_basic(BG96_UART_COMMAND_PORT, "ATI+QPOWD\r", 6, 4000, BG96_RETURN_OK);
        timeout_init(1, 5000);
        while (1)
        {
            if (-1 == check_timeout(1, 0))
            {
                // timeout
                break;
            }
        }
    }
    bg96_SysCondition_pwrkey_H();    // PE0 L out
    return;
}

uint8_t bg96_is_netaccess (void)
{
    return g_bg96_cgatt_flg;
}

static int32_t bg96_take_mutex(uint8_t mutex_flag)
{
#if 0
	if(0 != (mutex_flag & MUTEX_TX))
	{
		if( xSemaphoreTake( g_bg96_tx_semaphore, xMaxSemaphoreBlockTime ) != pdTRUE )
		{
        	return -1;
		}
	}

	if(0 != (mutex_flag & MUTEX_RX))
	{
		if( xSemaphoreTake( g_bg96_rx_semaphore, xMaxSemaphoreBlockTime ) != pdTRUE )
		{
			if(0 != (mutex_flag & MUTEX_TX))
			{
				xSemaphoreGive( g_bg96_tx_semaphore );
			}
			return -1;
		}
	}
#else
	if( xSemaphoreTake( g_bg96_semaphore, xMaxSemaphoreBlockTime ) != pdTRUE )
	{
		return -1;
	}
#endif
	return 0;
}

static void bg96_give_mutex(uint8_t mutex_flag)
{
#if 0
	if(0 != (mutex_flag & MUTEX_RX))
	{
		xSemaphoreGive( g_bg96_rx_semaphore);
	}
	if(0 != (mutex_flag & MUTEX_TX))
	{
		xSemaphoreGive( g_bg96_tx_semaphore);
	}
#else
	xSemaphoreGive( g_bg96_semaphore);
#endif
	return;
}


wifi_err_t R_CELLULAR_BG96_Close(void)
{
	int i;
	wifi_err_t api_ret = WIFI_SUCCESS;

	if(0 == R_CELLULAR_BG96_IsConnected())
	{
		R_CELLULAR_BG96_Disconnect();
	}
	bg96_serial_close();
	bg96_delete_recv_task();

	for(i=0; i<WIFI_CFG_CREATABLE_SOCKETS; i++)
	{
		R_BYTEQ_Close(g_bg96_socket[i].socket_byteq_hdl);
	}

    bg96_power_down(1);

	g_bg96_system_state = BG96_SYSTEM_CLOSE;

	return api_ret;
}


wifi_err_t R_CELLULAR_BG96_Disconnect (void)
{
	int32_t i;
	int32_t ret;
	uint8_t mutex_flag;
	wifi_err_t api_ret = WIFI_SUCCESS;

	return api_ret;
}


wifi_err_t R_CELLULAR_BG96_SocketClose (int32_t socket_no)
{
	wifi_err_t api_ret = WIFI_SUCCESS;

	if((socket_no >= WIFI_CFG_CREATABLE_SOCKETS) || (socket_no < 0))
	{
		return WIFI_ERR_SOCKET_NUM;
	}
	if( BG96_SYSTEM_CLOSE == g_bg96_system_state)
	{
		return WIFI_ERR_NOT_OPEN;
	}

	if(g_bg96_socket[socket_no].socket_create_flag == 1)
	{
		api_ret = R_CELLULAR_BG96_SocketShutdown (socket_no);

		R_BYTEQ_Flush(g_bg96_socket[socket_no].socket_byteq_hdl);

		g_bg96_socket[socket_no].ipversion = 0;
		g_bg96_socket[socket_no].protocol = 0;
		g_bg96_socket[socket_no].socket_create_flag = 0;
		g_bg96_socket[socket_no].ssl_flag = 0;
		g_bg96_socket[socket_no].ssl_type = 0;
		g_bg96_socket[socket_no].socket_status = WIFI_SOCKET_STATUS_CLOSED;
        g_bg96_socket[socket_no].qiurc_rcv_flg = 0;
	}
	return api_ret;

}

int32_t R_CELLULAR_BG96_IsConnected (void)
{
	int32_t ret = -1;

	if(BG96_SYSTEM_CONNECT == g_bg96_system_state)
	{
		ret = 0;
	}
	return ret;
}

wifi_err_t R_CELLULAR_BG96_SocketShutdown (int32_t socket_no)
{
	wifi_err_t api_ret = WIFI_SUCCESS;
	int32_t subroutain_ret;
	uint8_t mutex_flag;

	if((socket_no >= WIFI_CFG_CREATABLE_SOCKETS) || (socket_no < 0) ||
			(0 == g_bg96_socket[socket_no].socket_create_flag) )
	{
		return WIFI_ERR_SOCKET_NUM;
	}

    if (g_bg96_socket[socket_no].socket_status <= WIFI_SOCKET_STATUS_SOCKET)
    {
        return api_ret;
    }

	if( BG96_SYSTEM_CLOSE == g_bg96_system_state)
	{
		return WIFI_ERR_NOT_OPEN;
	}
	mutex_flag = (MUTEX_TX | MUTEX_RX);
    if(0 == bg96_take_mutex(mutex_flag))
	{
        memset(buff, 0x00, sizeof(buff));
		sprintf((char *)buff,"AT+QICLOSE=%d\r",socket_no);
		subroutain_ret = bg96_serial_send_with_recvtask(BG96_UART_COMMAND_PORT, buff, 3, 15000, BG96_RETURN_OK, BG96_SET_CIPCLOSE, socket_no, &dummy_len, 0);
		if(subroutain_ret == -2)
		{
			subroutain_ret = 0;
		}

		if(subroutain_ret != 0)
		{
				api_ret = WIFI_ERR_MODULE_COM;
		}
		else
		{
			g_bg96_socket[socket_no].socket_status = WIFI_SOCKET_STATUS_SOCKET;
		}
    	/* Give back the socketInUse mutex. */
		bg96_give_mutex(mutex_flag);

    }
    else
    {
    	api_ret = WIFI_ERR_TAKE_MUTEX;
    }
	return api_ret;
}

void reset_button(void)
{
	bg96_SysCondition_pwrkey_L();
	timeout_only_wait(1, 8000);
	bg96_SysCondition_pwrkey_H();
	timeout_init(1, 15000);         // timeout 10ms
}
static int32_t bg96_serial_send_recv_test(uint8_t serial_ch_id, uint8_t *ptextstring, uint16_t response_type, uint16_t timeout_ms, bg96_return_code_t expect_code)
{
	volatile int32_t timeout;
		volatile uint32_t ercd;
		volatile ARM_USART_STATUS sci_status_tx;
		volatile ARM_USART_STATUS sci_status_rx;
		static volatile uint32_t recvcnt = 0;
		volatile int32_t ret = 0;
		memset(recvbuff,0,sizeof(recvbuff));
		timeout_init(serial_ch_id, timeout_ms);
		if(ptextstring != NULL)
		{
			timeout = 0;
			recvcnt = 0;
			g_bg96_uart_teiflag[serial_ch_id] = 0;
			ercd = gsp_sci5_dev->Send(ptextstring, strlen((const char *)ptextstring));
			if(ARM_DRIVER_OK != ercd)
			{
				return -1;
			}

	        sci_status_tx = gsp_sci5_dev->GetStatus();

	        while (sci_status_tx.tx_busy == 1 )
	        {
	            sci_status_tx = gsp_sci5_dev->GetStatus();

	            if(-1 == check_timeout(serial_ch_id, recvcnt))
	            {
	            	return -1;
	            }
	        }
			while(1)
			{
				if(0 != g_bg96_uart_teiflag[serial_ch_id])
				{
					break;
				}
				if(-1 == check_timeout(serial_ch_id, recvcnt))
				{
					timeout = 1;
					break;
				}
			}
			if(timeout == 1)
			{
				return -1;
			}
		}
		while(1)
			{
				ercd = gsp_sci5_dev->Receive(&recvbuff[recvcnt], response_type);
				if(ARM_DRIVER_OK == ercd)
				{
					bytetimeout_init(serial_ch_id, response_type);
					sci_status_rx = gsp_sci5_dev->GetStatus();
					while (sci_status_rx.rx_busy == 1)
					{
						sci_status_rx = gsp_sci5_dev->GetStatus();
						if(-1 == check_timeout(serial_ch_id, 0))
							{
								flag_ = false;
							}
						if(-1 == check_bytetimeout(serial_ch_id, recvcnt))
						{
							ret = -1;
							flag_ = true;
							break;
						}
					}
					recvcnt = (uint32_t )response_type;
				}
				else
				{
					return -1;
				}

				if (recvcnt >= (uint32_t )response_type)
				{
					break;
				}
			}
			if(timeout == 1)
			{
				return -1;
			}

		return ret;
}

static int32_t bg96_serial_response_test( int32_t recvcnt,bg96_return_code_t expect_code)
{
	/* Response data check */
	int32_t ret = -1;
	recvbuff[recvcnt] = '\0';
	if(recvcnt >= strlen((const char *)bg96_result_code[expect_code][0]))
	{
		if(0 == strncmp((const char *)&recvbuff[recvcnt - strlen((const char *)bg96_result_code[expect_code][0]) ],
				(const char *)bg96_result_code[expect_code][0],
				strlen((const char *)bg96_result_code[expect_code][0])))
		{
			ret = 0;
		}
	}

	return ret;

}
