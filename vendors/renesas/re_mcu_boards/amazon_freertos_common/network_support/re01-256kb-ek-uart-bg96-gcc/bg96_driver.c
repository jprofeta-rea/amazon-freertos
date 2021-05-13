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
// removed 2020/10 start
/////#define SCI_RX_BUSIZ_DEFAULT                    (1500*3)
// removed 2020/10 end

// changed 2020/10 start
#define UART_BUS_SPEED          (115200)              /* UART bus speed(bps) */
// changed 2020/10 end

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

// removed 2020/10 start
/////static int32_t bg96_serial_close(void);
// removed 2020/10 end

static int32_t bg96_serial_data_port_open(void);
static int32_t bg96_serial_send_basic(uint8_t serial_ch_id, uint8_t *ptextstring, uint16_t response_type, uint16_t timeout_ms, bg96_return_code_t expect_code);
static int32_t bg96_change_socket_index(uint8_t socket_no);
static TickType_t g_sl_bg96_tcp_recv_timeout = 3000;		/* ## slowly problem ## unit: 1ms */
static int32_t bg96_take_mutex(uint8_t mutex_flag);
static void bg96_give_mutex(uint8_t mutex_flag);

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

// added 2020/10 start
extern ARM_DRIVER_USART Driver_USART9;
static ARM_DRIVER_USART *gsp_sci9_dev = &Driver_USART9;
// added 2020/10 end

// added 2020/10 start
wifi_err_t R_CELLULAR_BG96_SocketShutdown (int32_t socket_no);
// added 2020/10 end

// added 2020/10 start
uint32_t dummy_len = 0;
// added 2020/10 end

int32_t bg96_wifi_init(void)
{
	int32_t ret;

	uint32_t cgatt_cnt = 0;
    uint32_t recvlen = 0;
	uint8_t atcmd[128];

	if( BG96_SYSTEM_CLOSE != g_bg96_system_state)
	{
		return WIFI_ERR_ALREADY_OPEN;
	}

    g_bg96_cgatt_flg = 0;

    // Module hardware reset
// removed 2020/10 start
/*
    // 3.8V ON
    cbt_SysCondition_v38en(1);
*/
// removed 2020/10 end

// changed 2020/10 start
/////    cbt_SysCondition_pwrkey_H();
    bg96_SysCondition_pwrkey_H();
// changed 2020/10 end

    // 200ms wait
// removed 2020/10 start
    //　QUECTEL 評価ボードで実施する場合の仮対応
/////    timeout_only_wait(1, 200);
// removed 2020/10 end

// changed 2020/10 start
// CBT 用の端子設定    QUECTEL 評価ボードで接続するなら修正が必要
// 現行は、接続なし
/*
    // RTS H P27 = H
    PORT2.PODR.BIT.B7 = 1;
    // DTR H PA2 = H
    PORTA.PODR.BIT.B7 = 1;
*/
// changed 2020/10 end

	ret = bg96_serial_open(115200);
	if(ret != 0)
	{
		return ret;
	}

    // PWRKEY  reset
// changed 2020/10 start
/////    cbt_SysCondition_pwrkey_L();
    bg96_SysCondition_pwrkey_L();
// changed 2020/10 end

    // 1s wait
// removed 2020/10 start
    //　QUECTEL 評価ボードで実施する場合の仮対応
/////    timeout_only_wait(1, 1000);
// removed 2020/10 end

    // PWRKEY  ON
// changed 2020/10 start
/////    cbt_SysCondition_pwrkey_H();
    bg96_SysCondition_pwrkey_H();
// changed 2020/10 end

    // wait STATUS(PD5) == L
// changed 2020/10 start
    //　QUECTEL 評価ボードで実施する場合の仮対応
/////    timeout_init(1, 15000);         // timeout 15s
    timeout_init(1, 10);         // timeout 10ms
// changed 2020/10 end
    while (1)
    {
// changed 2020/10 start
// CBT 用の端子設定    QUECTEL 評価ボードでも接続する場合、修正が必要
// 現行は、実装なし
/*
        if (PORTD.PIDR.BIT.B5 == 0)
        {
            ret = 0;
            break;
        }
        else if (-1 == check_timeout(1, 0))
        {
            // timeout
            ret = -1;
            break;
        }
*/
// changed 2020/10 start
    	//　QUECTEL 評価ボードで実施する場合の仮対応
    	// 固定で10ms 待つ
        if (-1 == check_timeout(1, 0))
        {
            // timeout
//            ret = -1;
            ret = 0;
            break;
        }
// changed 2020/10 end
// changed 2020/10 end
    }

    if(ret != 0)
    {
        // BG96 3.8V OFF
        bg96_power_down(0);
        return ret;
    }

    // 起動後、ATコマンド発行まで 200ms 空ける
// removed 2020/10 start
    //　QUECTEL 評価ボードで実施する場合の仮対応
/////    timeout_only_wait(1, 200);
// removed 2020/10 end

    // echo off
// changed 2020/10 start
/////    ret = bg96_serial_send_basic(BG96_UART_COMMAND_PORT, "ATE0\r", 5, 400, BG96_RETURN_OK);
    ret = bg96_serial_send_basic(BG96_UART_COMMAND_PORT, "ATE0\r", 6, 400, BG96_RETURN_OK);
// changed 2020/10 end

    // APN set
    memset(buff, 0x00, sizeof(buff));
	sprintf(buff, "AT+QICSGP=1,1,\"%s\",\"%s\",\"%s\",1\r", CELLULAR_APN, CELLULAR_APN_USERID, CELLULAR_APN_PASSWORD);
// changed 2020/10 start
/////	ret = bg96_serial_send_basic(BG96_UART_COMMAND_PORT, buff, 20, 4000, BG96_RETURN_OK);
	ret = bg96_serial_send_basic(BG96_UART_COMMAND_PORT, buff, 6, 4000, BG96_RETURN_OK);
// changed 2020/10 end

    cgatt_cnt = 0;
    while(1)
    {
        // 基地局アタッチ確認
// changed 2020/10 start
/////        ret = bg96_serial_send_basic(BG96_UART_COMMAND_PORT, "AT+CGATT?\r", 20, 400, BG96_RETURN_OK);
    	ret = bg96_serial_send_basic(BG96_UART_COMMAND_PORT, "AT+CGATT?\r", 19, 400, BG96_RETURN_OK);
// changed 2020/10 end
        if (ret == 0)
        {
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

        timeout_only_wait(1, 500);      // 500ms 待って 再チェック
    }
    if(ret != 0)
    {
        // BG96 power off & 3.8V OFF
        bg96_power_down(1);

        return ret;
    }

    bg96_socket_init();

// changed 2020/10 start
/////    ret = bg96_serial_send_basic(BG96_UART_COMMAND_PORT, "AT+QIACT=1\r", 20, 4000, BG96_RETURN_OK);
    ret = bg96_serial_send_basic(BG96_UART_COMMAND_PORT, "AT+QIACT=1\r", 6, 4000, BG96_RETURN_OK);
// changed 2020/10 end

    vStart_bg96_recv_task();

    g_bg96_cgatt_flg = 1;

    g_bg96_system_state = BG96_SYSTEM_CONNECT;

    // error handling T.B.D.

    return ret;
}

int32_t    R_CELLULAR_BG96_SocketCreate(uint32_t type, uint32_t ipversion)
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

// changed 2020/10 start
/////		ret = bg96_serial_send_with_recvtask(BG96_UART_COMMAND_PORT, buff, 3, 4000, BG96_RETURN_OK, BG96_SET_CIPSTART, socket_no, 0, 0);
		ret = bg96_serial_send_with_recvtask(BG96_UART_COMMAND_PORT, buff, 3, 4000, BG96_RETURN_OK, BG96_SET_CIPSTART, socket_no, &dummy_len, 0);
// changed 2020/10 end

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
// changed 2020/10 start
/////	sci_err_t ercd;
	uint8_t ercd;
// changed 2020/10 end
	uint8_t result;
    uint32_t value;
    uint8_t mutex_flag;
    uint32_t lenghttmp1;

// added 2020/10 start
    ARM_USART_STATUS sci_status_tx;
// added 2020/10 end

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

            // AT+QISEND= を送信し、 > の返信があるまでを確認
// changed 2020/10 start
/////			ret = bg96_serial_send_with_recvtask(BG96_UART_COMMAND_PORT, buff, 3, remaintime[socket_no], BG96_RETURN_OK_GO_SEND, BG96_SET_CIPSEND, socket_no, 0, 0);
			ret = bg96_serial_send_with_recvtask(BG96_UART_COMMAND_PORT, buff, 3, remaintime[socket_no], BG96_RETURN_OK_GO_SEND, BG96_SET_CIPSEND, socket_no, &dummy_len, 0);
// changed 2020/10 end

			if(ret != 0)
			{
				/* Give back the socketInUse mutex. */
				bg96_give_mutex(mutex_flag);
// changed 2020/10 start
                // RX BSP FIT module
/////				R_BSP_NOP();
				__NOP();
// changed 2020/10 end
				return -1;
			}

			timeout_init(socket_no, 3000);
			timeout = 0;

            bg96_response_set_queue( BG96_SET_CIPSEND_END, socket_no );
			g_bg96_uart_teiflag[BG96_UART_COMMAND_PORT] = 0;

            // > が来たら、実際の転送データを送信する

// changed 2020/10 start
// RX SCI FIT module
/////			ercd = R_SCI_Send(bg96_uart_sci_handle[BG96_UART_COMMAND_PORT], pdata+sended_length, lenghttmp1);
			ercd = gsp_sci9_dev->Send(pdata+sended_length, lenghttmp1);
// changed 2020/10 end

// changed 2020/10 start
/////			if(SCI_SUCCESS != ercd)
			if(ARM_DRIVER_OK != ercd)
// changed 2020/10 end
			{
				/* Give back the socketInUse mutex. */
				bg96_give_mutex(mutex_flag);
				return -2;
			}

// added 2020/10 start
			sci_status_tx = gsp_sci9_dev->GetStatus();

			while (sci_status_tx.tx_busy == 1 )
			{
			    sci_status_tx = gsp_sci9_dev->GetStatus();

// added 2020/10 start
				if(-1 == check_timeout(socket_no, 0))
				{
					/* Give back the socketInUse mutex. */
					bg96_give_mutex(mutex_flag);
					return 0;
				}
// added 2020/10 end
			}
// added 2020/10 end

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

			timeout_init(socket_no, 4000); //[RE01-test] change 2000 to 4000

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
// changed 2020/10 start
                // RX BSP FIT module
/////				R_BSP_NOP();
				__NOP();
// changed 2020/10 end
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
// changed 2020/10 start
/////    sci_err_t ercd;
    uint8_t ercd;
// changed 2020/10 end

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
// added 2020/10 start
    	// BYTEQ に格納済みデータから先に取得する・
    	while (1)
        {
            byteq_ret = R_BYTEQ_Get(g_bg96_socket[socket_no].socket_byteq_hdl, &bytedata);

            if(BYTEQ_SUCCESS == byteq_ret)
            {
                *(pdata + allrecv_len) = bytedata;
                allrecv_len++;

                if (allrecv_len >= length)   // BYTEQ だけで length 以上取得した
                {
                    break;
                }
            }
            else    // BYTEQ は empty
            {
                break;
            }

        }

        if (allrecv_len >= length)   // BYTEQ だけで length 以上取得した
        {
            break;
        }
// added 2020/10 end

        // BYTEQ にデータ蓄積なし、QIRD で取得
        // wait +QIURC: "recv",
        timeout_init(socket_no, timeout_ms);
        timeout = 0;

        // +QIURC: "recv",0 が来ているかどうかを確認する
        // 受信毎に+QIURC: "recv"が来るわけでなく、BG96のバッファが空になるまでは有効であるような動きをしている (BG96が)
        // +QIRD: 0 が来たら qiurc_rcv_flg を 0 に落とすようにする (bg96_driver_2 の方で)
        // また、一連の受信途中でBG96のバッファが空になることもあるので、毎回 qiurc_rcv_flg を確認する
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
			// TimeOutすることはあるので、エラーとしては扱わない
        	// BYTEQから読んだ分は返信する
            return allrecv_len;
        }

        mutex_flag = MUTEX_RX;
        if( 0 == bg96_take_mutex(mutex_flag) )
        {
            memset(pdata_l, 0x00, sizeof(pdata_l));
            tmpcnt++;

            recvlen = length - allrecv_len;

            // QIRD の仕様で Max1500
            if(recvlen > 1500)
            {
                recvlen = 1500;
            }

            memset(buff, 0x00, sizeof(buff));
            sprintf((char *)buff,"AT+QIRD=%d,%d\r",socket_no, recvlen);
            len = 0;
            // AT+QIRD= の受信要求をし、+QIRD: の返信を確認する
            ret = bg96_serial_send_with_recvtask(BG96_UART_COMMAND_PORT, buff, 3, remaintime[socket_no], BG96_RETURN_OK, BG96_SET_QIRD, socket_no, &len, 0);

            if(ret != 0)
            {
                /* Give back the socketInUse mutex. */
                bg96_give_mutex(mutex_flag);

// changed 2020/10 start
                // RX BSP FIT module
/////                R_BSP_NOP();
                __NOP();
// changed 2020/10 end

                if (ret == -2)
                {
                    ret = 0;
                }
                return ret;
            }

// added 2020/10 start
            if (len == 0)
            {
            	// len が 0 なら qiurc_rcv_flg 待ちから
            }
            else
            {
// added 2020/10 end

                timeout_init(socket_no, 4000); //[RE01-test] change 3000 to 4000
                timeout = 0;

                stored_len = 0;
                while(1)
                {
                    // BYTEQ から pdata にデータを格納する
                    byteq_ret = R_BYTEQ_Get(g_bg96_socket[socket_no].socket_byteq_hdl, &bytedata);

                    if(BYTEQ_SUCCESS == byteq_ret)
                    {
                        *(pdata + stored_len + allrecv_len) = bytedata;
                        stored_len++;

                        if (stored_len >= len)
                        {
                        	// len分取得したら次のQIRDへ
                            break;
                        }
                        else if (stored_len >= recvlen)
                        {
                        	// recvlen分取得したら終了 または length に届かない場合は継続
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
// changed 2020/10 start
/////                allrecv_len += len;
                allrecv_len += stored_len;
// changed 2020/10 end
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
// changed 2020/10 start
/////			ret = bg96_serial_send_with_recvtask(BG96_UART_COMMAND_PORT, buff, 3, 10000, BG96_RETURN_OK, BG96_SET_CIPCLOSE, socket_no, 0, 0);
			ret = bg96_serial_send_with_recvtask(BG96_UART_COMMAND_PORT, buff, 3, 10000, BG96_RETURN_OK, BG96_SET_CIPCLOSE, socket_no, &dummy_len, 0);
// changed 2020/10 end
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

// changed 2020/10 start
/////        func_ret = bg96_serial_send_with_recvtask(BG96_UART_COMMAND_PORT, buff, 3, 20000, BG96_RETURN_OK, BG96_SET_CIPDOMAIN, 0xff, 0, 0);
        func_ret = bg96_serial_send_with_recvtask(BG96_UART_COMMAND_PORT, buff, 3, 20000, BG96_RETURN_OK, BG96_SET_CIPDOMAIN, 0xff, &dummy_len, 0);
// changed 2020/10 end
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

// changed 2020/10 start
/////	sci_err_t ercd;
	volatile uint32_t ercd;
// changed 2020/10 end

// added 2020/10 start
	volatile ARM_USART_STATUS sci_status_tx;
	volatile ARM_USART_STATUS sci_status_rx;
// added 2020/10 end

// changed 2020/10 start
/*
	uint32_t recvcnt = 0;
	int32_t ret;
*/
	volatile uint32_t recvcnt = 0;
	volatile int32_t ret;
// changed 2020/10 end


	memset(recvbuff,0,sizeof(recvbuff));

	timeout_init(serial_ch_id, timeout_ms);

	if(ptextstring != NULL)
	{
		timeout = 0;
		recvcnt = 0;
		g_bg96_uart_teiflag[serial_ch_id] = 0;

// changed 2020/10 start
/////		ercd = R_SCI_Send(bg96_uart_sci_handle[serial_ch_id], ptextstring, strlen((const char *)ptextstring));
		ercd = gsp_sci9_dev->Send(ptextstring, strlen((const char *)ptextstring));
// changed 2020/10 end

// changed 2020/10 start
/////		if(SCI_SUCCESS != ercd)
		if(ARM_DRIVER_OK != ercd)
// changed  2020/10 end

		{
			return -1;
		}

// added 2020/10 start
        sci_status_tx = gsp_sci9_dev->GetStatus();

        while (sci_status_tx.tx_busy == 1 )
        {
            sci_status_tx = gsp_sci9_dev->GetStatus();

// added 2020/10 start
            if(-1 == check_timeout(serial_ch_id, recvcnt))
            {
            	return -1;
            }
// added 2020/10 end
        }
// added 2020/10 end

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
// changed 2020/10 start
/////		ercd = R_SCI_Receive(bg96_uart_sci_handle[serial_ch_id], &recvbuff[recvcnt], 1);
		ercd = gsp_sci9_dev->Receive(&recvbuff[recvcnt], response_type);
// changed 2020/10 end

// changed 2020/10 start
/////		if(SCI_SUCCESS == ercd)
		if(ARM_DRIVER_OK == ercd)
// changed 2020/10 end
		{
// added 2020/10 start
			bytetimeout_init(serial_ch_id, response_type);
// added 2020/10 end

// added 2020/10 start
			sci_status_rx = gsp_sci9_dev->GetStatus();

			while (sci_status_rx.rx_busy == 1 )
			{
			    sci_status_rx = gsp_sci9_dev->GetStatus();

// added 2020/10 start
			    if(-1 == check_bytetimeout(serial_ch_id, recvcnt))
			    {
			    	return -1;
			    }
// added 2020/10 end
		    }
// added 2020/10 end

// changed 2020/10 start
/////			recvcnt++;
			recvcnt = (uint32_t )response_type;
// changed 2020/10 end

// removed 2020/10 start
/*
			bytetimeout_init(serial_ch_id, response_type);
			if(recvcnt < 4)
			{
				continue;
			}
			if(recvcnt == sizeof(recvbuff)-2)
			{
				break;
			}
*/
// removed 2020/10 end

// changed 2020/10 end
		}

// added 2020/10 start
		else
		{
			return -1;
		}

		if (recvcnt >= (uint32_t )response_type)
		{
			break;
		}
// added 2020/10 end

// removed 2020/10 start
/*
		if(-1 == check_bytetimeout(serial_ch_id, recvcnt))
		{
			break;
		}
		if(-1 == check_timeout(serial_ch_id, recvcnt))
		{
			timeout = 1;
			break;
		}
*/
// removed 2020/10 end

	}
	if(timeout == 1)
	{
		return -1;
	}

	/* Response data check */
	ret = -1;
	recvbuff[recvcnt] = '\0';
	if(recvcnt >= strlen((const char *)bg96_result_code[expect_code][g_bg96_return_mode]))
	{
		if(0 == strncmp((const char *)&recvbuff[recvcnt - strlen((const char *)bg96_result_code[expect_code][g_bg96_return_mode]) ],
				(const char *)bg96_result_code[expect_code][g_bg96_return_mode],
				strlen((const char *)bg96_result_code[expect_code][g_bg96_return_mode])))
		{
			ret = 0;
		}
	}
	return ret;
}

int32_t bg96_serial_send_with_recvtask(uint8_t serial_ch_id, uint8_t *ptextstring, uint16_t response_type, uint16_t timeout_ms, bg96_return_code_t expect_code,  uint8_t command, uint8_t socket_no, uint32_t *length, uint8_t delay_flag)
{
	volatile int32_t timeout;

// changed 2020/10 start
/////	sci_err_t ercd;
	uint8_t ercd;
// changed 2020/10 end

// added 2020/10 start
	ARM_USART_STATUS sci_status_tx;
// added 2020/10 end

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

// changed 2020/10 start
/////		ercd = R_SCI_Send(bg96_uart_sci_handle[serial_ch_id], ptextstring, strlen((const char *)ptextstring));
		ercd = gsp_sci9_dev->Send(ptextstring, strlen((const char *)ptextstring));
// changed 2020/10 end

// changed 2020/10 start
/////		if(SCI_SUCCESS != ercd)
	    if(ARM_DRIVER_OK != ercd)
// changed 2020/10 end

		{
			return -1;
		}

// changed 2020/10 start
	    sci_status_tx = gsp_sci9_dev->GetStatus();

	    while (sci_status_tx.tx_busy == 1 )
	    {
	        sci_status_tx = gsp_sci9_dev->GetStatus();

// added 2020/10 start
	        if(-1 == check_bytetimeout(serial_ch_id, 1))
	        {
	            return -2;
	        }
// added 2020/10 end
	    }
// changed 2020/10 end

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

// changed 2020/10 start
/////        if (length != 0)
        if (length != NULL)
// changed 2020/10 end
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
// changed 2020/10 start
/////	sci_err_t   my_sci_err;
	uint8_t   my_sci_err;
// changed 2020/10 end

// changed 2020/10 start
/*
	R_SCI_PinSet_bg96_serial_default();

	g_bg96_sci_config[BG96_UART_COMMAND_PORT].async.baud_rate    = baudrate;
	g_bg96_sci_config[BG96_UART_COMMAND_PORT].async.clk_src      = SCI_CLK_INT;
	g_bg96_sci_config[BG96_UART_COMMAND_PORT].async.data_size    = SCI_DATA_8BIT;
	g_bg96_sci_config[BG96_UART_COMMAND_PORT].async.parity_en    = SCI_PARITY_OFF;
	g_bg96_sci_config[BG96_UART_COMMAND_PORT].async.parity_type  = SCI_EVEN_PARITY;
	g_bg96_sci_config[BG96_UART_COMMAND_PORT].async.stop_bits    = SCI_STOPBITS_1;
	g_bg96_sci_config[BG96_UART_COMMAND_PORT].async.int_priority = 15;    // 1=lowest, 15=highest

    my_sci_err = R_SCI_Open(SCI_CH_bg96_serial_default, SCI_MODE_ASYNC, &g_bg96_sci_config[BG96_UART_COMMAND_PORT], bg96_uart_callback_command_port, &bg96_uart_sci_handle[BG96_UART_COMMAND_PORT]);
*/

//    if (ARM_DRIVER_OK != gsp_sci9_dev->Initialize(usart_callback))
    if (ARM_DRIVER_OK != gsp_sci9_dev->Initialize(bg96_uart_callback_command_port))
    {
        return -1;
    }
    if (ARM_DRIVER_OK != gsp_sci9_dev->PowerControl(ARM_POWER_FULL))
    {
        return -1;
    }
    if (ARM_DRIVER_OK != gsp_sci9_dev->Control((ARM_USART_MODE_ASYNCHRONOUS |    /* UART (Asynchronous) */
                                                ARM_USART_DATA_BITS_8       |    /* 8 Data bits */
                                                ARM_USART_PARITY_NONE       |    /* No Parity */
                                                ARM_USART_STOP_BITS_1       |    /* 1 Stop bit */
                                                ARM_USART_FLOW_CONTROL_NONE)     /* No Flow Control */
                                               ,UART_BUS_SPEED))
    {
        return -1;
    }

    /** enable transmit and receive */
    if (ARM_DRIVER_OK != gsp_sci9_dev->Control(ARM_USART_CONTROL_TX_RX,1))
    {
        return -1;
    }
// changed 2020/10 end

// changed 2020/10 start
/*
    if(SCI_SUCCESS != my_sci_err)
    {
    	return -1;
    }
*/
// changed 2020/10 end

    return 0;
}

int32_t bg96_serial_close(void)
{
// changed 2020/10 start
// RX SCI FIT module
/////    R_SCI_Close(bg96_uart_sci_handle[BG96_UART_COMMAND_PORT]);

// added 2020/10 start
//	uint32_t   sci_err = ARM_DRIVER_ERROR;
// added 2020/10 end

// changed 2020/10 start
    if (ARM_DRIVER_OK != gsp_sci9_dev->PowerControl(ARM_POWER_OFF))
//	sci_err = gsp_sci9_dev->PowerControl(ARM_POWER_OFF);
//    if (ARM_DRIVER_OK != sci_err)
// changed 2020/10 end
    {
        while(1)
        {
            ;   /* Intentionally empty braces. */
        }
    }
// changed 2020/10 end

// changed 2020/10 start
    return 0;
//    return sci_err;
// changed 2020/10 end

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

// changed 2020/10 start
/////static void bg96_uart_callback_command_port(void *pArgs)
static void bg96_uart_callback_command_port(uint32_t event)
// changed 2020/10 end
{
// changed 2020/10 start
// RX SCI FIT module
/////    sci_cb_args_t   *p_args;

/////    p_args = (sci_cb_args_t *)pArgs;

/////    if (SCI_EVT_RX_CHAR == p_args->event)
/////    {
/////        /* From RXI interrupt; received character data is in p_args->byte */
/////    	R_BSP_NOP();
/////    }
/////#if SCI_CFG_TEI_INCLUDED
/////	else if (SCI_EVT_TEI == p_args->event)
/////	{
/////		g_bg96_uart_teiflag[BG96_UART_COMMAND_PORT] = 1;

/////		R_BSP_NOP();
/////	}
/////#endif
/////    else if (SCI_EVT_RXBUF_OVFL == p_args->event)
/////    {
/////        /* From RXI interrupt; rx queue is full; 'lost' data is in p_args->byte
/////           You will need to increase buffer size or reduce baud rate */
/////    	R_BSP_NOP();

/////        effl = 1;
/////    }
/////    else if (SCI_EVT_OVFL_ERR == p_args->event)
/////    {
/////        /* From receiver overflow error interrupt; error data is in p_args->byte
/////           Error condition is cleared in calling interrupt routine */
/////    	R_BSP_NOP();

/////        effl = 2;
/////    }
/////    else if (SCI_EVT_FRAMING_ERR == p_args->event)
/////    {
/////        /* From receiver framing error interrupt; error data is in p_args->byte
/////           Error condition is cleared in calling interrupt routine */
/////    	R_BSP_NOP();

/////        effl = 3;
/////    }
/////    else if (SCI_EVT_PARITY_ERR == p_args->event)
/////    {
/////        /* From receiver parity error interrupt; error data is in p_args->byte
/////           Error condition is cleared in calling interrupt routine */
/////    	R_BSP_NOP();

/////        effl = 4;
/////    }
/////    else
/////    {
/////        /* Do nothing */
/////    }


    /** Check event */
    switch( event )
    {
        case ARM_USART_EVENT_SEND_COMPLETE:
            {
            ;   /* Describe the process when sending is completed */
// added 2020/10 start
        	    g_bg96_uart_teiflag[BG96_UART_COMMAND_PORT] = 1;
//        	    __NOP();
// added 2020/10 end
            }
        break;

        case ARM_USART_EVENT_RECEIVE_COMPLETE:
            {
            ;   /* Describe processing when receiving is completed */
// added 2020/10 start
                __NOP();
// added 2020/10 end
            }
        break;

// added 2020/10 start
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
// added 2020/10 end

        default:
            {
            /* Resume reception when a reception error occurs */
//            gs_retry_flg = true;
            }
        break;
    }

// changed 2020/10 end

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
			// 現在時刻取得
			memset(buff, 0x00, sizeof(buff));
			strcpy((char *)buff,"AT+QLTS=2\r");
// changed 2020/10 start
/////			func_ret = bg96_serial_send_with_recvtask(BG96_UART_COMMAND_PORT, buff, 3, 300, BG96_RETURN_OK, BG96_GET_CIPSNTPTIME, 0xff, 0, 0);
			func_ret = bg96_serial_send_with_recvtask(BG96_UART_COMMAND_PORT, buff, 3, 300, BG96_RETURN_OK, BG96_GET_CIPSNTPTIME, 0xff, &dummy_len, 0);
// changed 2020/10 end
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
        // power off
// changed 2020/10 start
/////        bg96_serial_send_basic(BG96_UART_COMMAND_PORT, "ATI+QPOWD\r", 20, 4000, BG96_RETURN_OK);
        bg96_serial_send_basic(BG96_UART_COMMAND_PORT, "ATI+QPOWD\r", 6, 4000, BG96_RETURN_OK);
// changed 2020/10 end

        // wait STATUS(PD5) == H
// changed 2020/10 start
        // QUECTEL 評価ボードで実施する場合の仮対応
/////        timeout_init(1, 10000);         // timeout 10s
        timeout_init(1, 10);         // timeout 10ms
// changed 2020/10 end
        while (1)
        {
// changed 2020/10 start
// CBT 用の端子設定    QUECTEL 評価ボードで接続する場合は、修正が必要
// 現行は、固定時間を待つ実装
/*
            if (PORTD.PIDR.BIT.B5 == 1)
            {
                break;
            }
            else if (-1 == check_timeout(1, 0))
            {
                // timeout
                break;
            }
*/
// changed 2020/10 start
        	// QUECTEL 評価ボードで実施する場合の仮対応
        	// 10ms 固定で待つ
            if (-1 == check_timeout(1, 0))
            {
                // timeout
                break;
            }
// changed 2020/10 end
// changed 2020/10 end
        }
    }

// removed 2020/10 start
/*
    // BG96 3.8V OFF
    cbt_SysCondition_v38en(0);
*/
// removed 2020/10 end

// changed 2020/10 start
/////    cbt_SysCondition_pwrkey_H();    // PE0 L out
    bg96_SysCondition_pwrkey_H();    // PE0 L out
// changed 2020/10 end

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

// changed 2020/10 start
/////		subroutain_ret = bg96_serial_send_with_recvtask(BG96_UART_COMMAND_PORT, buff, 3, 15000, BG96_RETURN_OK, BG96_SET_CIPCLOSE, socket_no, 0, 0);
		subroutain_ret = bg96_serial_send_with_recvtask(BG96_UART_COMMAND_PORT, buff, 3, 15000, BG96_RETURN_OK, BG96_SET_CIPCLOSE, socket_no, &dummy_len, 0);
// changed 2020/10 end

		// BG96の仕様で QICLOSE はタイムアウトしてもソケット切断されるのでエラーとして扱わない。
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

