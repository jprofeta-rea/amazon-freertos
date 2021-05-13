#ifndef BG96_DRIVER_H
#define BG96_DRIVER_H

#include "r_byteq_if.h"

#define DEBUGLOG  0
#define BG96_PORT_DEBUG 0

/* Configuration */
#define BG96_USE_UART_NUM (1)

#define BG96_RETURN_TEXT_OK          "OK\r\n"
#define BG96_RETURN_TEXT_ERROR       "ERROR\r\n"
#define BG96_RETURN_TEXT_READY       "\r\nready\r\n"
#define BG96_RETURN_TEXT_OK_GO_SEND  "\r\n> "
#define BG96_RETURN_TEXT_SEND_BYTE   " bytes\r\n"
#define BG96_RETURN_TEXT_SEND_OK     "SEND OK\r\n"
#define BG96_RETURN_TEXT_SEND_FAIL   "SEND FAIL\r\n"

#define BG96_SOCKET_STATUS_TEXT_CLOSED          "CLOSED"
#define BG96_SOCKET_STATUS_TEXT_SOCKET          "SOCKET"
#define BG96_SOCKET_STATUS_TEXT_BOUND           "BOUND"
#define BG96_SOCKET_STATUS_TEXT_LISTEN          "LISTEN"
#define BG96_SOCKET_STATUS_TEXT_CONNECTED       "CONNECTED"

#define BG96_UART_COMMAND_PORT 0

// changed 2020/10 start
/////#define CREATEABLE_SOCKETS (4)
//#define CREATEABLE_SOCKETS (2)
#define CREATEABLE_SOCKETS (5)	//[RE01-test]
// changed 2020/10 end

#define BG96_RETRY_GATT          (500)

typedef enum
{
	BG96_RETURN_OK            = 0,
	BG96_RETURN_ERROR,
	BG96_RETURN_READY,
	BG96_RETURN_OK_GO_SEND,
	BG96_RETURN_SEND_BYTE,
	BG96_RETURN_SEND_OK,
	BG96_RETURN_SEND_FAIL,
	BG96_RETURN_ENUM_MAX,
}bg96_return_code_t;

typedef enum
{
	BG96_RETURN_STRING_TEXT            = 0,
	BG96_RETURN_STRING_MAX,
}bg96_return_string_t;

typedef enum
{
	BG96_SECURITY_OPEN            = 0,
	BG96_SECURITY_WPA,
	BG96_SECURITY_WPA2,
	BG96_SECURITY_WEP,
	BG96_SECURITY_UNDEFINED,
	BG96_SECURITY_MAX,
}bg96_security_t;

typedef enum
{
    BG96_SYSTEM_CLOSE=0,
    BG96_SYSTEM_CONNECT,
} bg96_system_status_t;

typedef enum
{
	BG96_SOCKET_STATUS_CLOSED            = 0,
	BG96_SOCKET_STATUS_SOCKET,
	BG96_SOCKET_STATUS_BOUND,
	BG96_SOCKET_STATUS_LISTEN,
	BG96_SOCKET_STATUS_CONNECTED,
	BG96_SOCKET_STATUS_MAX,
}bg96_socket_status_t;

typedef enum
{
	BG96_SET_CWMODE_CUR = 0,
	BG96_SET_CIPMUX,
	BG96_SET_CWJAP_CUR,
	BG96_SET_CWLIF,
	BG96_GET_CIPAPMAC,
	BG96_GET_CIPSTA,
	BG96_GET_CIPSNTPTIME,
	BG96_SET_CIPDOMAIN,
	BG96_SET_CIPSTART,
	BG96_SET_CIPSEND,
	BG96_SET_CIPSEND_END,
	BG96_SET_CIPCLOSE,
	BG96_SET_QIRD,
	BG96_SET_CIMI,
	BG96_SET_CSQ,
	BG96_LIST_MAX
}bg96_command_list;

typedef struct bg96_socket_tag
{
	uint32_t receive_num;
	uint32_t receive_count;
	uint32_t put_error_count;
// changed 2020/10 start (メモリ削減)
/////	uint8_t socket_recv_buff[8192];
uint8_t socket_recv_buff[2048];
// changed 2020/10 end (メモリ削減)

	byteq_hdl_t socket_byteq_hdl;
	uint8_t socket_create_flag;

	uint8_t socket_status;
	uint8_t ipversion;
	uint8_t protocol;

    TickType_t send_starttime;
    TickType_t send_thistime;
    TickType_t send_endtime;
    TickType_t recv_starttime;
    TickType_t recv_thistime;
    TickType_t recv_endtime;
    uint8_t send_timeout_overflow_flag;
    uint8_t recv_timeout_overflow_flag;
    uint8_t ssl_flag;
    uint8_t ssl_type;
    uint8_t ssl_cert_key_id;
    uint8_t ssl_ca_id;

    uint8_t qiurc_rcv_flg;
}bg96_socket_t;



typedef enum 			// Wi-Fi APIエラーコード
{
	WIFI_SUCCESS            = 0,	// 成功
	WIFI_ERR_PARAMETER	    = -1,	// 引数が無効です。
	WIFI_ERR_ALREADY_OPEN   = -2,	// すでに初期化済みです
	WIFI_ERR_NOT_OPEN       = -3,	// 初期化していません
	WIFI_ERR_SERIAL_OPEN    = -4,	// シリアルの初期化ができません。
	WIFI_ERR_MODULE_COM     = -5,	// WiFiモジュールとの通信に失敗しました。
	WIFI_ERR_NOT_CONNECT    = -6,	// アクセスポイントに接続していません。
	WIFI_ERR_SOCKET_NUM     = -7,	// 利用可能なソケットがありません。
	WIFI_ERR_SOCKET_CREATE  = -8,	// ソケットを作成できません。
	WIFI_ERR_CHANGE_SOCKET  = -9,	// ソケットを切り替えられません。
	WIFI_ERR_SOCKET_CONNECT = -10,	// ソケットに接続できません。
	WIFI_ERR_BYTEQ_OPEN     = -11,	// BYTEQの割り当てに失敗しました。
	WIFI_ERR_SOCKET_TIMEOUT = -12,	// ソケットの送信でタイムアウトしました。
	WIFI_ERR_TAKE_MUTEX     = -13,	// Mutexの取得に失敗しました。
} wifi_err_t;

typedef struct
{
    uint8_t year[5];
    uint8_t month[3];
    uint8_t day[3];
    uint8_t hour[3];
    uint8_t min[3];
    uint8_t sec[3];
}
bg96_datetime_t;


int32_t bg96_wifi_init(void);
int32_t bg96_socket_init(void);
int32_t bg96_tcp_connect(uint8_t socket_no, uint32_t ipaddr, uint16_t port);
int32_t bg96_tcp_send(uint8_t socket_no, uint8_t *pdata, int32_t length, uint32_t timeout);
int32_t bg96_tcp_recv(uint8_t socket_no, uint8_t *pdata, int32_t length, uint32_t timeout);
int32_t bg96_tcp_disconnect(uint8_t socket_no);
int32_t bg96_dns_query(uint8_t *ptextstring, uint32_t *ulipaddr);
int32_t bg96_serial_tcp_recv_timeout_set(uint8_t socket_no, TickType_t timeout_ms);

int32_t bg96_serial_close(void);

uint8_t bg96_get_time (bg96_datetime_t *p_time, uint16_t str_size);
void bg96_power_down (uint8_t mode);
uint8_t bg96_is_netaccess (void);

wifi_err_t R_CELLULAR_BG96_Disconnect (void);
wifi_err_t R_CELLULAR_BG96_SocketClose(int32_t socket_no);
int32_t    R_CELLULAR_BG96_IsConnected (void);


int32_t bg96_serial_send_with_recvtask(uint8_t serial_ch_id, uint8_t *ptextstring, uint16_t response_type, uint16_t timeout_ms, bg96_return_code_t expect_code,  uint8_t command, uint8_t socket_no, uint32_t *len, uint8_t delay_flag);

// added 2020/10 start
void vStart_bg96_recv_task( void );
void bg96_response_set_queue( uint8_t command, uint8_t socket );
int8_t bg96_response_get_queue( uint8_t command, uint8_t socket, uint8_t *result, uint32_t *value);
// added 2020/10 end

void bg96_delete_recv_task( void );

// changed 2020/10 start
/////#define WIFI_CFG_CREATABLE_SOCKETS   		4
//#define WIFI_CFG_CREATABLE_SOCKETS   		2
#define WIFI_CFG_CREATABLE_SOCKETS   		5	//[RE01-test]
// changed 2020/10 end


extern bg96_socket_t g_bg96_socket[WIFI_CFG_CREATABLE_SOCKETS];

extern bg96_system_status_t g_bg96_system_state;

extern uint8_t macaddress[6];
extern uint8_t ipaddress[4];
extern uint8_t subnetmask[4];
extern uint8_t gateway[4];

extern uint32_t dnsaddress[4];

extern char g_bg96_m[3];
extern char g_bg96_d[3];
extern char g_bg96_h[3];
extern char g_bg96_min[3];

extern bg96_datetime_t g_bg96_time;

#define WIFI_SOCKET_IP_PROTOCOL_TCP (6)
#define WIFI_SOCKET_IP_VERSION_4    (4)

typedef enum
{
    WIFI_SOCKET_STATUS_CLOSED=0,	//CLOSED
    WIFI_SOCKET_STATUS_SOCKET,		//SOCKET
    WIFI_SOCKET_STATUS_BOUND,		//BOUND
    WIFI_SOCKET_STATUS_LISTEN,		//LISTEN
    WIFI_SOCKET_STATUS_CONNECTED,	//CONNECTED
    WIFI_SOCKET_STATUS_MAX,	        //MAX
} wifi_socket_status_t;

typedef enum
{
	WIFI_SECURITY_OPEN            = 0,
	WIFI_SECURITY_WEP,
	WIFI_SECURITY_WPA,
	WIFI_SECURITY_WPA2,
	WIFI_SECURITY_UNDEFINED,
}wifi_security_t;

typedef enum
{
	WIFI_EVENT_WIFI_REBOOT = 0,
	WIFI_EVENT_WIFI_DISCONNECT,
	WIFI_EVENT_SERIAL_OVF_ERR,
	WIFI_EVENT_SERIAL_FLM_ERR,
	WIFI_EVENT_SERIAL_RXQ_OVF_ERR,
	WIFI_EVENT_RCV_TASK_RXB_OVF_ERR,
	WIFI_EVENT_SOCKET_CLOSED,
	WIFI_EVENT_SOCKET_RXQ_OVF_ERR,
} wifi_err_event_enum_t;



#endif /* #define BG96_DRIVER_H */
