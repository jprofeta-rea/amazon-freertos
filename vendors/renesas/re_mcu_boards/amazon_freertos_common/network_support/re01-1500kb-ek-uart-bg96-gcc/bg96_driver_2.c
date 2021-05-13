#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "FreeRTOSIPConfig.h"
#include "platform.h"
#include "r_byteq_if.h"
#include "bg96_driver.h"
#include "bg96_common.h"

#include "R_Driver_USART.h"

#define BG96_DATA_RECEIVE         "+QIURC: \"recv\","
#define BG96_DATA_RECEIVE_COMMAND "+QIURC: \"recv\",%d"
#define BG96_READ_MAC             "+CIPAPMAC"
#define BG96_READ_IP              "+CIPSTA"
#define BG96_READ_SNTP            "+QLTS:"
#define BG96_READ_DNS             "+QIURC: \"dnsgip\","
#define BG96_READ_TCP             "+QIOPEN:"
#define BG96_READ_CSQ             "+CSQ:"
#define BG96_DATA_RECEIVE_QIRD    "+QIRD:"
#define BG96_CLOSE         		  "+QIURC: \"closed\","

void bg96_response_set_result( uint8_t result, uint32_t value);
uint8_t bg96_response_get_now_queue(void);
uint8_t bg96_response_get_now_queue_socket(void);

extern uint8_t recvbuff[2048+20];
uint32_t recv_count;
uint32_t tmp_recvcnt;
uint8_t receive_status;
uint8_t receive_sub_status1;
uint8_t receive_sub_status2;
uint8_t recv_continue;

bg96_socket_t g_bg96_socket[WIFI_CFG_CREATABLE_SOCKETS];

uint8_t macaddress[6];
uint8_t ipaddress[4];
uint8_t subnetmask[4];
uint8_t gateway[4];

uint32_t dnsaddress[4];

uint8_t responce_wait_queue[10][3];
uint32_t responce_wait_queue_v[10];
uint8_t responce_wait_queue_set_index;
uint8_t responce_wait_queue_wait_index;

char g_bg96_m[3];
char g_bg96_d[3];
char g_bg96_h[3];
char g_bg96_min[3];
bg96_datetime_t g_bg96_time;

uint32_t length = 0;

static void bg96_recv_task( void * pvParameters );
TaskHandle_t g_bg96_recv_task_handle;

// added 2020/10 start
extern ARM_DRIVER_USART Driver_USART5;
static ARM_DRIVER_USART *gsp_sci5_dev = &Driver_USART5;
// added 2020/10 end

// added 2020/10 start
// changed 2020/10 start (メモリ削減)
/////uint8_t data_deb[1600];
// changed 2020/10 start (メモリ削減)
/////uint8_t data_deb[5120]; // RSA OK (4096 では溢れる場合あり)
uint8_t data_deb[6144]; // RSA OK (5120 では溢れる場合あり)
// changed 2020/10 end (メモリ削減)
// changed 2020/10 end (メモリ削減)
// added 2020/10 end

// added 2020/10 start
void bg96_usart_abort(uint32_t *new_cnt, uint32_t *pre_cnt);
// added 2020/10 end

void vStart_bg96_recv_task( void )
{
    BaseType_t x;

    /* Create the echo client tasks. */
    xTaskCreate( bg96_recv_task,                               /* The function that implements the task. */
                 "bg96_recv",                                     /* Just a text name for the task to aid debugging. */
// changed 2020/10 start (メモリ削減)
/////                 2048,
				 (configMINIMAL_STACK_SIZE * 4),
// changed 2020/10 end (メモリ削減)
                 ( void * ) 0,                                    /* The task parameter, not used in this case. */
// changed 2020/10 start
/////                 tskIDLE_PRIORITY + 6,
//[RE01-test]                 tskIDLE_PRIORITY + 5,
                 tskIDLE_PRIORITY + 6,	//[RE01-test]
// changed 2020/10 end
                 &g_bg96_recv_task_handle );                                          /* The task handle is not used. */
}

void bg96_delete_recv_task( void )
{
    /* Delete wifi driver at response tasks. */
    if( NULL != g_bg96_recv_task_handle )
    {
    	vTaskDelete(g_bg96_recv_task_handle);
    	g_bg96_recv_task_handle = NULL;
    }
}

static void bg96_recv_task( void * pvParameters )
{
	uint8_t data;
// changed 2020/10 start
// RX FIT module
/////	sci_err_t ercd;
	uint8_t ercd;
// changed 2020/10 end
	char * string_ercd;
    char recv_tmp[50];
    char recv_tmp_qird[50];
	int sscanf_ret;
	int i;
	uint32_t dns_cnt, dns_max;
	uint32_t socket_no = 0;
	uint32_t close_socket_no = 0;
	uint32_t len;

// changed 2020/10 start
	// RX FIT module
	byteq_err_t byteq_ercd;
/////	uint8_t byteq_ercd;
// changed 2020/10 end

    uint8_t of_flag = 0;
    uint8_t rcv_flag = 0;
    uint32_t recv_all_cnt_bk; 

    uint32_t task_st_num;

	memset(recvbuff,0,sizeof(recvbuff));
	recv_count = 0;
	dns_cnt = 0;
	dns_max = 0;

// added 2020/10 start
	ARM_USART_STATUS sci_status_rx;
	uint32_t pre_rx_cnt = 0;
	uint32_t new_rx_cnt = 0;
	uint32_t check_new_rx_cnt = 0;
// added 2020/10 end

    for( ; ; )
    {

// changed 2020/10 start

    	sci_status_rx = gsp_sci5_dev->GetStatus();

    	if ( sci_status_rx.rx_busy == 0 )
    	{
// changed 2020/10 start (メモリ削減)
/////    		ercd = gsp_sci5_dev->Receive(&data_deb[0], 1600);
// changed 2020/10 start (メモリ削減)
/////    		ercd = gsp_sci5_dev->Receive(&data_deb[0], 5120); // RSA OK (4096 では溢れる場合あり)
    		ercd = gsp_sci5_dev->Receive(&data_deb[0], 6144); // RSA OK (5120 では溢れる場合あり)
// changed 2020/10 end (メモリ削減)
// changed 2020/10 end (メモリ削減)

    	    if(ARM_DRIVER_ERROR_BUSY == ercd)
    	    {
    	    	vTaskDelay( 1 );
    	        continue;
    	    }
    	    else if (ARM_DRIVER_OK == ercd)
    	    {
    	        ;
    	    }
    	    else
    	    {
    	    	while(1);
    	    }
    	}

    	new_rx_cnt = gsp_sci5_dev->GetRxCount();

    	 if (new_rx_cnt != pre_rx_cnt)
// changed 2020/10 end
    	{
    		if (recv_count >= (sizeof(recvbuff) - 1))
            {
                of_flag = 1;
            }
            else
            {
/////                recvbuff[recv_count] = data;
/////                recv_count++;
            }
#if DEBUGLOG == 2
    		putchar(data);
#endif

// added 2020/10 start
    	    for (int i = pre_rx_cnt; i < new_rx_cnt; i++)
    	    {
    	    	recvbuff[recv_count] = data_deb[i];

    	    	data = data_deb[i];
    	    	recv_count++;
// added 2020/10 end

    		switch(receive_status)
    		{
	    		case 0:
		    		switch(data)
		    		{
		    			case '\r':
		    				receive_status = 1;
			    			receive_sub_status1 = 0;
			    			receive_sub_status2 = 0;
		    			break;
		    			case '+':
		    				receive_status = 2;
			    			receive_sub_status1 = 0;
			    			receive_sub_status2 = 0;
			    			tmp_recvcnt = 0;
		    			break;
		    			case '>':
		    				receive_status = 9;
			    			receive_sub_status1 = 0;
			    			receive_sub_status2 = 0;
			    			break;
		    			default:
		    				receive_status = 0xff;
			    			receive_sub_status1 = 0;
			    			receive_sub_status2 = 0;
		    			break;
		    		}
    			break;
	    		case 1:	/* \rから始まる \r\n????\r\n */
	    			switch(receive_sub_status1)
	    			{
		    			case 0:	/*\n待ち*/
		    				if(data == '\n')
		    				{
		    					receive_sub_status1 = 1;
		    				}
		    				break;
		    			case 1:	/*後半\r\n or > 待ち*/
		    				if(data == '+')
		    				{
                                // \r\n+... の場合   次のステートからは +... で処理をする
		    					receive_status = 2;
		    					receive_sub_status1 = 0;
		    					tmp_recvcnt = recv_count-1;
		    				}
		    				else
		    				{
		    					receive_sub_status1 = 2;
		    				}
		    				break;
		    			case 2:	/*後半\r\n or > 待ち*/
							{
								string_ercd = strstr(&recvbuff[2],"\r\n");
								if(string_ercd != NULL)
								{
                                    // AT+QIOPEN  =TCP接続コマンド時
                                    // 一旦 \r\nOK\r\n を破棄して次のメッセージを受け取る
									if (BG96_SET_CIPSTART == bg96_response_get_now_queue()) {
										tmp_recvcnt = 0;
									}
                                    // AT+QIDNSGIP  =DNS取得コマンド時
                                    // 一旦 \r\nOK\r\n を破棄して次のメッセージを受け取る
									else if(BG96_SET_CIPDOMAIN == bg96_response_get_now_queue())
									{
										tmp_recvcnt = 0;
									}
									else if(0 == strncmp(&recvbuff[2],BG96_RETURN_TEXT_OK,4))
									{
										if(BG96_SET_CIPSEND == bg96_response_get_now_queue())
										{
                                            // AT+QISEND  =TCP送信コマンド時   "\r\nOK\r\n> " 待ち
					    					receive_status = 8;
		   					    			tmp_recvcnt = recv_count;
											break;
										}
										else if (BG96_SET_CIPDOMAIN == bg96_response_get_now_queue())
										{
					    					receive_status = 2;
		   					    			tmp_recvcnt = recv_count;
											break;
										}
										else
										{
											/* 通常のOK　*/
											bg96_response_set_result( BG96_RETURN_OK, 0 );
										}
									}
									else if(0 == strncmp(&recvbuff[2],BG96_RETURN_TEXT_ERROR,7))
									{
										bg96_response_set_result( BG96_RETURN_ERROR, 0 );
									}
									else if(0 == strncmp(&recvbuff[2],BG96_RETURN_TEXT_SEND_OK,9))
									{
										bg96_response_set_result( BG96_RETURN_SEND_OK, 0 );
									}
									else if(0 == strncmp(&recvbuff[2],BG96_RETURN_TEXT_SEND_FAIL,11))
									{
										bg96_response_set_result( BG96_RETURN_SEND_FAIL, 0 );
									}
									memset(recvbuff,0,sizeof(recvbuff));
									recv_count = 0;
			    					receive_status = 0;
					    			receive_sub_status1 = 0;
					    			receive_sub_status2 = 0;
// added 2020/10 start
					    			bg96_usart_abort(&new_rx_cnt, &pre_rx_cnt);
// added 2020/10 end
								}
                                //
								else if (0 == strncmp(recvbuff, BG96_RETURN_TEXT_OK_GO_SEND, 4))
								{
									bg96_response_set_result( BG96_RETURN_OK_GO_SEND, 0 );
									memset(recvbuff,0,sizeof(recvbuff));
									recv_count = 0;
			    					receive_status = 0;
					    			receive_sub_status1 = 0;
					    			receive_sub_status2 = 0;
// added 2020/10 start
					    			bg96_usart_abort(&new_rx_cnt, &pre_rx_cnt);
// added 2020/10 end
								}
							}
		    				break;
		    			default :
							memset(recvbuff,0,sizeof(recvbuff));
							recv_count = 0;
							tmp_recvcnt = 0;
	    					receive_status = 0;
			    			receive_sub_status1 = 0;
			    			receive_sub_status2 = 0;
// added 2020/10 start
			    			bg96_usart_abort(&new_rx_cnt, &pre_rx_cnt);
// added 2020/10 end
			    			break;
	    			}
    				break;
	    		case 2:	/* +からはじまる　 +?????:???? */
	    			switch(receive_sub_status1)
	    			{
		    			case 0:	/* ':'待ち */
			    			if(data == ':')
		    				{
                                // TCP接続 +QIOPEN: 確認
                                if (0 == strcmp(&recvbuff[tmp_recvcnt], BG96_READ_TCP))
                                {
                                    receive_sub_status1 = 2;
                                }
                                // +QLTS: 現在時刻
   					    		else if (0 == strcmp(&recvbuff[tmp_recvcnt], BG96_READ_SNTP))
   					    		{
   					    			receive_status = 6;
   					    			receive_sub_status1 = 0;
   					    			tmp_recvcnt = recv_count;
   					    		}
								// +QIRD: 受信
								else if (0 == strcmp(&recvbuff[tmp_recvcnt], BG96_DATA_RECEIVE_QIRD))
								{
									receive_sub_status1 = 4;
									tmp_recvcnt = recv_count;
								}
                                else
                                {
                                    receive_sub_status1 = 1;
                                }
		    				}
	    					break;
		    			case 1:	/* ','待ち */
			    			if(data == ',')
		    				{
                                // TCP受信 +QIURC: "recv", 確認
								if (0 == strcmp(&recvbuff[tmp_recvcnt], BG96_DATA_RECEIVE))
								{
									receive_sub_status1 = 3;

								}
                                // DNS取得 +QIURC: "dnsgip", 確認
								else if (0 == strcmp(&recvbuff[tmp_recvcnt], BG96_READ_DNS))
								{
									receive_status = 7;
									receive_sub_status1 = 0;
									tmp_recvcnt = recv_count;
								}
                                // TCP クローズ受信  +QIURC: "close", 確認
								else if (0 == strcmp(&recvbuff[tmp_recvcnt], BG96_CLOSE))
								{
									receive_sub_status1 = 5;
									tmp_recvcnt = recv_count;
								}
								else
								{
									memset(recvbuff,0,sizeof(recvbuff));
									recv_count = 0;
			    					receive_status = 0;
					    			receive_sub_status1 = 0;
					    			receive_sub_status2 = 0;
// added 2020/10 start
					    			bg96_usart_abort(&new_rx_cnt, &pre_rx_cnt);
// added 2020/10 end
								}
		    				}

		    				break;
		    			case 2:	/* "\r\n" 待ち */
							string_ercd = strstr(&recvbuff[2],"\r\n");
							if(string_ercd != NULL)
							{
								memset(recv_tmp, 0x00, sizeof(recv_tmp));
								memset(recv_tmp_qird, 0x00, sizeof(recv_tmp_qird));
                                // TCP接続返信 ソケット番号とエラー0を確認
                                socket_no = bg96_response_get_now_queue_socket();
								sprintf(recv_tmp, "%s %d,0\r\n", BG96_READ_TCP, bg96_response_get_now_queue_socket());
								if (0 == strcmp(&recvbuff[tmp_recvcnt], recv_tmp))
								{
									bg96_response_set_result( BG96_RETURN_OK, 0 );
								}

								memset(recvbuff,0,sizeof(recvbuff));
								recv_count = 0;
								receive_status = 0;
								receive_sub_status1 = 0;
								receive_sub_status2 = 0;
// added 2020/10 start
								bg96_usart_abort(&new_rx_cnt, &pre_rx_cnt);
// added 2020/10 end
                            }
		    				break;
		    			case 3:	/* recv の "\r\n" 待ち */
							string_ercd = strstr(&recvbuff[2],"\r\n");
							if(string_ercd != NULL)
							{
                                // TCPデータ受信 +QIURC: "recv" のソケット番号とデータサイズ取得
	   					    	sscanf_ret = sscanf(&recvbuff[tmp_recvcnt], BG96_DATA_RECEIVE_COMMAND, &socket_no);
	   					    	if(sscanf_ret == 1)
	   					    	{
	   					    		g_bg96_socket[socket_no].receive_count = 0;

                                    g_bg96_socket[socket_no].qiurc_rcv_flg = 1;
	   					    		receive_status = 0;
	   					    		receive_sub_status1 = 0;
									memset(recvbuff,0,sizeof(recvbuff));
									recv_count = 0;
									receive_status = 0;
									receive_sub_status1 = 0;
									receive_sub_status2 = 0;
	   					    	}
	   					    	else
	   					    	{
									memset(recvbuff,0,sizeof(recvbuff));
									recv_count = 0;
									receive_status = 0;
									receive_sub_status1 = 0;
									receive_sub_status2 = 0;
	   					    	}
                            }
		    				break;
		    			case 4:/* Buffer Access Mode recv の "\r\n" 待ち */
							string_ercd = strstr(&recvbuff[2],"\r\n");
							if(string_ercd != NULL)
							{
								sscanf_ret = sscanf(&recvbuff[tmp_recvcnt]," %d" , &len);
								if(sscanf_ret == 1)
	   					    	{
                                    socket_no = bg96_response_get_now_queue_socket();
                                    if (len == 0)
									{
										g_bg96_socket[socket_no].qiurc_rcv_flg = 0;
									}
	   					    		g_bg96_socket[socket_no].receive_num = len;
	   					    		g_bg96_socket[socket_no].receive_count = 0;
									length = len;
	   					    		receive_status = 3;
	   					    		receive_sub_status1 = 0;
	   					    		bg96_response_set_result( BG96_RETURN_OK, len);
	   					    	}
	   					    	else
	   					    	{
									memset(recvbuff,0,sizeof(recvbuff));
									recv_count = 0;
									receive_status = 0;
									receive_sub_status1 = 0;
									receive_sub_status2 = 0;
									bg96_response_set_result( BG96_RETURN_ERROR, 0 );
// added 2020/10 start
									bg96_usart_abort(&new_rx_cnt, &pre_rx_cnt);
// added 2020/10 end
	   					    	}
                            }
		    				break;
		    			case 5:	/* "\r\n" 待ち */
							string_ercd = strstr(&recvbuff[2],"\r\n");
							if(string_ercd != NULL)
							{
								sscanf_ret = sscanf(&recvbuff[tmp_recvcnt]," %d" , &close_socket_no);
								memset(recvbuff,0,sizeof(recvbuff));
								recv_count = 0;
								receive_status = 0;
								receive_sub_status1 = 0;
								receive_sub_status2 = 0;
								bg96_response_set_result( BG96_RETURN_ERROR, 0 );
// added 2020/10 start
								bg96_usart_abort(&new_rx_cnt, &pre_rx_cnt);
// added 2020/10 end
                            }
		    				break;

		    			default :
							memset(recvbuff,0,sizeof(recvbuff));
							recv_count = 0;
							tmp_recvcnt = 0;
	    					receive_status = 0;
			    			receive_sub_status1 = 0;
			    			receive_sub_status2 = 0;
// added 2020/10 start
			    			bg96_usart_abort(&new_rx_cnt, &pre_rx_cnt);
// added 2020/10 end
			    			break;
	    			}
	    			break;
	    		case 3:	/* TCPデータ受信 */
// changed 2020/10 start
                    if (g_bg96_socket[socket_no].receive_count < g_bg96_socket[socket_no].receive_num)
                    {
                        byteq_ercd = R_BYTEQ_Put(g_bg96_socket[socket_no].socket_byteq_hdl, data);
                        if(byteq_ercd != BYTEQ_SUCCESS)
                        {
                            g_bg96_socket[socket_no].put_error_count++;
                        }
                        g_bg96_socket[socket_no].receive_count++;
                    }
                    else
                    {
                        if (recv_count >= 4)
                        {
                            tmp_recvcnt = recv_count - 4;
                        }
                        else
                        {
                            tmp_recvcnt = 0;
                        }

                        if (strstr(&recvbuff[tmp_recvcnt], "OK\r\n") != NULL)
                        {
                            memset(recvbuff,0,sizeof(recvbuff));
                            recv_count = 0;
                            receive_status = 0;
                            receive_sub_status1 = 0;
                            receive_sub_status2 = 0;
                            g_bg96_socket[socket_no].receive_count = 0;
                            g_bg96_socket[socket_no].receive_num = 0;

// changed 2020/10 start
                            bg96_usart_abort(&new_rx_cnt, &pre_rx_cnt);
// changed 2020/10 end
                        }
                    }
// changed 2020/10 end
	    			break;
	    		case 6:	// QLTS
					if(data == '\n')
					{
                        char year[5]  = {0};
                        char sspzz[6] = {0};
                        char dst      = 0;
                        memset(g_bg96_time.year, 0x00, sizeof(g_bg96_time.year));
                        memset(g_bg96_time.month, 0x00, sizeof(g_bg96_time.month));
                        memset(g_bg96_time.day, 0x00, sizeof(g_bg96_time.day));
                        memset(g_bg96_time.hour, 0x00, sizeof(g_bg96_time.hour));
                        memset(g_bg96_time.min, 0x00, sizeof(g_bg96_time.min));
                        memset(g_bg96_time.sec, 0x00, sizeof(g_bg96_time.sec));

                        sscanf(&recvbuff[tmp_recvcnt], " \"%4s/%2s/%2s,%2s:%2s:%5s,%c\"\r\n",
                                year, g_bg96_time.month, g_bg96_time.day, g_bg96_time.hour, g_bg96_time.min, sspzz, &dst);

                        memcpy(g_bg96_time.year, year, 4);
                        memcpy(g_bg96_time.sec, sspzz, 2);

                        memset(recvbuff,0,sizeof(recvbuff));
						recv_count = 0;
						receive_status = 0;
						receive_sub_status1 = 0;
						receive_sub_status2 = 0;
// added 2020/10 start
						bg96_usart_abort(&new_rx_cnt, &pre_rx_cnt);
// added 2020/10 end
					}
					break;
	    		case 7:	/* DNS */
	    			if(NULL != strstr(&recvbuff[tmp_recvcnt],"\r\n"))
					{
                        // +QIURC: "dnsgip" は、最初にDNSの個数が返信されるので、elseでDNS個数を取得
                        // bg96_response_set_result() が複数回実行されると誤動作するので、max時に更新する
	    				if (recvbuff[tmp_recvcnt] == '"') {
	    					dns_cnt++;
	    					sscanf(&recvbuff[tmp_recvcnt],"\"%d.%d.%d.%d\"\r\n",&dnsaddress[0],&dnsaddress[1],&dnsaddress[2],&dnsaddress[3]);
	    					if (dns_cnt >= dns_max) {
	    						bg96_response_set_result( BG96_RETURN_OK, 0 );
								dns_cnt = 0;
								dns_max = 0;
// added 2020/10 start
								bg96_usart_abort(&new_rx_cnt, &pre_rx_cnt);
// added 2020/10 end
	    					}
	    				}
	    				else {
	    					uint32_t dummy;
	    					sscanf(&recvbuff[tmp_recvcnt],"0,%d,%d\r\n",&dns_max,&dummy);   // dummyはDNS有効期限(不使用)
	    				}
						memset(recvbuff,0,sizeof(recvbuff));
						recv_count = 0;
						receive_status = 0;
						receive_sub_status1 = 0;
						receive_sub_status2 = 0;
					}
					break;
	    		case 8:	/* CIPSEND -> OK\r\n>  */
	    			if(0 == strcmp(&recvbuff[tmp_recvcnt],"> "))
					{
						bg96_response_set_result( BG96_RETURN_OK_GO_SEND, 0 );
						memset(recvbuff,0,sizeof(recvbuff));
						recv_count = 0;
						receive_status = 0;
						receive_sub_status1 = 0;
						receive_sub_status2 = 0;
// added 2020/10 start
						bg96_usart_abort(&new_rx_cnt, &pre_rx_cnt);
// added 2020/10 end
					}
					break;
	    		case 9:	/* CIPSEND -> OK\r\n>  */
					if(data == ' ')
					{
						bg96_response_set_result( BG96_RETURN_OK_GO_SEND, 0 );
						memset(recvbuff,0,sizeof(recvbuff));
						recv_count = 0;
						receive_status = 0;
						receive_sub_status1 = 0;
						receive_sub_status2 = 0;
// added 2020/10 start
						bg96_usart_abort(&new_rx_cnt, &pre_rx_cnt);
// added 2020/10 end
					}
					break;
	    		case 0xff:	/* その他  ????\r\n */
	    			switch(receive_sub_status1)
	    			{
		    			case 0:	/* \r\n待ち */
		    				string_ercd = strstr(&recvbuff[1],"\r\n");
		    				if(string_ercd != NULL)
		    				{
								if(NULL != strstr(&recvbuff[1],BG96_RETURN_TEXT_SEND_FAIL))
								{
									bg96_response_set_result( BG96_RETURN_SEND_FAIL, 0 );
								}

		    					/* 読み捨て */
								memset(recvbuff,0,sizeof(recvbuff));
								recv_count = 0;
		    					receive_status = 0;
				    			receive_sub_status1 = 0;
				    			receive_sub_status2 = 0;
// added 2020/10 start
				    			bg96_usart_abort(&new_rx_cnt, &pre_rx_cnt);
// added 2020/10 end
		    				}
		    				break;
	    			}
	    			break;
	    		default:
					memset(recvbuff,0,sizeof(recvbuff));
					recv_count = 0;
					receive_status = 0;
	    			receive_sub_status1 = 0;
	    			receive_sub_status2 = 0;
// added 2020/10 start
	    			bg96_usart_abort(&new_rx_cnt, &pre_rx_cnt);
// added 2020/10 end
    				break;
    		    }
// added 2020/10 start
            }
    	    pre_rx_cnt = new_rx_cnt;
// added 2020/10 end
    	}
    	else
    	{
// added 2020/10 start
    	    vTaskDelay( 1 );
// added 2020/10 end
    	}
    }	
}

void bg96_response_set_queue( uint8_t command, uint8_t socket )
{
	responce_wait_queue[responce_wait_queue_set_index][0] = command;
	responce_wait_queue[responce_wait_queue_set_index][1] = socket;
	responce_wait_queue[responce_wait_queue_set_index][2] = 0xff;
	responce_wait_queue_v[responce_wait_queue_set_index] = 0;

	responce_wait_queue_set_index++;
	if(responce_wait_queue_set_index >= 10)
	{
		responce_wait_queue_set_index = 0;
	}
}

int8_t bg96_response_get_queue( uint8_t command, uint8_t socket, uint8_t *result, uint32_t *value)
{
	int i;

	for(i = 0;i<10;i++)
	{
		if(responce_wait_queue[i][0] == command && responce_wait_queue[i][1] == socket)
		{
			if((responce_wait_queue[i][2] != 0xff) && (responce_wait_queue[i][2] != 0xfe))
			{
				*result = responce_wait_queue[i][2];
				*value = responce_wait_queue_v[i];
				responce_wait_queue[i][2] = 0xfe;
				break;
			}
		}
	}
	if(i>= 10)
	{
		return -1;
	}
	return 0;
}

uint8_t bg96_response_get_now_queue(void)
{
	return 	responce_wait_queue[responce_wait_queue_wait_index][0];
}

uint8_t bg96_response_get_now_queue_socket(void)
{
	return 	responce_wait_queue[responce_wait_queue_wait_index][1];
}

void bg96_response_set_result( uint8_t result, uint32_t value)
{
	responce_wait_queue[responce_wait_queue_wait_index][2] = result;
	responce_wait_queue_v[responce_wait_queue_wait_index] = value;

	responce_wait_queue_wait_index++;
	if(responce_wait_queue_wait_index >= 10)
	{
		responce_wait_queue_wait_index = 0;
	}
}

// added 2020/10 start
void bg96_usart_abort(uint32_t *new_cnt, uint32_t *pre_cnt)
{
	uint32_t check_new_rx_cnt;

    check_new_rx_cnt = gsp_sci5_dev->GetRxCount();
    if (*new_cnt == check_new_rx_cnt)
    {
        gsp_sci5_dev->Control(ARM_USART_ABORT_RECEIVE, 0);
        *pre_cnt = 0;
        *new_cnt = 0;
        check_new_rx_cnt = 0;
    }
}
// added 2020/10 end
