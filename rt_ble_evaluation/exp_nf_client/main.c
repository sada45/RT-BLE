/*
 * Copyright (C) 2019 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief       Client to test and benchmark raw L2CAP COC for NimBLE
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "nimble_riot.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/util/util.h"

#include "test_utils/expect.h"
#include "shell.h"
#include "thread.h"
#include "thread_flags.h"
#include "net/bluetil/ad.h"
#include "xtimer.h"
#include "controller/ble_ll_conn.h"
#include "controller/ble_ll.h"
#include "controller/ble_ll_ctrl.h"
#include "controller/ble_phy.h"
#include "sema.h"
#include "os/os_cputime.h"
#include "nimble_l2cap_test_conf.h"


#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define FLAG_UP             (1u << 0)
#define FLAG_UPD_START      (1u << 1)
#define FLAG_CONNECTED		(1u << 2)
//#define FLAG_TX_UNSTALLED   (1u << 2)
//#define FLAG_RECV           (1u << 3)
//#define FLAG_BREAK          (1u << 4)

#define UPD_FORHEAD         600

/* synchronization state */
static thread_t *_main;
static thread_t *conn_upd_thread;
static char conn_upd_thread_buf[1024];

/* buffer allocation */
static os_membuf_t _coc_mem[OS_MEMPOOL_SIZE(MBUFCNT, MBUFSIZE)];
static struct os_mempool _coc_mempool;
static struct os_mbuf_pool _coc_mbuf_pool;
static uint8_t connection_counter = 0;
static uint16_t connected_conn_handle;
static sema_t connected_sema;
static sema_t subrate_sema;

//struct pure_address{
//	uint8_t val1;
//	uint8_t val2;
//	uint8_t val3;
//	uint8_t val4;
//	uint8_t val5;
//	uint8_t val6;
//};
//uint8_t addr_type;

extern uint32_t base_tick;

struct node_conf{
	uint16_t central_pktlen;
	uint16_t peripheral_pktlen;
	uint32_t lat;
	double wcet_prob;
};


static struct node_conf conf_list[MYNEWT_VAL_BLE_L2CAP_COC_MAX_NUM] = {{C_LEN, P_LEN, LATENCY, PERCENTILE},
															 		   {C_LEN, P_LEN, LATENCY, PERCENTILE},
															 		   {C_LEN, P_LEN, LATENCY, PERCENTILE},
															 		   {C_LEN, P_LEN, LATENCY, PERCENTILE},
															 		   {C_LEN, P_LEN, LATENCY, PERCENTILE},
															 		   {C_LEN, P_LEN, LATENCY, PERCENTILE},
															 		   {C_LEN, P_LEN, LATENCY, PERCENTILE},
															 		   {C_LEN, P_LEN, LATENCY, PERCENTILE}};

static uint8_t device_address[MYNEWT_VAL_BLE_L2CAP_COC_MAX_NUM][6] ={{0xb7,0x5d,0x7a,0x2b,0xd8,0xe1},
																	 {0xfd,0xa7,0x01,0xbd,0xb8,0xc9},
																	 {0x4a,0x5d,0xc5,0x9c,0xd0,0xeb},
																	 {0x13,0x56,0x5f,0xf1,0x99,0xe4},
																	 {0x32,0x66,0xab,0xa7,0x68,0xf9},
																	 {0x2c,0xc9,0xbc,0x23,0x05,0xfe},
																	 {0x02,0x6c,0x49,0xb8,0x99,0xcf},
																	 {0x5c,0xdd,0x62,0xf4,0x33,0xfb}
																	};

static uint16_t handles[MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM)] = { 0 };
static struct ble_l2cap_chan *cocs[MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM)] = { 0 };

static struct ble_gap_conn_params gap_conn_params = {
    .scan_itvl = 0x0005,
    .scan_window = 0x0005,
    .itvl_min = 64,
    .itvl_max = 64,
    .latency = BLE_GAP_INITIAL_CONN_LATENCY,
    .supervision_timeout = BLE_GAP_INITIAL_SUPERVISION_TIMEOUT,
    .min_ce_len = BLE_GAP_INITIAL_CONN_MIN_CE_LEN,
    .max_ce_len = BLE_GAP_INITIAL_CONN_MAX_CE_LEN,
};

static void _on_data(struct ble_l2cap_event *event)
{
    int res;
    (void)res;
    struct os_mbuf *rxd = event->receive.sdu_rx;
    expect(rxd != NULL);
	//	uint32_t recv_time = os_cputime_ticks_to_usecs(os_cputime_get32());
	//	  for (int i = 0; i < MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM); i++){
	//		if (event->receive.conn_handle == handles[i] && device_type[i] == 1){
	//			oneshot_recv_num++;
	//			oneshot_delay[i] = recv_time - req_send_time[i];
	//			if (oneshot_recv_num == oneshot_num){
	////				thread_flags_set(collect_thread, FLAG_RECV);
	//			}
	//			break;
	//		}
	//	  }

    /* free buffer */
    res = os_mbuf_free_chain(rxd);
    expect(res == 0);
    rxd = os_mbuf_get_pkthdr(&_coc_mbuf_pool, 0);
    expect(rxd != NULL);
    res = ble_l2cap_recv_ready(event->receive.chan, rxd);
    expect(res == 0);
}

static int _on_l2cap_evt(struct ble_l2cap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
        case BLE_L2CAP_EVENT_COC_CONNECTED:;
            printf("# L2CAP: CONNECTED");
            printf("  conn num = %d\n", connection_counter);
            for (int i = 0; i < MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM); i++) {
				if (handles[i] == event->connect.conn_handle){
					cocs[i] = event->connect.chan;
					break;
				}
            }
			thread_flags_set(_main, FLAG_UP);
            break;
        case BLE_L2CAP_EVENT_COC_DISCONNECTED:
            printf("# L2CAP: DISCONNECTED");
			for (int i = 0; i < MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM); i++) {
				if (handles[i] == event->disconnect.conn_handle){
					handles[i] = 0;
					cocs[i] = NULL;
					break;
				}
			}
            printf("conn num = %d\n", connection_counter);
            break;
        case BLE_L2CAP_EVENT_COC_DATA_RECEIVED:
            _on_data(event);
            break;
        case BLE_L2CAP_EVENT_COC_TX_UNSTALLED:
//            thread_flags_set(collect_thread, FLAG_TX_UNSTALLED);
            break;
        case BLE_L2CAP_EVENT_COC_ACCEPT:
            /* this event should never be triggered for the L2CAP client */
            /* fallthrough */
        default:
            expect(0);
            break;
    }

    return 0;
}

static int _on_gap_evt(struct ble_gap_event *event, void *arg)
{
    (void)arg;
	ble_addr_t *addr;

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT: {
            connection_counter++;
            addr = (ble_addr_t *)arg;
            for (int i = 0; i < MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM); i++){
				if (memcmp(&addr->val, &device_address[i], 6) == 0 && handles[i] == 0){
					handles[i] = event->connect.conn_handle;
					break;
				}
            }
			free(addr);
            struct os_mbuf *sdu_rx = os_mbuf_get_pkthdr(&_coc_mbuf_pool, 0);
            expect(sdu_rx != NULL);
            ble_l2cap_connect(event->connect.conn_handle, APP_CID, APP_MTU, sdu_rx,
                                       _on_l2cap_evt, NULL);

            break;
        }
        case BLE_GAP_EVENT_DISCONNECT:
        	puts("====================================");
        	connection_counter--;
           	for (int i = 0; i < MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM); i++) {
				if (memcmp(&device_address[i], &(event->disconnect.conn.peer_id_addr.val), 6) == 0){
					printf("conn_handle = %d disconnect\n", handles[i]);
					handles[i] = 0;
					break;
				}
           	}
            break;
        case BLE_GAP_EVENT_CONN_UPDATE:
        	break;
        default:
            break;
    }

    return 0;
}

#if RTBLE
static int _cmd_subrate(int argc, char **argv)
{
	(void) argc;
	(void) argv;
	printf("subrate start\n");
	struct ble_ll_conn_sm *connsm = ble_ll_conn_find_by_handle(1);
	printf("error code=%d\n", rt_ble_subrating_tweak(connsm, 8, 0, 1));
	return 0;
}
#endif

static int _cmd_listanchor(int argc, char **argv)
{
	(void)argc;
	(void)argv;
	struct ble_ll_conn_sm *connsm;
	printf("cur_time = %lu\n", os_cputime_get32());
	for (int i = 0; i < MYNEWT_VAL_BLE_L2CAP_COC_MAX_NUM; i++) {
		connsm = ble_ll_conn_find_by_handle(i+1);
		printf("conn_handle = %d",i+1);
		if (connsm) {
			printf(" anchor = %lu\n", connsm->anchor_point);
		}
		else {
			printf(" NULL\n");
		}
	}
	return 0;
}

static const shell_command_t _cmds[] = {
//	{ "flood", "", _cmd_floodtest},
//	{"ls", "", _cmd_listanchor},
#if RTBLE
	{"sb", "", _cmd_subrate},
#endif
	{"ls", "", _cmd_listanchor},
    { NULL, NULL, NULL }
};

static int connected_cb(uint16_t conn_handle)
{
	connected_conn_handle = conn_handle;
	sema_post(&connected_sema);
	return 0;
}

int main(void)
{
    int res;
	uint16_t max_len;
	int packet_num;
    /* save context of the main */
    _main = thread_get_active();
    /* initialize buffers and setup the test environment */
    res = os_mempool_init(&_coc_mempool, MBUFCNT, MBUFSIZE, _coc_mem, "appbuf");
    expect(res == 0);
    res = os_mbuf_pool_init(&_coc_mbuf_pool, &_coc_mempool, MBUFSIZE, MBUFCNT);
    expect(res == 0);
	/* Start to connect */
	ble_ll_conn_set_connected_cb(connected_cb);
	ble_addr_t *target_addr = NULL;
	for (int i = 0; i < NODE_NUM; i++) {
		do {
			target_addr = (ble_addr_t *)malloc(sizeof(ble_addr_t));
			target_addr->type = BLE_ADDR_RANDOM;
			memcpy(&target_addr->val, &device_address[i], 6);
			printf("connect to device %d\n", i+1);
			max_len = MAX(conf_list[i].central_pktlen, conf_list[i].peripheral_pktlen);
			packet_num = (max_len + 2 - 1) / 247 + 1;
			double ci = (conf_list[i].lat * 1000 - 5004) / (CONFIG_NF * packet_num);
			gap_conn_params.itvl_max = (uint16_t)(ci / 1250);
			gap_conn_params.itvl_min = gap_conn_params.itvl_max;
			printf("est transmit connection interval = %d\n", gap_conn_params.itvl_max);
			res = ble_gap_connect(nimble_riot_own_addr_type, target_addr, BLE_HS_FOREVER,
						&gap_conn_params, _on_gap_evt, target_addr);
			expect(res == 0);
			thread_flags_wait_all(FLAG_UP);
			xtimer_msleep(500);
			if (connection_counter < (i+1)) {
				continue;
			}
			res = sema_wait_timed_ztimer(&connected_sema, ZTIMER_USEC, 1000000UL);
			if (res != 0) {
				puts("connected sema err");
				ble_gap_conn_cancel();
			}
			xtimer_msleep(500);
		} while(connection_counter < (i+1));
	}
	printf("All devices connected\n");
	char line_buf[512];
    shell_run(_cmds, line_buf, SHELL_DEFAULT_BUFSIZE);
    return 0;
}
