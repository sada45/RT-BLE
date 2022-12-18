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
#include "rt_ble.h"
#include "os/os_cputime.h"



#include "nimble_l2cap_test_conf.h"

#define FLAG_UP             (1u << 0)
#define FLAG_SYNC           (1u << 1)
#define FLAG_TX_UNSTALLED   (1u << 2)
#define FLAG_CONTINUE       (1u << 3)
#define FLAG_ENTER_CRITICAL	(1u << 4)
#define FLAG_LEAVE_CRTTICAL	(1u << 5)

#define UPD_FORHEAD         600

/* synchronization state */
static thread_t *_main;
static thread_t *collect_thread;

/* buffer allocation */
static os_membuf_t _coc_mem[OS_MEMPOOL_SIZE(MBUFCNT, MBUFSIZE)];
static struct os_mempool _coc_mempool;
static struct os_mbuf_pool _coc_mbuf_pool;
static uint8_t connection_counter = 0;
static uint16_t connected_conn_handle;
static sema_t connected_sema;
static sema_t subrate_sema;


extern uint32_t base_tick;

struct node_conf{
	uint16_t central_pktlen;
	uint16_t peripheral_pktlen;
	uint32_t lat;
	uint32_t percentile;
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

static kernel_pid_t txend_output_pid;
static thread_t *txend_output_thread = NULL;
static char thread_buf[2048];


static struct ble_gap_conn_params gap_conn_params = {
    .scan_itvl = 0x0005,
    .scan_window = 0x0005,
    .itvl_min = 8,
    .itvl_max = 8,
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
           thread_flags_set(_main, FLAG_TX_UNSTALLED);
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
					printf("conn_handle = %d disconnect, reason = %d\n", handles[i], event->disconnect.reason);
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

#if C_LEN
static int ble_send(struct ble_l2cap_chan *chan, uint8_t *data, int len){
    int res = 0;
    struct os_mbuf *txd;
    if (chan == NULL){
		return 0;
    }
	txd = os_mbuf_get_pkthdr(&_coc_mbuf_pool, 0);
	expect(txd != NULL);
	res = os_mbuf_append(txd, data, len);
	expect(res == 0);
	do {
        res = ble_l2cap_send(chan, txd);
        if (res == BLE_HS_EBUSY) {
            thread_flags_wait_all(FLAG_TX_UNSTALLED);
        }
    } while (res == BLE_HS_EBUSY);
    if ((res != 0) && (res != BLE_HS_ESTALLED)) {
        expect(0);
	}
	return len;
}

void *txend_output_func(void *args) 
{
    (void)args;
    ble_ll_sema_consume();
    while(1) {
        ble_ll_conn_wait_start_tx_sema();
        ble_ll_conn_wait_output_sem();
    }
}
#endif


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
	printf("base_tick = %lu, cur_time = %lu\n", base_tick+7, os_cputime_get32());
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


void *txend_output_func(void *args) 
{
    (void)args;
    ble_ll_sema_consume();
    while(1) {
        ble_ll_conn_wait_start_tx_sema();
        ble_ll_conn_wait_output_sem();
    }
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

#if RTBLE
static void subrate_rx_cb(uint16_t conn_handle)
{
	connected_conn_handle = conn_handle;
	sema_post(&subrate_sema);
	return;
}
#endif

static int connected_cb(uint16_t conn_handle)
{
	connected_conn_handle = conn_handle;
	sema_post(&connected_sema);
	return 0;
}

int main(void)
{
    int res;
	rt_ble_conn_t *conn;
	uint16_t cmd_flag;

    /* save context of the main */
    _main = thread_get_active();
    sema_create(&connected_sema, 0);
    sema_create(&subrate_sema, 0);
    /* initialize buffers and setup the test environment */
    res = os_mempool_init(&_coc_mempool, MBUFCNT, MBUFSIZE, _coc_mem, "appbuf");
    expect(res == 0);
    res = os_mbuf_pool_init(&_coc_mbuf_pool, &_coc_mempool, MBUFSIZE, MBUFCNT);
    expect(res == 0);

	printf("REPORT_INTERVAL=%d\n", REPORT_INTERVAL);
	printf("C_LEN=%d\n", C_LEN);
	printf("P_LEN=%d\n", P_LEN);
	printf("LATENCY=%d\n", LATENCY);
	printf("PERCENTILE=%d\n", PERCENTILE);
	printf("PKT_LOSS=%d\n", PKT_LOSS_RATE);
	/* Start to connect */
	ble_ll_conn_set_connected_cb(connected_cb);
#if RTBLE
	ble_ll_conn_set_subrate_callback(subrate_rx_cb);
	rt_ble_tree_init();
#endif
	ble_addr_t *target_addr = NULL;
	for (int i = 0; i < 1; i++) {
		// conn = rt_ble_add_new_conn(i+1, conf_list[i].central_pktlen, conf_list[i].peripheral_pktlen,
		// 					conf_list[i].lat, conf_list[i].percentile);
		conn = rt_ble_add_test_conn(CONF_LEVEL);
		printf("handle=%d, level=%d, offset=%d, slot_num=%d\n", conn->conn_handle, conn->level, conn->offset, conn->slot_num);
		if (!conn) {
			printf("There is no enough resource for device-%d\n", i);
			xtimer_msleep(100);
			continue;
		}

		do {
			target_addr = (ble_addr_t *)malloc(sizeof(ble_addr_t));
			target_addr->type = BLE_ADDR_RANDOM;
			memcpy(&target_addr->val, &device_address[i], 6);
			printf("connect to device %d, offset = %d\n", i+1, conn->offset);
#if RTBLE
			gap_conn_params.offset = conn->offset;
#endif
			res = ble_gap_connect(nimble_riot_own_addr_type, target_addr, BLE_HS_FOREVER,
						&gap_conn_params, _on_gap_evt, target_addr);
			expect(res == 0);
			thread_flags_wait_all(FLAG_UP);
			xtimer_msleep(500);
			if (connection_counter < (i+1)) {
				ble_gap_conn_cancel();
				continue;
			}
			res = sema_wait_timed_ztimer(&connected_sema, ZTIMER_USEC, 1000000UL);
			if (res != 0) {
				puts("connected sema err");
				ble_gap_conn_cancel();
			}
			res = rt_ble_connected(connected_conn_handle, conf_list[i].peripheral_pktlen);
			if (res != 0) {
				puts("connected err");
				ble_gap_conn_cancel();
			}
			res = sema_wait_timed_ztimer(&subrate_sema, ZTIMER_USEC, 5000000UL);
			if (res != 0) {
				puts("subrate sema err");
				ble_gap_conn_cancel();
			}
			res = rt_ble_update_callback(i+1);
			if (res != 0) {
				puts("update callback err");
				ble_gap_conn_cancel();
			}
			xtimer_msleep(100);
		} while(connection_counter < (i+1));
	}
	printf("All devices connected\n");
	ble_phy_clear();
	txend_output_pid = thread_create(thread_buf, sizeof(thread_buf), THREAD_PRIORITY_MAIN-1,
								THREAD_CREATE_STACKTEST, txend_output_func, NULL, "periodic report");
	txend_output_thread = thread_get(txend_output_pid);
	_cmd_listanchor(0, NULL);
	struct ble_ll_conn_sm *connsm = ble_ll_conn_find_by_handle(1);
	for (int i = 0; i < 50; i++) {
		printf("\nmove-even-%d, start_move=%lu", i, os_cputime_ticks_to_usecs(os_cputime_get32()));
		rt_ble_move_right_even();
		sema_wait(&subrate_sema);
		printf(",sem_end=%lu", os_cputime_ticks_to_usecs(os_cputime_get32()));
	}
	xtimer_msleep(2000);
	for (int i = 0; i < 50; i++) {
		printf("\nmove-odd-%d, start_move=%lu", i, os_cputime_ticks_to_usecs(os_cputime_get32()));
		rt_ble_move_right_odd();
		sema_wait(&subrate_sema);
		printf(", sem_end=%lu", os_cputime_ticks_to_usecs(os_cputime_get32()));
	}
	printf("\nmove anchor test done\n");
	thread_flags_wait_all(FLAG_UP);
    return 0;
}

// rv -s 1 -p rtble/rtble_move_anchor