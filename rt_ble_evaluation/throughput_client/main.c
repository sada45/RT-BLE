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


#include "nimble_l2cap_test_conf.h"

#define FLAG_UP             (1u << 0)
#define FLAG_UPD_START      (1u << 1)
//#define FLAG_TX_UNSTALLED   (1u << 2)
//#define FLAG_RECV           (1u << 3)
//#define FLAG_BREAK          (1u << 4)

#define UPD_FORHEAD         600

/* synchronization state */
static thread_t *_main;
static thread_t *collect_thread;

/* buffer allocation */
static os_membuf_t _coc_mem[OS_MEMPOOL_SIZE(MBUFCNT, MBUFSIZE)];
static struct os_mempool _coc_mempool;
static struct os_mbuf_pool _coc_mbuf_pool;
static uint8_t connection_counter = 0;

struct pure_address{
	uint8_t val1;
	uint8_t val2;
	uint8_t val3;
	uint8_t val4;
	uint8_t val5;
	uint8_t val6;
};
uint8_t addr_type;


static struct pure_address device_address[MYNEWT_VAL_BLE_L2CAP_COC_MAX_NUM] ={
																				{0xb7,0x5d,0x7a,0x2b,0xd8,0xe1},
																				{0xfd,0xa7,0x01,0xbd,0xb8,0xc9},
//																				{0x4a,0x5d,0xc5,0x9c,0xd0,0xeb},
//																				{0x13,0x56,0x5f,0xf1,0x99,0xe4},
//																				{0x32,0x66,0xab,0xa7,0x68,0xf9},
//																				{0x2c,0xc9,0xbc,0x23,0x05,0xfe},
//																				{0x61,0x83,0x6e,0x8c,0x30,0xd4},
//																				{0x02,0x6c,0x49,0xb8,0x99,0xcf}
																				};

static uint16_t handles[MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM)] = { 0 };
static uint32_t anchors[MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM)] = { 0 };
static uint8_t anchor_usec[MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM)] = { 0 };
static uint8_t upd_flag[MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM)] = { 0 };
static struct ble_l2cap_chan *cocs[MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM)] = { 0 };
static sem_t s;


//static int oneshot_index[MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM)] = {};
static uint8_t device_type[MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM)] = {0};
static uint32_t req_send_time[MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM)] = { 0 };
static uint32_t oneshot_delay[MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM)] = { 0 };
static uint8_t oneshot_num = 0;
static uint8_t oneshot_recv_num = 0;
static thread_t *txend_thread = NULL;
static char thread_buf[2048];
static uint8_t ciupd_flag = 0;
static int txend_counter = 0;

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
/* The updated connection interval */
static uint16_t device_ci[MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM)] = {64, 64};
static uint8_t anchor_offset[MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM)] = {0, 5};

//static uint16_t device_ci[MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM)] = {0};
////
//static uint8_t anchor_offset[MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM)] = {0};


//static int ble_send(struct ble_l2cap_chan *chan, uint8_t *data, int len);

static void update_connection_counter(void){
	connection_counter = 0;
	for (int i = 0; i < MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM); i++){
		if (handles[i] != 0){
			connection_counter++;
		}
	}
}

static void _on_data(struct ble_l2cap_event *event)
{
    int res;
    (void)res;
    struct os_mbuf *rxd = event->receive.sdu_rx;
    expect(rxd != NULL);
	uint32_t recv_time = os_cputime_ticks_to_usecs(os_cputime_get32());
//	printf("recv at %ld\n, len=%d", recv_time, OS_MBUF_PKTLEN(rxd));
    for (int i = 0; i < MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM); i++){
		if (event->receive.conn_handle == handles[i] && device_type[i] == 1){
			oneshot_recv_num++;
			oneshot_delay[i] = recv_time - req_send_time[i];
			if (oneshot_recv_num == oneshot_num){
//				thread_flags_set(collect_thread, FLAG_RECV);
			}
			break;
		}
    }
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
            update_connection_counter();
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
            update_connection_counter();
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
    printf("# GAP event: %i\n", (int)event->type);
    uint16_t gap_conn_handle = event->conn_update.conn_handle;
	ble_addr_t *addr;
	int res;
	int i;

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT: {
            addr = (ble_addr_t *)arg;
            for (int i = 0; i < MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM); i++){
				if (memcmp(&addr->val, &device_address[i], 6) == 0 && handles[i] == 0){
					handles[i] = event->connect.conn_handle;
					break;
				}
            }
            update_connection_counter();
			free(addr);
            struct os_mbuf *sdu_rx = os_mbuf_get_pkthdr(&_coc_mbuf_pool, 0);
            expect(sdu_rx != NULL);
            ble_l2cap_connect(event->connect.conn_handle, APP_CID, APP_MTU, sdu_rx,
                                       _on_l2cap_evt, NULL);

            break;
        }
        case BLE_GAP_EVENT_DISCONNECT:
        	puts("====================================");
           	for (int i = 0; i < MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM); i++) {
				if (memcmp(&device_address[i], &(event->disconnect.conn.peer_id_addr.val), 6) == 0){
					printf("conn_handle = %d disconnect\n", handles[i]);
					handles[i] = 0;
					break;
				}
           	}
           	update_connection_counter();
            break;
        case BLE_GAP_EVENT_CONN_UPDATE:
        	sem_post(&s);
        	break;
        default:
            break;
    }

    return 0;
}

//static int ble_send(struct ble_l2cap_chan *chan, uint8_t *data, int len){
//    int res = 0;
//    struct os_mbuf *txd;
//    if (chan == NULL){
//		printf("no such connection \n");
//		return 0;
//    }
//	txd = os_mbuf_get_pkthdr(&_coc_mbuf_pool, 0);
//	expect(txd != NULL);
//	res = os_mbuf_append(txd, data, len);
//	expect(res == 0);
//	do {
//        res = ble_l2cap_send(chan, txd);
//        if (res == BLE_HS_EBUSY) {
//            thread_flags_wait_all(FLAG_TX_UNSTALLED);
//        }
//    } while (res == BLE_HS_EBUSY);
//    if ((res != 0) && (res != BLE_HS_ESTALLED)) {
//        printf("# err: failed to send (%i)\n", res);
//        expect(0);
//	}
//	return len;
//}
//

static int _cmd_ciupdate(int argc, char **argv){
	(void)argc;
	(void)argv;
	int res;
	int32_t time_diff;
	uint32_t time_diff_usec;
	uint32_t now_ticks;
	uint32_t connection_interval_ticks = os_cputime_usecs_to_ticks(100000);
	struct ble_gap_upd_params params = {6, 50, BLE_GAP_INITIAL_CONN_LATENCY, 0x0100,
										BLE_GAP_INITIAL_CONN_MIN_CE_LEN, BLE_GAP_INITIAL_CONN_MAX_CE_LEN};
	params.itvl_min = 6;
	params.itvl_max = 50;
	params.supervision_timeout = 0x0100;

	ciupd_flag = 1;
	thread_flags_set(txend_thread, FLAG_UPD_START);
	ble_ll_conn_list_anchor_points(anchors, anchor_usec);
	now_ticks = os_cputime_get32();
	time_diff = anchors[0] - now_ticks;
	if (time_diff < UPD_FORHEAD) {
		time_diff += connection_interval_ticks;
		printf("no enought time difference, move to next connection event\n");
	}
	time_diff -= UPD_FORHEAD;
	printf("Timer start\n");
	time_diff_usec = os_cputime_ticks_to_usecs((uint32_t)time_diff);
	xtimer_usleep(time_diff_usec);
	ble_ll_conn_list_anchor_points(anchors, anchor_usec);
//	printf("2->now = %ld, a1 = %ld, a2 = %ld, a3 = %ld, a4 = %ld, a5=%ld, a6=%ld, a7=%ld, a8=%ld\n", os_cputime_get32(), anchors[0], anchors[1], anchors[2], anchors[3], anchors[4], anchors[5], anchors[6], anchors[7]);
	ble_ll_ciupd_retransmission_reset();
	txend_counter = 0;
	for (int i = 0; i < MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM); i++){
		// nimble only use the max interval
		params.itvl_min = device_ci[i];
		params.itvl_max = device_ci[i];
		res = ble_gap_update_params(i+1, &params);
		printf("gap update res = %d\n", res);
		expect(res == 0);
	}
	return 0;
}



static int _cmd_listanchor(int argc, char **argv){
	(void) argc;
	(void) argv;
	ble_ll_conn_list_anchor_points(anchors, anchor_usec);
	uint8_t state = ble_ll_state_get();
	printf("state = %d, %ld", state, os_cputime_ticks_to_usecs(os_cputime_get32()));
	printf("handles->");
	for (int i = 0; i < MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM); i++) {
		printf("%d ", handles[i]);
	}
	printf("\n//AP ");  // Show the start of the anchor points output
	for (int i = 0; i < MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM); i++){
		if (handles[i] == 0){  // The connecton is not established
			printf("0");
		}
		else{
			printf("%ld,", os_cputime_ticks_to_usecs(anchors[handles[i]-1]) + anchor_usec[handles[i] - 1]);
		}
	}
	uint32_t a1 = os_cputime_ticks_to_usecs(anchors[1]) + anchor_usec[1];
	uint32_t a2 = os_cputime_ticks_to_usecs(anchors[0]) + anchor_usec[0];
	printf("diff = %d\n", a1-a2);
	printf("//end\n");
	return 0;
}


static int _cmd_runtime(int argc, char **argv){
	(void) argc;
	(void) argv;
	uint32_t runtime;
	uint32_t numofruns;
	get_avg_runtime(&runtime, &numofruns);
	printf("run time = %ld nums = %ld avg = %ld\n", runtime, numofruns, runtime / numofruns);
	return 0;
}

static int _cmd_radiotime(int argc, char **argv){
	(void) argc;
	(void) argv;
	uint32_t radio_time;
	uint32_t tx_time;
	uint32_t rx_time;
	ble_phy_get_radio_time(&radio_time, &rx_time, &tx_time);
	printf("tx_time = %ld rx_time = %ld\n", tx_time, rx_time);
	return 0;
}

static int _cmd_clear_radiotime(int argc, char **argv){
	(void) argc;
	(void) argv;
	ble_phy_clear_radio_time();
	return 0;
}

//static int _cmd_setpriority(int argc, char **argv){
//	(void) argc;
//	(void) argv;
//	for (int i = 0; i < MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM); i++){
//		ble_ll_conn_set_priority(handles[i], device_priority[i]);
//	}
//	return 0;
//}

static int _cmd_get_overlap_times(int argc, char **argv) {
	(void)argc;
	(void)argv;
	printf("%ld, %ld %ld\n", os_cputime_ticks_to_usecs(os_cputime_get32()),
		ble_ll_sched_get_overlap_times(), ble_ll_conn_get_overlaptimes());
	return 0;
}

static int _cmd_get_overlap_data(int argc, char **argv) {
	(void)argc;
	(void)argv;
	ble_ll_sched_get_overlap_data();
	return 0;
}

#if USE_BLEX
static int _cmd_move_anchor(int argc, char **argv) {
	(void)argc;
	(void)argv;
	_cmd_listanchor(0, NULL);
	ble_ll_ctrl_move_anchor_point(2, 39);
	sem_wait(&s);
	_cmd_listanchor(0, NULL);
}
#endif


static const shell_command_t _cmds[] = {
//	{ "flood", "", _cmd_floodtest},
    { "ciupdate", "", _cmd_ciupdate },
    { "ls", "", _cmd_listanchor},
    { "rt", "", _cmd_runtime},
    { "radiotime", "", _cmd_radiotime},
    { "clrradio", "", _cmd_clear_radiotime},
//    { "sp", "", _cmd_setpriority},
    { "ot", "", _cmd_get_overlap_times},
    { "od", "", _cmd_get_overlap_data},
#if USE_BLEX
    { "mv", "", _cmd_move_anchor},
#endif
    { NULL, NULL, NULL }
};

static int ciupdate(void){
	int res;
	int32_t time_diff;
	uint32_t time_diff_usec;
	uint32_t now_ticks;
	uint32_t connection_interval_ticks = os_cputime_usecs_to_ticks(100000);
	struct ble_gap_upd_params params = {6, 50, BLE_GAP_INITIAL_CONN_LATENCY, 0x0100,
										BLE_GAP_INITIAL_CONN_MIN_CE_LEN, BLE_GAP_INITIAL_CONN_MAX_CE_LEN};
	params.itvl_min = 6;
	params.itvl_max = 50;
	params.supervision_timeout = 0x0100;

	ciupd_flag = 1;
	thread_flags_set(txend_thread, FLAG_UPD_START);
	ble_ll_conn_list_anchor_points(anchors, anchor_usec);
	now_ticks = os_cputime_get32();
	time_diff = anchors[0] - now_ticks;
	if (time_diff < UPD_FORHEAD) {
		time_diff += connection_interval_ticks;
		printf("no enought time difference, move to next connection event\n");
	}
	time_diff -= UPD_FORHEAD;
	printf("Timer start\n");
	time_diff_usec = os_cputime_ticks_to_usecs((uint32_t)time_diff);
	xtimer_usleep(time_diff_usec);
	ble_ll_conn_list_anchor_points(anchors, anchor_usec);
//	printf("2->now = %ld, a1 = %ld, a2 = %ld, a3 = %ld, a4 = %ld, a5=%ld, a6=%ld, a7=%ld, a8=%ld\n", os_cputime_get32(), anchors[0], anchors[1], anchors[2], anchors[3], anchors[4], anchors[5], anchors[6], anchors[7]);
	ble_ll_ciupd_retransmission_reset();
	txend_counter = 0;
	for (int i = 0; i < MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM); i++){
		// nimble only use the max interval
		params.itvl_min = device_ci[i];
		params.itvl_max = device_ci[i];
		res = ble_gap_update_params(i+1, &params);
		printf("gap update res = %d\n", res);
		expect(res == 0);
	}
	return 0;
}


void *txend_thread_func(void *args){
	(void) args;
//	uint8_t last_flag;
//	uint32_t real_tx_time;

	ble_ll_sema_consume();
	while (1){
//		if (!ciupd_flag) {
//			thread_flags_wait_all(FLAG_UPD_START);
//		}
//		last_flag = ble_ll_get_real_send_time(&real_tx_time);
//		txend_counter++;
//		printf("%d ", txend_counter);
//		if (last_flag & 0x2) {
//			printf("delaytxend=%ld\n", real_tx_time);
//		}
//		else {
//			printf("txend=%ld\n", real_tx_time);
//		}
		xtimer_msleep(10000);
		printf("ot=%ld,%ld,%ld\n", os_cputime_ticks_to_usecs(os_cputime_get32()),
				ble_ll_sched_get_overlap_times(), ble_ll_conn_get_overlaptimes());
	}
}


int main(void)
{
    int res;
	uint32_t radio_time;
	uint32_t rx_time;
	uint32_t tx_time;

    /* save context of the main */
    _main = thread_get_active();
    /* initialize buffers and setup the test environment */
    res = os_mempool_init(&_coc_mempool, MBUFCNT, MBUFSIZE, _coc_mem, "appbuf");
    expect(res == 0);
    res = os_mbuf_pool_init(&_coc_mbuf_pool, &_coc_mempool, MBUFSIZE, MBUFCNT);
    expect(res == 0);
//    printf("165=%ld, 164=%ld", os_cputime_ticks_to_usecs(165), os_cputime_ticks_to_usecs(164));
	/* Start to connect */
	ble_ll_sched_set_anchor_offset(anchor_offset);
	for (int i = 0; i < MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM); i ++) {
		printf("offset=%d", anchor_offset[i]);
	}
	puts("");
#ifdef FIX_ANCHOR
	puts("FIX ANCHOR");
#endif
	ble_addr_t *target_addr = NULL;
	ble_ll_sched_set_anchor_offset(anchor_offset);

//	ble_ll_sched_timetable_extend(16);
//	ble_ll_sched_cal_conn_estab_winoffset(8);

//	for (int i = 0; i < MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM); i++) {
	for (int i = 0; i < 1; i++) {
		target_addr = (ble_addr_t *)malloc(sizeof(ble_addr_t));
		target_addr->type = BLE_ADDR_RANDOM;
		memcpy(&target_addr->val, &device_address[i], 6);
		do{
			printf("connect to device %d\n", i+1);
			res = ble_gap_connect(nimble_riot_own_addr_type, target_addr, BLE_HS_FOREVER,
									&gap_conn_params, _on_gap_evt, target_addr);
			expect(res == 0);
			thread_flags_wait_all(FLAG_UP);
			xtimer_msleep(500);
		}while(connection_counter < (i+1));
	}
	printf("All devices connected\n");
//	kernel_pid_t txendpid = thread_create(thread_buf, sizeof(thread_buf), THREAD_PRIORITY_MAIN-1,
//								THREAD_CREATE_STACKTEST, txend_thread_func, NULL, "periodic report");
//	txend_thread = thread_get(txendpid);
	sem_init(&s, 0, 0);
//	ciupdate();
//	ble_ll_conn_start_pkt_loss();
    char line_buf[512];
    shell_run(_cmds, line_buf, SHELL_DEFAULT_BUFSIZE);
//	while (true) {
//		xtimer_msleep(1000);
//		ble_phy_get_radio_time(&radio_time, &rx_time, &tx_time);
//		printf("app%ld,tx=%ld,rx=%ld\n", os_cputime_ticks_to_usecs(os_cputime_get32()), os_cputime_ticks_to_usecs(tx_time),
//					os_cputime_ticks_to_usecs(rx_time));
//	}

    return 0;
}
