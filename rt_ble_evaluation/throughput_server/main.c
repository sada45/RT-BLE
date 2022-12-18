#include <stdio.h>
#include <stdlib.h>

#include "nimble_riot.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/util/util.h"


#include "test_utils/expect.h"
#include "net/bluetil/ad.h"
#include "controller/ble_ll_conn.h"
#include "controller/ble_ll.h"
#include "controller/ble_ll_sched.h"

#include "nimble_l2cap_test_conf.h"

#define REPORT_INTERVAL    1000
#define REPORT_SIZE        100
#define PROCESS_DELAY      1

#define FLAG_UP             (1u << 0)
#define FLAG_SYNC           (1u << 1)
#define FLAG_TX_UNSTALLED   (1u << 2)
#define FLAG_CONTINUE       (1u << 3)

#define CMD_TYPE_POS 0
#define CMD_INTERVAL 3


/* BLE connection state */
static uint16_t _handle = 0;
static struct ble_l2cap_chan *_coc = NULL;
static thread_t *_main;
static uint32_t report_interval = 1000;


/* buffer allocation */
static os_membuf_t _coc_mem[OS_MEMPOOL_SIZE(MBUFCNT, MBUFSIZE)];
static struct os_mempool _coc_mempool;
static struct os_mbuf_pool _coc_mbuf_pool;
static uint8_t _rxbuf[10000];
static char thread_buf[2048];
static kernel_pid_t report_pid = -1;
static thread_t *report_thread = NULL;


/* safe AD fields */
static uint8_t _ad_buf[BLE_HS_ADV_MAX_SZ];
static bluetil_ad_t _ad;
static struct ble_gap_adv_params _adv_params = {.itvl_min = BLE_GAP_ADV_ITVL_MS(100),
												.itvl_max = BLE_GAP_ADV_ITVL_MS(100)};
static uint8_t pause = 1;

static uint8_t counter = 0;

static void _advertise_now(void);

static void ble_send(uint8_t *data, uint16_t length){
    int res = 0;
    struct os_mbuf *txd;
	txd = os_mbuf_get_pkthdr(&_coc_mbuf_pool, 0);
	expect(txd != NULL);
	res = os_mbuf_append(txd, data, length);
	expect(res == 0);
	do {
    	if (_coc == NULL){
			printf("no such connection \n");
			return;
    	}
        res = ble_l2cap_send(_coc, txd);
        if (res == BLE_HS_EBUSY) {
            thread_flags_wait_all(FLAG_TX_UNSTALLED);
        }
    } while (res == BLE_HS_EBUSY);
    if ((res != 0) && (res != BLE_HS_ESTALLED)) {
        printf("# err: failed to send (%i)\n", res);
        expect(0);
	}
	return;

}

void *report_thread_func(void *args){
	(void) args;
	uint32_t radio_time;
	uint32_t rx_time;
	uint32_t tx_time;
	while (1){
		if (pause){
			puts("paused");
			thread_flags_wait_all(FLAG_CONTINUE);
			xtimer_msleep(5000);
		}
		xtimer_msleep(REPORT_INTERVAL);
		memset(_rxbuf, counter, REPORT_SIZE);
		ble_phy_get_radio_time(&radio_time, &rx_time, &tx_time);
		printf("\n%d-app=%ld ", counter, os_cputime_ticks_to_usecs(os_cputime_get32()));
		printf(",tx=%ld,rx=%ld", os_cputime_ticks_to_usecs(tx_time),
					os_cputime_ticks_to_usecs(rx_time));
		xtimer_msleep(PROCESS_DELAY);
		ble_send(_rxbuf, REPORT_SIZE);
//		if (counter == 255) {
//			break;
//		}
		counter++;
	}
	return NULL;
}


static void _on_data(struct ble_l2cap_event *event)
{
    int res;
    struct os_mbuf *rxd;

    rxd = event->receive.sdu_rx;
    expect(rxd != NULL);
    int rx_len = (int)OS_MBUF_PKTLEN(rxd);
    expect(rx_len <= (int)APP_MTU);
    res = os_mbuf_copydata(rxd, 0, rx_len, _rxbuf);
    report_interval = *(_rxbuf + CMD_INTERVAL);

    /* allocate new mbuf for receiving new data */
    os_mbuf_free_chain(rxd);
    rxd = os_mbuf_get_pkthdr(&_coc_mbuf_pool, 0);
    expect(rxd != NULL);
    res = ble_l2cap_recv_ready(_coc, rxd);
    expect(res == 0);
}

static int _on_gap_evt(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    printf("# GAP event %i\n", (int)event->type);
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            _handle = event->connect.conn_handle;
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            _coc = NULL;
            _handle = 0;
            _advertise_now();
            break;
        case BLE_GAP_EVENT_MTU:
        	break;
        case BLE_GAP_EVENT_CONN_UPDATE:
        	ble_phy_clear_radio_time();
        	printf("CONN updated\n");
//        	pause = 0;
//        	thread_flags_set(_main, FLAG_UP);
//        	thread_flags_set(report_thread, FLAG_CONTINUE);

        case BLE_GAP_EVENT_CONN_UPDATE_REQ:
        default:
            break;
    }
    return 0;
}

static int _on_l2cap_evt(struct ble_l2cap_event *event, void *arg)
{
    (void)arg;
		printf("l2cap event type = %d\n", event->type);
    switch (event->type) {
        case BLE_L2CAP_EVENT_COC_CONNECTED: {
            _coc = event->connect.chan;
            struct ble_l2cap_chan_info info;
            ble_l2cap_get_chan_info(_coc, &info);
            printf("new connection dcid = %d\n", info.dcid);
            puts("# L2CAP: CONNECTED");
            printf("# MTUs: our %i, remote %i\n",
                   (int)info.our_l2cap_mtu, (int)info.peer_l2cap_mtu);
        	pause = 0;
        	thread_flags_set(_main, FLAG_UP);
        	thread_flags_set(report_thread, FLAG_CONTINUE);
//        	ble_ll_conn_start_pkt_loss();
            break;
        }
        case BLE_L2CAP_EVENT_COC_DISCONNECTED:
            _coc = NULL;
            puts("# L2CAP: DISCONNECTED");
            pause = 1;
            break;
        case BLE_L2CAP_EVENT_COC_ACCEPT: {
            struct os_mbuf *sdu_rx = os_mbuf_get_pkthdr(&_coc_mbuf_pool, 0);
            expect(sdu_rx != NULL);
            ble_l2cap_recv_ready(event->accept.chan, sdu_rx);
            break;
        }
        case BLE_L2CAP_EVENT_COC_DATA_RECEIVED:
            _on_data(event);
            break;
        case BLE_L2CAP_EVENT_COC_TX_UNSTALLED:
            /* this event is expected, but we have nothing to do here */
            if (report_thread != NULL){
				thread_flags_set(report_thread, FLAG_TX_UNSTALLED);
            }
            break;
        default:
            expect(0);
            break;
    }

    return 0;
}

static void _advertise_now(void)
{
    int res = ble_gap_adv_start(nimble_riot_own_addr_type, NULL, BLE_HS_FOREVER,
                                &_adv_params, _on_gap_evt, NULL);
   	printf("res = %d\n", res);
    expect(res == 0);
}

int main(void)
{
	uint8_t last_flag;
    int res;
    uint32_t real_tx_time;
    uint32_t radio_time;
    uint32_t rx_time;
    uint32_t tx_time;

    (void)res;
    puts("NimBLE L2CAP test server");
	_main = thread_get_active();
    /* initialize buffers and setup the test environment */
    res = os_mempool_init(&_coc_mempool, MBUFCNT, MBUFSIZE, _coc_mem, "appbuf");
    expect(res == 0);
    res = os_mbuf_pool_init(&_coc_mbuf_pool, &_coc_mempool, MBUFSIZE, MBUFCNT);
    expect(res == 0);
    uint8_t *my_addr = ble_ll_get_our_devaddr(1);
    for (int i = 0; i < 6; i++){
		printf("0x%x,", my_addr[i]);
    }
    printf("\n");

    /* create l2cap server */
    res = ble_l2cap_create_server(0x0235, APP_MTU, _on_l2cap_evt, NULL);
    expect(res == 0);

    /* initialize advertising data and parameters */
    _adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    _adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    bluetil_ad_init_with_flags(&_ad, _ad_buf, sizeof(_ad_buf),
                               BLUETIL_AD_FLAGS_DEFAULT);
    bluetil_ad_add_name(&_ad, APP_NODENAME);
    res = ble_gap_adv_set_data(_ad.buf, (int)_ad.pos);
    expect(res == 0);
    /* start advertising the test server */
    _advertise_now();
   	report_pid = thread_create(thread_buf, sizeof(thread_buf), THREAD_PRIORITY_MAIN-1,
								THREAD_CREATE_STACKTEST, report_thread_func, NULL, "periodic report");
	report_thread = thread_get(report_pid);
    puts("# now advertising");
    thread_flags_wait_all(FLAG_UP);

	ble_phy_clear_radio_time();  // Remove the inference of connection establish
    printf("*************radio counter start time = %ld**************\n", os_cputime_ticks_to_usecs(os_cputime_get32()));
    while (1){
		last_flag = ble_ll_get_real_send_time(&real_tx_time);
		if (last_flag & 0x4) {
			printf(",manualloss");
		}
		if (last_flag & 0x2) {
			printf(",delaytxend=%ld", os_cputime_ticks_to_usecs(real_tx_time));
		}
		else {
			printf(",txend=%ld", os_cputime_ticks_to_usecs(real_tx_time));
		}
		if (last_flag & 0x1) {
			ble_phy_get_radio_time(&radio_time, &rx_time, &tx_time);
			printf(",tx=%ld,rx=%ld", os_cputime_ticks_to_usecs(tx_time),
					os_cputime_ticks_to_usecs(rx_time));
		}
    }
    return 0;
}
