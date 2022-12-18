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

// int main(void)
// {
// 	uint16_t num = 6;
// 	uint16_t c_len = 247 * num-2;
// 	uint16_t p_len = 247 * num-2;
// 	uint32_t lat = 200;
// 	uint8_t more_resource = 1;
// 	int capacity = 0;
// 	rt_ble_conn_t *conn = NULL;
// 	rt_ble_conn_t *next_conn;

// 	rt_ble_tree_init();
// 	uint32_t time[100];
// 	for (int i = 0; i < 8; i++){
// 		for (int j = 0; j < 100; j++) {
// 			if (conn != NULL) {
// 				free(conn);
// 			}
// 			time[j] = xtimer_now_usec();
// 			conn = rt_ble_allocate(0, 100, 50, 90, 1, 1);
// 			time[j] = ximter_now_usec() - time[j];
// 		}
// 		rt_ble_occupy(conn->level, conn->offset, conn->slot_num, 1);
// 		for (int j = 0; j < 100; j++) {
// 			printf(",%lu", time[j]);
// 			if (j % 10 == 9) {
// 				xtimer_msleep(10);
// 			}
// 		}
// 	}
// 	// while (1) {
// 	// 	conn = rt_ble_allocate(c_len, p_len, lat, 90, capacity, 1);
// 	// 	if (conn) {
// 	// 		rt_ble_occupy(conn->level, conn->offset, conn->slot_num, capacity);
// 	// 		// print_tree();
// 	// 		next_conn = rt_ble_allocate(245, 245, 50, 90, 31, 1);
// 	// 		if (next_conn) {
// 	// 			capacity++;
// 	// 		}
// 	// 		else {
// 	// 			printf("capacity = %d\n", capacity);
// 	// 			break;
// 	// 		}
// 	// 	}
// 	// 	else {
// 	// 		break;
// 	// 	}
// 	// 	xtimer_msleep(200);
// 	// }
// 	thread_flags_wait_all(0x80);
//     return 0;
// }

int main(void)
{
	rt_ble_conn_t *conn = NULL;

	rt_ble_tree_init();
	uint32_t time[100];
	uint32_t lat = 3000;
	for (int i = 0; i < 512; i++) {
		for (int j = 0; j < 100; j++) {
			time[j] = xtimer_now_usec();
			for (int k = 0; k < 10; k++) {
				conn = rt_ble_allocate(0, 100, lat, 90, 1, 1);
				if (conn == NULL) {
					puts("ERROR");
					return 0;
				}
				free(conn);
			}	
			time[j] = xtimer_now_usec() - time[j];
		}
		conn = rt_ble_allocate(0, 100, lat, 90, 1, 1);
		rt_ble_occupy(conn->level, conn->offset, conn->slot_num, 1);
		free(conn);
		conn = NULL;
		for (int j = 0; j < 100; j++) {
			printf(",%lu", time[j]);
		}
	}
	printf("\n\ndone\n");
    return 0;
}
