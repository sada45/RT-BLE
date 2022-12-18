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
#include "blex.h"

#include "nimble_l2cap_test_conf.h"


int main(void)
{
	uint16_t num = 6;
	uint16_t c_len = 247 * num-2;
	uint16_t p_len = 247 * num-2;
	uint32_t lat = 200;
	uint8_t more_resource = 1;
	int capacity = 0;
	blex_conn_t conn;
	int res;

	blex_init();

	while(1) {
		res = blex_add_new_conn(capacity+1, c_len, p_len, 200, &conn);
		if (res == -1) {
			printf("1capacity=%d\n", capacity);
		}
		print_timetable();
		res = blex_try_add(31, 245, 245, 50);
		if (res == -1) {
			break;
		}
		capacity++;
		xtimer_msleep(100);
	}
	printf("capacity = %d\n", capacity);
}
