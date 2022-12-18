/*
 * Copyright (C) 2022 Zhejiang University
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     module_blex
 * @{
 *
 * @file
 * @brief       blex implementation
 *
 * @author      Yeming Li <liymemnets@zju.edu.cn>
 *
 * @}
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <string.h>
#include "stdio_rtt.h"
#include "controller/ble_ll_conn.h"
#include "os/os_cputime.h"
#include "test_utils/expect.h"
#include "blex.h"

#define MAX_L2CAP_US        2088
#define MIN_LL_US           80
#define MAX_EXCH_US         4476
#define ONE_MAX_EXCH_US     2468
#define EMPTY_EXCH_US       460
#define SLOT_US             5004
#define START_UP_US			214

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))



int pow2[9] = {8, 16, 32, 64, 128, 256, 512, 1024, 2048};
uint32_t pow164[9] = {328, 656, 1312, 2624, 5248, 10496, 20992, 41984, 83968};
/* 3-bit for node type, 5-bit for connection handle */
uint8_t *tree[9] = { 0 };
uint16_t left_occupied_num[9] = { 0 };
uint16_t right_occupied_num[9] = { 0 };
uint16_t continue_cell_resource[512] = { 0 };
int8_t tree_level = -1;
uint32_t ticks_2000;
uint32_t ticks_1000;
uint8_t conn1_flag = 0;
uint8_t is_in[8] = { 0 };
//int small_counter = 0;

uint8_t current_mode = 0;

extern uint32_t base_tick;
extern uint8_t g_ble_ll_sched_offset_ticks;
uint32_t max_itvl_tick = 0;

timetable_item_t *time_table;

sema_t upd_sema;

static int blex_get_level(uint16_t itvl)
{
	for (int i = 0; i < 9; i++) {
		if (pow2[i] == itvl) {
			return i;
		}
	}
	return -1;
}

int blex_timetable_extend(uint16_t mitvl)
{
	timetable_item_t *ptr = time_table;
	timetable_item_t *last_ptr= time_table;
	timetable_item_t *new_ptr;

	if (mitvl * 41 <= max_itvl_tick){
		return 0;
	}
	if (!time_table) {
		goto copy_done;
	}
	puts("extend the timetable");
	while (last_ptr->next){
		last_ptr = last_ptr->next;
	}
	/* BLEX: Extend the time table */
	uint32_t end_ticks = mitvl * 41;
	while (1) {
		new_ptr = (timetable_item_t *)malloc(sizeof(timetable_item_t));
		memset(new_ptr, 0, sizeof(timetable_item_t));
		new_ptr->handle = ptr->handle;
		new_ptr->start_ticks = ptr->start_ticks + max_itvl_tick;
		new_ptr->end_ticks = ptr->end_ticks + max_itvl_tick;
		new_ptr->duration = ptr->duration;
		last_ptr->next = new_ptr;
		if (new_ptr->end_ticks >= end_ticks) {
			break;
		}
		last_ptr = new_ptr;
		ptr = ptr->next;
	}
	/* Merge the empty time table items with same handle */
	ptr = time_table;
	last_ptr = time_table->next;
	while(last_ptr){
		if (ptr->handle == 0 && last_ptr->handle == 0) {
			ptr->end_ticks = last_ptr->end_ticks;
			ptr->duration = last_ptr->end_ticks - ptr->start_ticks;
			ptr->next = last_ptr->next;
			free(last_ptr);
			last_ptr = ptr->next;
		}
		else {
			ptr = ptr->next;
			last_ptr = last_ptr->next;
		}
	}
copy_done:
	tree_level = blex_get_level(mitvl);
	max_itvl_tick = mitvl * 41;
	printf("tree_level = %d, max_itvl_tick = %lu\n", tree_level, max_itvl_tick);
	return 0;
}



int blex_add_new_conn(uint16_t conn_handle, uint16_t central_pktlen,
							uint16_t peripheral_pktlen, uint32_t latency,
							blex_conn_t *conn)
{
    uint16_t nc;
    uint16_t np;
    uint32_t tx_time;
    uint32_t rem_time;
    uint16_t conn_itvl;
    uint32_t origin_end;
    timetable_item_t *ptr;
    timetable_item_t *new_conn;
    timetable_item_t *next_empty;

    if (central_pktlen == 0) {
		nc = 0;
    }
    else {
		nc = (central_pktlen + 1) / 247 + 1;
    }
    if (peripheral_pktlen == 0) {
		np = 0;
    }
    else {
		np = (peripheral_pktlen + 1) / 247 + 1;
    }

    tx_time = START_UP_US + MIN(nc, np) * MAX_EXCH_US + (MAX(nc, np) - MIN(nc, np)) * ONE_MAX_EXCH_US;
	printf("nc = %d, np = %d, tx_time = %lu\n", nc, np, tx_time);
    tx_time = os_cputime_usecs_to_ticks(tx_time) + 1;
	// For establish the connection, we must ensure a minimal 5ms slot
	if (tx_time < 123) {
		tx_time = 123;
	}
    rem_time = os_cputime_usecs_to_ticks(latency * 1000) - tx_time;
    conn_itvl = 0;
    if ((uint32_t)(pow2[0] * 41) > rem_time) {
		return -1;
    }
	for (int i = 1; i < 9; i++) {
		if ((uint32_t)(pow2[i] * 41) > rem_time) {
			conn_itvl = pow2[i-1];
			break;
		}
	}
	if (conn_itvl == 0) {
		conn_itvl = (uint16_t)pow2[8];
	}
	printf("conn itvl = %d\n", conn_itvl);
    if (time_table == NULL) {
		blex_timetable_extend(conn_itvl);
		ptr = (timetable_item_t *)malloc(sizeof(timetable_item_t));
		memset(ptr, 0, sizeof(timetable_item_t));
		ptr->handle = conn_handle;
		ptr->start_ticks = 0;
		ptr->end_ticks = tx_time;
		ptr->duration = tx_time;
		next_empty = (timetable_item_t *)malloc(sizeof(timetable_item_t));
		memset(next_empty, 0, sizeof(timetable_item_t));
		next_empty->start_ticks = ptr->end_ticks;
		next_empty->end_ticks = max_itvl_tick;
		next_empty->handle = 0;
		next_empty->duration = next_empty->end_ticks - next_empty->start_ticks;
		ptr->next = next_empty;
		next_empty->next = NULL;
		conn->conn_itvl = conn_itvl;
		conn->offset = 0;
		time_table = ptr;
		return 0;
	}

    ptr = time_table->next;
    while (ptr && (ptr->start_ticks <= conn_itvl * 41)) {
		if (ptr->handle == 0 && ((tx_time + ticks_2000) < ptr->duration)) {
			origin_end = ptr->end_ticks;
			printf("origin_end = %lu\n", origin_end);
			new_conn = (timetable_item_t *)malloc(sizeof(timetable_item_t));
			memset(new_conn, 0, sizeof(timetable_item_t));
			if (ptr->start_ticks % 41 == 0) {
				new_conn->start_ticks = ptr->start_ticks + 41;
			}
			else {
				new_conn->start_ticks = 41 * ((ptr->start_ticks - 1) / 41 + 1);
			}
			new_conn->handle = conn_handle;
			new_conn->end_ticks = new_conn->start_ticks + tx_time;
			new_conn->duration = tx_time;
			ptr->end_ticks = new_conn->start_ticks;
			ptr->duration = ptr->end_ticks - ptr->start_ticks;

			if (new_conn->end_ticks < origin_end) {
				next_empty = (timetable_item_t *)malloc(sizeof(timetable_item_t));
				memset(next_empty, 0, sizeof(timetable_item_t));
				next_empty->handle = 0;
				next_empty->start_ticks = new_conn->end_ticks;
				next_empty->end_ticks = origin_end;
				next_empty->duration = next_empty->end_ticks - next_empty->start_ticks;
			}
			else {
				next_empty = NULL;
			}
			ptr->next = new_conn;
			new_conn->next = next_empty;
			conn->conn_itvl = conn_itvl;
			conn->offset = new_conn->start_ticks / 41;
			return 0;
		}
		ptr = ptr->next;
	}
	return -1;
}


int blex_try_add(uint16_t conn_handle, uint16_t central_pktlen,
							uint16_t peripheral_pktlen, uint32_t latency)
{
	(void)conn_handle;
	uint8_t can_insert_flag;
	uint8_t find_flag;
    uint16_t nc;
    uint16_t np;
    uint32_t tx_time;
    uint32_t rem_time;
    uint16_t conn_itvl;
    timetable_item_t *ptr;
	timetable_item_t *next_ce_ptr;
	uint32_t next_start_ticks;
	int num;

    if (central_pktlen == 0) {
		nc = 0;
    }
    else {
		nc = (central_pktlen + 1) / 247 + 1;
    }
    if (peripheral_pktlen == 0) {
		np = 0;
    }
    else {
		np = (peripheral_pktlen + 1) / 247 + 1;
    }

    tx_time = START_UP_US + MIN(nc, np) * MAX_EXCH_US + (MAX(nc, np) - MIN(nc, np)) * ONE_MAX_EXCH_US;
    tx_time = os_cputime_usecs_to_ticks(tx_time) + 1;
	// For establish the connection, we must ensure a minimal 5ms slot
	if (tx_time < 123) {
		tx_time = 123;
	}
    rem_time = os_cputime_usecs_to_ticks(latency * 1000) - tx_time;
    conn_itvl = 0;
    if ((uint32_t)(pow2[0] * 41) > rem_time) {
		return -1;
    }
	for (int i = 1; i < 9; i++) {
		if ((uint32_t)(pow2[i] * 41) > rem_time) {
			conn_itvl = pow2[i-1];
			break;
		}
	}
	if (conn_itvl == 0) {
		conn_itvl = (uint16_t)pow2[8];
	}
	printf("conn itvl = %d\n", conn_itvl);

    ptr = time_table->next;
	num = max_itvl_tick / (conn_itvl * 41);

    while (ptr && (ptr->start_ticks <= conn_itvl * 41)) {
		if (ptr->handle == 0 && ((tx_time + ticks_2000) < ptr->duration)) {
			can_insert_flag = 1;
			for (int i = 1; i < num; i++) {
				find_flag = 0;
				next_start_ticks = ptr->start_ticks + conn_itvl * 41 * i;
				next_ce_ptr = ptr;
				// Check wether can insert after
				while(next_ce_ptr && (next_ce_ptr->start_ticks <= next_start_ticks) && (next_ce_ptr->end_ticks > next_start_ticks)) {
					if (next_ce_ptr->handle == 0 && ((tx_time + ticks_2000) < (next_ce_ptr->end_ticks - next_start_ticks))) {
						find_flag = 1;
						break;
					}
					next_ce_ptr = next_ce_ptr->next;
				}

				if (find_flag == 0) {
					can_insert_flag = 0;
					break;
				}
			}
			if (can_insert_flag == 1) {
				return 0;
			}
		}
		ptr = ptr->next;
	}
	return -1;
}

void print_timetable_item(timetable_item_t *ptr)
{
	struct ble_ll_conn_sm *connsm;

	printf("handle = %d, start_time = %lu, end_time = %lu",
			ptr->handle, ptr->start_ticks, ptr->end_ticks);
	if (ptr->handle > 0) {
		connsm = ble_ll_conn_find_by_handle(ptr->handle);
		printf(",expect_ce_len = %f", connsm->expect_conn_event_len);
	}
	puts("");
}

void print_timetable(void)
{
	timetable_item_t *ptr = time_table;

	printf("++++++++++++++++++++++++++++\n");
	while (ptr) {
		print_timetable_item(ptr);
		ptr = ptr->next;
	}
}


int blex_update(void)
{
	uint16_t handles[8];
	int conn_num;
	uint32_t exp_ticks;
	int res;
	struct ble_ll_conn_sm *connsm;
	timetable_item_t *ptr;
	timetable_item_t *temp;
	timetable_item_t *next_temp;

	print_timetable();
	ptr = time_table;
	while(ptr) {
		conn_num = 0;
		memset(is_in, 0, 8);
		if (ptr->handle != 0) {
			connsm = ble_ll_conn_find_by_handle(ptr->handle);
			expect(connsm);
			exp_ticks = (uint32_t)(connsm->expect_conn_event_len) + 1;
			if (ptr->duration != exp_ticks) {
				if (exp_ticks > ptr->duration && ptr->next && ptr->next->handle == 0 &&
					((ptr->next->duration + ptr->duration) < exp_ticks)) {
					exp_ticks = ptr->next->duration + ptr->duration;
				}
				ptr->duration = exp_ticks;
				ptr->end_ticks = ptr->start_ticks + exp_ticks;
//				printf("handle = %d, new ptr->duration = %lu, start_ticks = %lu, end_ticks = %lu\n",
//					ptr->handle, ptr->duration, ptr->start_ticks, ptr->end_ticks);
				/* The next empty cell is always exists no matter its duration */
				if (ptr->next && ptr->next->handle == 0) {
					ptr->next->start_ticks = ptr->end_ticks;
					ptr->next->duration = ptr->next->end_ticks - ptr->next->start_ticks;
				}
			}
		}
		/* The idle cell is too small or too large */
		if (ptr->next && ptr->next->handle == 0 &&
			(ptr->next->duration < ticks_1000 || ptr->next->duration > ticks_2000)) {
			temp = ptr->next->next;

			while(temp) {
				if (temp->handle != 0 && is_in[temp->handle-1] == 0) {
					handles[conn_num] = temp->handle;
					is_in[temp->handle-1] = 1;
					conn_num++;
				}
				temp = temp->next;
			}
		}

		/* The shrink the cell */
		temp = ptr;

		if (ptr->handle > 0 && ptr->next && ptr->next->handle == 0 && ptr->next->duration > ticks_2000) {
			printf("shirk, ptr->handle = %d, next->handle = %d, next->start_time = %lu, next->end_time = %lu, next->duration = %lu\n",
					ptr->handle, ptr->next->handle, ptr->start_ticks, ptr->end_ticks, ptr->duration);
			for (int i = 0; i < conn_num; i++) {
				res = blex_move_anchor(ble_ll_conn_find_by_handle(handles[i]), -1);
				expect(res == 0);
				printf("shirk start wait\n");
				res = blex_wait_conn_upd_end();
				puts("shirk wait end");
				temp = time_table;
				next_temp = temp->next;
				while(next_temp) {
					if (temp->handle == 0 && next_temp->handle == handles[i]) {
						printf("before temp start_time = %lu, end_time = %lu, duration = %lu\n", temp->start_ticks,
								temp->end_ticks, temp->duration);
						temp->end_ticks -= 41;
						temp->duration -= 41;
						printf("after temp start_time = %lu, end_time = %lu, duration = %lu\n", temp->start_ticks,
								temp->end_ticks, temp->duration);
						next_temp->start_ticks -= 41;
						next_temp->end_ticks -= 41;
						if (next_temp->next) {
							if (next_temp->next->handle > 0) {
								expect(0);
							}
							next_temp->next->start_ticks -= 41;
							next_temp->next->duration += 41;
						}
						else {
							/* We append a new idle cell at the last of the timetable */
							timetable_item_t *t = (timetable_item_t *)malloc(sizeof(timetable_item_t));
							memset(t, 0, sizeof(timetable_item_t));
							t->handle = 0;
							t->start_ticks = next_temp->end_ticks;
							t->duration = 41;
							t->end_ticks = t->start_ticks + 41;
							next_temp->next = t;
						}
					}
					temp = temp->next;
					next_temp = next_temp->next;
				}
			}
		}
		if (ptr->handle > 0 && ptr->next && ptr->next->handle == 0 && ptr->next->duration < ticks_1000) {
			printf("extend the cell move %d connections\n", conn_num);
			for (int i = (conn_num-1); i >= 0; i--) {
				printf("move anchor handle = %d", handles[i]);
				res = blex_move_anchor(ble_ll_conn_find_by_handle(handles[i]), 1);
				expect(res == 0);
				puts("extend wait start");
				res = blex_wait_conn_upd_end();
				puts("upd wait end");
				temp = time_table;
				next_temp = temp->next;
				while(next_temp) {
					if (temp->handle == 0 && next_temp->handle == handles[i]) {
						next_temp->start_ticks += 41;
						next_temp->end_ticks += 41;
						expect(next_temp->end_ticks <= max_itvl_tick);
						temp->end_ticks += 41;
						temp->duration += 41;
						if (next_temp->next->handle == 0) {
							next_temp->next->start_ticks += 41;
							next_temp->next->duration -= 41;
						}
					}
					temp = temp->next;
					next_temp = next_temp->next;
				}
			}
		}
		ptr = ptr->next;
	}

	return 0;
}


void blex_move_right(void)
{
	int res;
	struct ble_ll_conn_sm *connsm = ble_ll_conn_find_by_handle(1);
	res = blex_move_anchor(connsm, 8);
	expect(res == 0);
	res = blex_wait_conn_upd_end();
}

void blex_move_left(void)
{
	int res;
	struct ble_ll_conn_sm *connsm = ble_ll_conn_find_by_handle(1);
	res = blex_move_anchor(connsm, -4);
	printf("res = %d\n", res);
	expect(res == 0);
	res = blex_wait_conn_upd_end();
}


void blex_connected(uint16_t conn_handle)
{
	struct ble_ll_conn_sm *connsm = ble_ll_conn_find_by_handle(conn_handle);
	if (!connsm) {
		printf("no conn\n");
		return;
	}
	if (!conn1_flag) {
		conn1_flag = 1;
		base_tick = connsm->anchor_point - g_ble_ll_sched_offset_ticks;
	}
	// print_timetable();
}

void blex_init(void)
{
	tree_level = -1;
	time_table = NULL;
	conn1_flag = 0;
	max_itvl_tick = 0;
	ticks_2000 = os_cputime_usecs_to_ticks(2250);
	ticks_1000 = os_cputime_usecs_to_ticks(1000);
}
