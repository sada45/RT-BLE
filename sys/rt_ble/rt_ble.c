/*
 * Copyright (C) 2022 Zhejiang University
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     module_rt_ble
 * @{
 *
 * @file
 * @brief       rt_ble implementation
 *
 * @author      Yeming Li <liymemnets@zju.edu.cn>
 *
 * @}
 */


/* Implementation of the module */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <string.h>
#include "stdio_rtt.h"
#include "rt_ble.h"
#include "controller/ble_ll_conn.h"
#include "os/os_cputime.h"
#include "test_utils/expect.h"



int pow2[9] = {2, 4, 8, 16, 32, 64, 128, 256, 512};
uint32_t pow164[9] = {328, 656, 1312, 2624, 5248, 10496, 20992, 41984, 83968};
/* 3-bit for node type, 5-bit for connection handle */
uint8_t *tree[9] = { 0 };
uint16_t left_occupied_num[9] = { 0 };
uint16_t right_occupied_num[9] = { 0 };
uint16_t continue_cell_resource[512] = { 0 };
int8_t tree_level = -1;
uint8_t conn1_flag = 0;

rt_ble_conn_t *conn_list[MYNEWT_VAL_BLE_L2CAP_COC_MAX_NUM] = { 0 };
rt_ble_conn_t *critical_conn_list[MYNEWT_VAL_BLE_L2CAP_COC_MAX_NUM] = { 0 };
rt_ble_conn_t *pending_conn = NULL;
uint8_t current_mode = 0;

extern uint32_t base_tick;
extern uint8_t g_ble_ll_sched_offset_ticks;

/* RT-BLE: reverse the bits, so we can get the offset with index or change index to offset*/
static
uint16_t reverse_bit(uint16_t n, int len)
{
    n = ((n & 0xff00) >> 8) | ((n & 0x00ff) << 8);
    n = ((n & 0xf0f0) >> 4) | ((n & 0x0f0f) << 4);
    n = ((n & 0xcccc) >> 2) | ((n & 0x3333) << 2);
    n = ((n & 0xaaaa) >> 1) | ((n & 0x5555) << 1);
    return n >> (16 - len);
}

void print_tree(void) {
    printf("print_tree===========%d\n", tree_level);
    for (int i = 0; i <= tree_level; i++) {
    	printf("|");
        for (int j = 0; j < pow2[i]; j++) {
            printf("%d %d, %d|", reverse_bit(j, i+1), tree[i][j] >> 5, tree[i][j] & 0x1f);
            if (j == (pow2[i] / 2)-1) {
                printf("--|");
            }
        }
        printf("\n");
    }
    printf("-----------------\n|");
    for (int i = 0; i < pow2[tree_level]; i++) {
        printf("%d|", continue_cell_resource[i]);
    }
    printf("\n");
}


static void sci_check(double *a, int *e)
{
    while (*a >= 10.0) {
        *a = *a / 10.0;
        *e = *e + 1;
    }
}

static int sci_add(double *a, int *a_e, double *b, int *b_e)
{
    if (*a > 10.0 || *b > 10.0) {
        printf("sci add error\n");
        return -1;
    }
    if (*a_e < *b_e) {
        *b = pow(10.0, (*b_e-*a_e)) * (*b);
        *b_e = *a_e;
    }
    else if (*a_e > *b_e) {
        *a = pow(10.0, (*a_e - *b_e)) * (*a);
        *a_e = *b_e;
    }
    *a = *a + *b;
    sci_check(a, a_e);
    sci_check(b, b_e);
    return 0;
}

static int sci_lt(double a, int a_e, double b, int b_e) 
{
    if (b_e > a_e) {
        return 1;
    }
    else if (b_e == a_e) {
        return a < b? 1: 0;
    }
    else {
        return 0;
    }
}

static uint16_t rt_ble_retrans_num(int nm, uint32_t pkt_loss_rate, uint32_t wcet_prob) {
    uint32_t success_rate = 100 - pkt_loss_rate;
    int sum_s = 0;
    double sum = 1.0;
    int add_s = 0;
    double add;
    uint16_t nre = 0;
    double percentile;
    int percentile_s;
    if (wcet_prob >= 100) {
        percentile = 1;
        percentile_s = 0;
    }
    else {
        percentile = wcet_prob / 10.0;
        percentile_s = -1;
    }

    for (int i = 0; i < nm; i++) {
        sum *= success_rate;
        sum_s -= 2;  // divided by 100
        sci_check(&sum, &sum_s);
    }
    add_s = sum_s;
    add = sum;
    while (sci_lt(sum, sum_s, percentile, percentile_s)) {
        nre++;
        add *= (nm + nre - 1) * pkt_loss_rate / nre;
        add_s  -= 2;
        sci_check(&add, &add_s);
        sci_add(&sum, &sum_s, &add, &add_s);
        
    }
    // printf("nre = %d, sum=%f, sum_e = %d, percetile=%f, percentile_s = %d\n", nre, sum, sum_s, percentile, percentile_s);
    return nre;
}

void rt_ble_tree_init(void)
{
    int num =2;

    for(int i = 0; i < 9; i++) {
        tree[i] = (uint8_t *)malloc(num);
        memset(tree[i], 0, num);
        num = num << 1;
    }
}


int rt_ble_tree_extend(int8_t level)
{
    if (level <= tree_level) {
        return 0;
    }
    else if (level > 9) {
        return -EINVAL;
    }
    /* The tree is not initilized */
    if (tree_level == -1 || continue_cell_resource[0] > 0) {
        for (int i = 0; i < pow2[level]; i++) {
            continue_cell_resource[i] = pow2[level] - i;
        }
    }
    else {
        for (int i = tree_level; i < level; i++) {
            memcpy(continue_cell_resource + pow2[i], continue_cell_resource, 2 * pow2[i]);
        }
    }
    tree_level = level;
    return 0;
}

int rt_ble_tree_shrink(int8_t level)
{
    if (level > tree_level) {
        return -EINVAL;
    }
    tree_level = level;
    return 0;
}

rt_ble_conn_t* rt_ble_find_resource(int8_t origin_level, int origin_slot_num, uint16_t nrec, uint16_t nrep, uint32_t origin_last_ce_us, uint32_t lat, 
                                    uint16_t origin_ce_num, uint32_t rem_us, uint16_t conn_handle, uint8_t search_order)
{
    int offset;
    uint8_t flag;
    rt_ble_conn_t *conn;
    uint16_t off;
    int16_t diff;
    int slot_num = origin_slot_num;
    int8_t level = origin_level;
    uint32_t last_ce_us = origin_last_ce_us;
    uint16_t ce_num = origin_ce_num;

	// print_tree();
    while (level >= 0) {
        diff = left_occupied_num[level] - right_occupied_num[level];
//        printf("left_occupied = %d, right_occupied = %d\n",  left_occupied_num[level], right_occupied_num[level]);
        if ((search_order == SEARCH_LEFT_FIRTST) ||
        	((diff > 0) && (diff < 2) && (left_occupied_num[level] % 2 == 1)) || (diff <= 0)) {
            for (int i = 0; i < pow2[level]; i++) {
                if ((tree[level][i] & 0xE0) == TYPE_EMPTY) {
                    flag = 1;
                    off = reverse_bit(i, level+1);
                    for (offset = off; offset < pow2[tree_level]; offset += pow2[level]) {
                        if (continue_cell_resource[offset] < slot_num) {
                            flag = 0;
                            break;
                        }
                    }
                    // We allocate the first cell that can satisfy the application requirements
                    if (flag) {
                        conn = (rt_ble_conn_t *)malloc(sizeof(rt_ble_conn_t));
                        conn->conn_handle = conn_handle;
                        conn->level = level;
                        conn->slot_num = slot_num;
                        conn->offset = off;
                        return conn;
                    }
                }
            }
        }
        else {
            for (int i = pow2[level] / 2; i < pow2[level]; i++) {
                if ((tree[level][i] & 0xE0) == TYPE_EMPTY) {
                    flag = 1;
                    off = reverse_bit(i, level+1);
                    for (offset = off; offset < pow2[tree_level]; offset += pow2[level]) {
                        if (continue_cell_resource[offset] < slot_num) {
                            flag = 0;
                            break;
                        }
                    }
                    // We allocate the first cell that can satisfy the application requirements
                    if (flag) {
                        conn = (rt_ble_conn_t *)malloc(sizeof(rt_ble_conn_t));
                        conn->conn_handle = conn_handle;
                        conn->level = level;
                        conn->slot_num = slot_num;
                        conn->offset = off;
                        return conn;
                    }
                }
            }
            for (int i = 0; i < pow2[level] / 2; i++) {
                if ((tree[level][i] & 0xE0) == TYPE_EMPTY) {
                    flag = 1;
                    off = reverse_bit(i, level+1);
                    for (offset = off; offset < pow2[tree_level]; offset += pow2[level]) {
                        if (continue_cell_resource[offset] < slot_num) {
                            flag = 0;
                            break;
                        }
                    }
                    // We allocate the first cell that can satisfy the application requirements
                    if (flag) {
                        conn = (rt_ble_conn_t *)malloc(sizeof(rt_ble_conn_t));
                        conn->conn_handle = conn_handle;
                        conn->level = level;
                        conn->slot_num = slot_num;
                        conn->offset = off;
                        return conn;
                    }
                }
            }
        }
        //================================================
        do {
            level--;
            if (level < 0) {
                return NULL;
            }
            slot_num = ((origin_slot_num - 1) / (pow2[origin_level] / pow2[level])) + 1;
            if (slot_num <= 2) {
                rem_us = 0;
                ce_num = (pow2[origin_level] / pow2[level]) + MAX(nrec, nrep);
                if (MAX(nrec, nrep) == 0) {
                    last_ce_us = origin_last_ce_us - (pow2[origin_level] / pow2[level] - 1) * (slot_num * SLOT_US - START_UP_US);
                }
                else {
                    // The former one already be the retransimission CE
                    last_ce_us = origin_last_ce_us;
                }
            }
            else {
                ce_num = origin_ce_num + (pow2[origin_level] / pow2[level]) - 1;
                if (nrec == 0 && nrep == 0) {
                    last_ce_us = origin_last_ce_us - (pow2[origin_level] / pow2[level] - 1) * (slot_num * SLOT_US - START_UP_US);
                }
                else {
                    last_ce_us = origin_last_ce_us;
                }
            }
        } while(last_ce_us + ce_num * pow2[level] * SLOT_US + rem_us > lat);
    }
    return NULL;
}

int8_t rt_ble_get_level(uint32_t ci_us)
{
    uint32_t ci_slots = ci_us / SLOT_US;
    uint32_t num = 512;
    int8_t level = 8;
    while (level >= 0) {
        if (num <= ci_slots) {
            return level;
        }
        num = num >> 1;
        level--;
    }
	return level;
}

int rt_ble_occupy(uint8_t level, uint16_t offset, uint16_t slot_num, uint16_t conn_handle)
{
    uint16_t index;
    int num;
    uint16_t temp;
    int upd_index;
    uint8_t left_flag;
    // printf("level=%d, offset=%d, slot_num = %d, conn_handle=%d\n", level, offset, slot_num, conn_handle);
    conn_handle &= 0x1f;
    for (int s = 0; s < slot_num; s++) {
        left_flag = ((offset+s) % 2 == 0)? 1:0;
        index = reverse_bit(offset+s, level+1);
        temp =  TYPE_PARENT_MASK | conn_handle;
        for (int i = 0; i < level; i++) {
            tree[i][reverse_bit(offset+s, i+1)] = temp;
            if (left_flag) {
                left_occupied_num[i]++;
            }
            else {
                right_occupied_num[i]++;
            }
        }

        tree[level][index] = TYPE_OCCUPIED | conn_handle;
        if (left_flag) {
            left_occupied_num[level]++;
        }
        else {
            right_occupied_num[level]++;
        }
        temp = TYPE_CHILD_MASK | conn_handle;
        num = 1;
        for (int i = level+1; i < 9; i++) {
            index = index * 2;
            num = num * 2;
            for (int j = index; j < index + num; j++) {
                tree[i][j] = temp;
            }
            if (left_flag) {
                left_occupied_num[i] += num;
            }
            else {
                right_occupied_num[i] += num;
            }
        }
    }
    for (int i = offset; i < pow2[tree_level]; i += pow2[level]) {
        temp = continue_cell_resource[i];
        continue_cell_resource[i] = 0;
        upd_index = i - 1;
        memset(continue_cell_resource + i, 0, 2 * slot_num);
        while (upd_index >= 0 && continue_cell_resource[upd_index] > 0) {
            continue_cell_resource[upd_index] -= temp;
            upd_index--;
        }
    }
    return 0;
}

int rt_ble_release(uint8_t level, uint16_t offset, uint16_t slot_num)
{
    uint16_t index;
    uint16_t num;
    uint16_t p_index, b_index, p_offset;
    uint16_t end_index;

    for (int i = 0; i < slot_num; i++) {
        // The part lower than the occupied cell
        index = reverse_bit(offset+i, level+1);
        num = 1;
        for (int l = level; l < 9; l++) {
            memset(tree[l]+index*num, 0, num);
            if ((offset+i) % 2 == 0) {
                left_occupied_num[l] -= num;
            }
            else {
                right_occupied_num[l] -= num;
            }
            num = num << 1;
        }
        // Then we update the parent node
        for (int l = level; l > 0; l--) {
            index = reverse_bit(offset, l+1);
            b_index = index % 2 == 0? index+1: index-1;
            p_index = MIN(index, b_index) / 2;
            p_offset = (offset >= pow2[l])? (offset - pow2[l]): offset;
            if (((tree[l][index] & TYPE_MASK) == TYPE_EMPTY) &&
            	((tree[l][b_index] & TYPE_MASK) == TYPE_EMPTY)) {
                tree[l-1][p_index] = 0;
                if (p_offset % 2 == 0) {
                    left_occupied_num[l-1]--;
                }
                else {
                    right_occupied_num[l-1]--;
                }
            }
        }
    }
    for (int i = offset; i < pow2[tree_level]; i+=pow2[level]) {
        end_index = i + slot_num;
        if (end_index == (pow2[tree_level]-1)) {
            continue_cell_resource[end_index] = 1;
        }
        else {
            continue_cell_resource[end_index] = continue_cell_resource[end_index+1] + 1;
        }
        end_index--;
        while(end_index >= i || continue_cell_resource[end_index] > 0) {
            continue_cell_resource[end_index] = continue_cell_resource[end_index+1] + 1;
            end_index--;
        }
    }
    return 0;
}

rt_ble_conn_t* rt_ble_allocate(uint16_t central_pktlen, uint16_t peripheral_pktlen,
								uint32_t wcet_lat, uint32_t wcet_prob, uint16_t conn_handle,
								uint8_t normal)
{
    uint16_t ce_slots_num;
    uint16_t new_ce_slots_num;
    uint16_t ce_num;
    uint32_t last_ce_us;
    int32_t ci_us;
    int8_t target_level;
    uint16_t nc;
    uint16_t nrec;
    uint16_t np;
    uint16_t nrep;
    uint32_t c_last_us;
    uint32_t p_last_us;
    uint32_t ce_duration;
    uint32_t rem_us;
    rt_ble_conn_t *conn = NULL;
    int ce_limit;
    int p_rem;
    int c_rem;
    int rem_ce_num;

    if (pending_conn) {
        printf("pending is not null\n");
        return NULL;
    }

    /* RT-BLE: calculate the number of L2CAP packets */
    if (central_pktlen == 0) {
		nc = 0;
		nrec = 0;
    }
    else {
    	/* RT-BLE: the first L2CAP packet has 2 bytes to store the length of
		 * the total L2CAP packet.
		 * And we take the upper integer
    	 */
		nc = (central_pktlen + 1) / 247 + 1;
		nrec = rt_ble_retrans_num(nc, PKT_LOSS_RATE, wcet_prob);
    }
    if (peripheral_pktlen == 0) {
		np = 0;
		nrep = 0;
    }
    else {
		np = (peripheral_pktlen + 1) / 247 + 1;
		nrep = rt_ble_retrans_num(np, PKT_LOSS_RATE, wcet_prob);
    }
    // printf("nc = %d, np = %d, nrec = %d, nrep = %d\n", nc, np, nrec, nrep);
    wcet_lat = 1000 * wcet_lat;  // Turn latency to microsecond

    if (peripheral_pktlen == 0) {
        p_last_us = 0;
    }
    else if ((peripheral_pktlen + 2) % 247 == 0) {
        p_last_us = MAX_L2CAP_US;
    }
    else {
        p_last_us = 112 + ((peripheral_pktlen + 2) % 247) * 8;
    }
    if (central_pktlen == 0) {
        c_last_us = 0;
    }
    else if ((central_pktlen + 2) % 247 == 0) {
        c_last_us = MAX_L2CAP_US;
    }
    else {
        c_last_us = 112 + ((central_pktlen + 2) % 247) * 8;
    }

    if (np > nc) {
        ce_duration = START_UP_US + (MAX(nc-1, 0) + MAX(np-1, 0)) * MAX_L2CAP_US + 
                        c_last_us + p_last_us + MIN(np - nc, nrec) * c_last_us + 80 * MAX(np-nc-nrec, 0) + np * 300;
        if (nrep > 0) {
            last_ce_us = START_UP_US + p_last_us + 80 + 300;
        }
        else {
            last_ce_us = ce_duration;
        }
    }
    else if (nc > np) {
        ce_duration = START_UP_US + (MAX(nc-1, 0) + MAX(np-1, 0)) * MAX_L2CAP_US + 
                      c_last_us + p_last_us + MIN(nc - np, nrep) * p_last_us + 80 * MAX(nc-np-nrep, 0) + nc * 300;
        if (nrec > 0) {
            last_ce_us = START_UP_US + c_last_us + 80 + 300;
        }
        else {
            last_ce_us = ce_duration;
        }
    }
    else {
        ce_duration = START_UP_US + (MAX(nc-1, 0) + MAX(np-1, 0)) * MAX_L2CAP_US + 
                      c_last_us + p_last_us + nc * 300;
        if (nrec > 0){
            last_ce_us = START_UP_US + c_last_us + p_last_us + 300;
        }
        else {
            last_ce_us = ce_duration;
        }
    }

    ce_slots_num = (ce_duration - 1) / SLOT_US + 1;
    ce_limit = (ce_slots_num - 1) / 2 + 1;
    p_rem = np + nrep - MAX(nc, np);
    if (p_rem < 0) {
        p_rem = 0;
    }
    c_rem = nc + nrec - MAX(nc, np);
    if (c_rem < 0) {
        c_rem = 0;
    }
    rem_ce_num = 0;
    if (ce_slots_num > 2) {
        if (c_rem == p_rem) {
            if (c_rem == 0) {
                ce_num = 1;
            }
            else {
                ce_num = 2 + (c_rem - 1) / ce_limit;
                rem_ce_num = (c_rem - 1) % ce_limit;
            }
        }
        else if (c_rem > p_rem) {
            if (c_rem == 1) {
                ce_num = 2;
            }
            else {
                ce_num = 3 + (c_rem - 2) / ce_limit;
            }
        }
        else {
            if (c_rem == 0) {
                ce_num = 1;
            }
            else {
                ce_num = 2 + (c_rem - 1) / ce_limit;
            }
            ce_num += p_rem - c_rem;
        }
    }
    else {
        ce_num = 1 + MAX(nrec, nrep);
    }
    rem_us = rem_ce_num * 10008;



    /* For now, the continuation number is 1, the retransmission can be done in the next connection event 
     * rather than the next base event */

    // if (ce_slots_num > 2 && nc > np) {
    //     uint16_t n_pend;
    //     if (nrec % 2 == 1) {
    //         n_pend = 1 + (nrec + 1) / 2;
    //     }
    //     else if (nrec > 0){
    //         /* The last packet is transmitted on the continuation event */
    //         n_pend = 1 + nrec / 2 + 1;
    //     }
    //     else {
    //         n_pend = 1;
    //     }
    //     // ce_num = n_pend;
    //     if ((np + nrep > nc) && (np + nrep - nc + 1) > n_pend) {
    //         // ce_num = np + nrep - nc + 1;
    //         if ((np + nrep - nc) == n_pend) {
    //             rem_us = 10008;
    //         }
    //         ce_num = np + nrep - nc;
    //     }
    //     else {
    //         ce_num = n_pend;
    //     }
    // }
    // else if (ce_slots_num > 2 && nc <= np && ((nc + nrec) > np)) {// here the np >= nc
    //     uint16_t skip_num = nc + nrec - np;
    //     uint16_t n_cend = 2 + skip_num / 2;
    //     if (nrep == n_cend) {
    //         rem_us = 10008;
    //     }
    //     ce_num = nrep;  // = 1 + nrep - 1. the nrec > 0, so no must > 0
    // }
    // else {
    //     ce_num = 1 + MAX(nrec, nrep);
    // }


    ci_us = (wcet_lat - last_ce_us) / ce_num; 
    if (ci_us < 10008) {
    	printf("no enough time resource\n");
		return NULL;
    }
    target_level = rt_ble_get_level(ci_us);
    if (target_level > tree_level) {
        rt_ble_tree_extend(target_level);
    }
    if (normal) {
        conn = rt_ble_find_resource(target_level, ce_slots_num, nrec, nrep, last_ce_us, wcet_lat, ce_num, rem_us, conn_handle, SEARCH_NONE);
    }
    else {
        // We allocate the slots for the node into critical mode
        if (conn_list[conn_handle-1]->offset % 2 == 0) {
            /* The cell is on the left tree */
            conn = rt_ble_find_resource(target_level, ce_slots_num, nrec, nrep, last_ce_us, wcet_lat, ce_num, rem_us, conn_handle, SEARCH_LEFT_FIRTST);
        }
        else {
            /* The cell is on the right tree */
            conn = rt_ble_find_resource(target_level, ce_slots_num, nrec, nrep, last_ce_us, wcet_lat, ce_num, rem_us, conn_handle, SEARCH_RIGHT_FIRST);
        }
    }
    if (conn) {
        if (conn->level < target_level) {
            printf("new level is lower than initial\n");
            new_ce_slots_num = (ce_slots_num - 1) / (pow2[target_level] / pow2[conn->level]) + 1;
            if (new_ce_slots_num <= 2) {
                ce_num = (pow2[target_level] / pow2[conn->level]) + MAX(nrec, nrep);
                if (MAX(nrec, nrep) == 0) {
                    last_ce_us = last_ce_us - (pow2[target_level] / pow2[conn->level] - 1) * (new_ce_slots_num * SLOT_US - START_UP_US);
                }
            }
            else {
                ce_num = ce_num - 1 + (pow2[target_level] / pow2[conn->level]);
                if (nrec ==0 && nrep == 0) {
                    last_ce_us = last_ce_us - (pow2[target_level] / pow2[conn->level] - 1) * (new_ce_slots_num * SLOT_US - START_UP_US);
                }
            }
            // printf("allocated conn=%d, level=%d, slot_num=%d, offset=%d\n", conn_handle, conn->level, conn->slot_num, conn->offset);
            // printf("est ce_num = %d, last_ce_us = %lu, trans_time = %lu\n", ce_num, last_ce_us, rem_us + last_ce_us + (ce_num-1) * pow2[conn->level] * 5004);
        }
        // else {
            // printf("allocated conn=%d, level=%d, slot_num=%d, offset=%d\n", conn_handle, conn->level, conn->slot_num, conn->offset);
            // To measure the latency on slave, we can only know when the packet is transmitted, without the 150 MSS after the transmission or receiving,
            // So we just minus 150 MSS at the end
            // printf("est ce_num = %d, last_ce_us = %lu, trans_time = %lu,", ce_num, last_ce_us, rem_us + last_ce_us + (ce_num-1) * pow2[conn->level] * 5004 - 150);
            // printf("slave trans time = %lu\n",  rem_us + last_ce_us + (ce_num-1) * pow2[conn->level] * 5004 - 80 - START_UP_US - 150);
        // }
    }
    return conn;
}


int rt_ble_connected(uint16_t conn_handle, uint16_t pri_len)
{
	struct ble_ll_conn_sm *connsm;
	uint32_t base_anchor;
	int32_t next_num;
	uint32_t cur_anchor_point;
	uint16_t cntr_diff;
	uint16_t cntr;
	int res;


    if (current_mode != MODE_ESTB_CONN || (!pending_conn) ||
    	(pending_conn && conn_handle != pending_conn->conn_handle)) {
    	// Some thing is wrong
    	assert(0);
    }
    connsm = ble_ll_conn_find_by_handle(conn_handle);
    if (!connsm) {
		return -EINVAL;
    }
   	connsm->cur_mode = CONNSM_STATE_NORMAL;
    connsm->level = pending_conn->level;
    connsm->offset = pending_conn->offset;
    connsm->slot_num = pending_conn->slot_num;
    if (pri_len == 0) {
        // The always sends a empty PDU
        connsm->peripheral_us = 80;
    }
    else {
        connsm->peripheral_us = (pri_len + 2) % 247;
        if (connsm->peripheral_us == 0) {
            connsm->peripheral_us = 247;
        }
        connsm->peripheral_us = connsm->peripheral_us * 8 + 80 + 32;
    }

    /* For now, the connection subrate is not started */
    /* We set the base tick to the next conenction anchor point */
    cur_anchor_point = connsm->anchor_point - g_ble_ll_sched_offset_ticks;
    cntr = connsm->event_cntr;
	if (conn_handle == 1 && conn1_flag == 0) {
		printf("set the base tick=%ld\n", cur_anchor_point);
		conn1_flag = 1;
		base_tick = cur_anchor_point;
    }
	next_num = (int32_t)(cur_anchor_point - base_tick - connsm->offset * 164);
	if (next_num >= 1) {
		base_anchor = (next_num - 1) / pow164[connsm->level] + 1;
		base_anchor = base_tick + connsm->offset * 164 + pow164[connsm->level] * base_anchor;
		cntr_diff = (base_anchor - cur_anchor_point) / 328;
	}
	else {
		base_anchor = 0;
		cntr_diff = (base_tick + connsm->offset * 164 - cur_anchor_point) / 328;
	}
	printf("new_cntr = %d, level = %d offset = %d \n", cntr + cntr_diff, connsm->level, connsm->offset);
	res = rt_ble_subrating_tweak(connsm, pow2[connsm->level]/2, cntr + cntr_diff, connsm->slot_num);
    return res;
}

int rt_ble_disconnect(uint16_t conn_handle)
{
    if (critical_conn_list[conn_handle-1]) {
        rt_ble_release(critical_conn_list[conn_handle-1]->level,
        		critical_conn_list[conn_handle-1]->offset, critical_conn_list[conn_handle-1]->slot_num);
        free(critical_conn_list[conn_handle-1]);
        critical_conn_list[conn_handle-1] = NULL;
    }
    if (conn_list[conn_handle-1]) {
        rt_ble_release(conn_list[conn_handle-1]->level, conn_list[conn_handle-1]->offset,
        				conn_list[conn_handle-1]->slot_num);
        free(conn_list[conn_handle-1]);
        conn_list[conn_handle-1] = NULL;
    }
    return 0;
}

void rt_ble_clear_pending(void)
{
    if (pending_conn) {
        free(pending_conn);
        pending_conn = NULL;
        current_mode = MODE_IDLE;
    }
}

static int rt_ble_subrating_critical(struct ble_ll_conn_sm *connsm, rt_ble_conn_t *target_conn)
{
	uint32_t sub_an;
    uint16_t cntr;
    int32_t n;

	if (!connsm) return -1;
	sub_an = connsm->subrate_anchor_point - g_ble_ll_sched_offset_ticks;
	cntr = connsm->event_cntr;
	n = sub_an - base_tick - 164 * target_conn->offset;
	if (n >= 1) {
		n = (n - 1) / pow164[target_conn->level] + 1;
		n = base_tick + 164 * target_conn->offset + n * pow164[target_conn->level] - sub_an;
	}
	else {
		n = base_tick + 164* target_conn->offset - sub_an;
	}

	if (n % 328 != 0) {
		puts("offset is not correct");
		return -EINVAL;
	}
	n = n / 328;
	printf("subrate param, level=%d, subrate factor = %d, slot_num = %d\n",target_conn->level,
			pow2[target_conn->level] / 2, target_conn->slot_num);
	return rt_ble_subrating_tweak(connsm, pow2[target_conn->level] / 2, cntr + n, target_conn->slot_num);
}

static int rt_ble_conn_upd_critical(struct ble_ll_conn_sm *connsm)
{
	int res;
	/* First, we switch the subrate factor to 1 */
	connsm->subrate_sema = 0;
	res = rt_ble_subrating_tweak(connsm, 1, connsm->subrate_base_event, 1);
	if (res) {
		puts("subrate failed");
		return res;
	}
	/* Then, we move the anchor point 5ms (1 slot) after*/
	res = rt_ble_conn_upd_tweak(connsm, 4);
	if (res) {
		puts("conn upd failed");
		return res;
	}
	/* Wait util the last connection event before the instant */
	rt_ble_conn_upd_wait();
	connsm->offset++;
	connsm->subrate_sema = 1;
	return 0;
}

int rt_ble_subrating_enter_critical(uint16_t conn_handle)
{
    struct ble_ll_conn_sm *connsm = ble_ll_conn_find_by_handle(conn_handle);

	if (!connsm) {
		return -EINVAL;
	}
	if ((connsm->offset % 2) != (pending_conn->offset % 2)) {
		puts("not in the same sub-tree");
		return -EINVAL;
	}

	return rt_ble_subrating_critical(connsm, pending_conn);
}

int rt_ble_conn_upd_enter_critical(uint16_t conn_handle)
{
    struct ble_ll_conn_sm *connsm = ble_ll_conn_find_by_handle(conn_handle);

    if ((conn_list[conn_handle-1]->offset % 2) == (pending_conn->offset % 2)) {
		return 0;
    }
    if (!connsm) {
		return -EINVAL;
    }
    return rt_ble_conn_upd_critical(connsm);
}

int rt_ble_subrating_leave_critical(uint16_t conn_handle)
{
    struct ble_ll_conn_sm *connsm = ble_ll_conn_find_by_handle(conn_handle);

    if (!connsm) {
		return -EINVAL;
    }
    if (critical_conn_list[conn_handle-1] == NULL) {
		return -EALREADY;
    }
	if ((conn_list[conn_handle-1]->offset % 2) != (critical_conn_list[conn_handle-1]->offset % 2)) {
		puts("Can not use subrating to leave critical");
		return -EINVAL;
	}
	return rt_ble_subrating_critical(connsm, conn_list[conn_handle-1]);


    return 0;
}

int rt_ble_conn_upd_leave_critical(uint16_t conn_handle)
{
	struct ble_ll_conn_sm *connsm = ble_ll_conn_find_by_handle(conn_handle);
	if (!connsm) {
		return -EINVAL;
	}
	if ((conn_list[conn_handle-1]->offset % 2) == (critical_conn_list[conn_handle-1]->offset % 2)) {
		return 0;
	}
	return rt_ble_conn_upd_critical(connsm);
}

rt_ble_conn_t* rt_ble_add_new_conn(uint16_t conn_handle, uint16_t central_pktlen,
							uint16_t peripheral_pktlen, uint32_t lat, uint32_t precent)
{
    if (conn_list[conn_handle-1]) {
    	puts("Connection already exist");
        return NULL;
    }
    printf("Add new connetion handle = %d, centeral_pktlen = %d, peripheral_pktlen = %d, lat = %lu, precentile = %lu\n",
    		conn_handle, central_pktlen, peripheral_pktlen, lat, precent);
    pending_conn = rt_ble_allocate(central_pktlen, peripheral_pktlen, lat, precent, conn_handle, 1);
    if (pending_conn) {
        current_mode = MODE_ESTB_CONN;
        return pending_conn;
    }
    printf("New conn level=%d, offset = %d, slot_num=%d\n", pending_conn->level, pending_conn->offset, pending_conn->slot_num);
    return NULL;
}

int rt_ble_enter_critical(uint16_t conn_handle, uint16_t central_pktlen,
							uint16_t peripheral_pktlen, uint32_t lat, uint32_t wcet_prob)
{
	int rc = 0;
    rt_ble_conn_t *conn;
    if (critical_conn_list[conn_handle-1]) {
        return -EEXIST;
    }

    conn = rt_ble_allocate(central_pktlen, peripheral_pktlen, lat, wcet_prob, conn_handle, 0);
    if (!conn) {
        return -ENOSPC;
    }
	current_mode = MODE_ENTER_CRITICAL;
	pending_conn = conn;
    if ((conn->offset % 2) != (conn_list[conn_handle-1]->offset % 2)) {
        rc = rt_ble_conn_upd_enter_critical(conn_handle);
    }
    /* The new cell is in the same sub-tree */
    rc = rt_ble_subrating_enter_critical(conn_handle);
	if (rc) {
		current_mode = MODE_IDLE;
		free(pending_conn);
		pending_conn = NULL;
	}
    return rc;
}

int rt_ble_leave_critical(uint16_t conn_handle)
{
	int res;
    if (!critical_conn_list[conn_handle-1]) {
        return -EALREADY;
    }
    if ((conn_list[conn_handle-1]->offset % 2) != (critical_conn_list[conn_handle-1]->offset % 2)) {
    	res = rt_ble_conn_upd_leave_critical(conn_handle);
    	expect(res == 0);
    }
	res = rt_ble_subrating_leave_critical(conn_handle);
	expect(res == 0);
    current_mode = MODE_LEAVE_CRITICAL;
    pending_conn = critical_conn_list[conn_handle-1];
    return 0;
}

int rt_ble_update_callback(uint16_t conn_handle)
{
    int rc;
    struct ble_ll_conn_sm *connsm = ble_ll_conn_find_by_handle(conn_handle);
    if (!connsm) {
		assert(0);
    }

    if (current_mode == MODE_ESTB_CONN && pending_conn && pending_conn->conn_handle == conn_handle) {
		rt_ble_occupy(pending_conn->level, pending_conn->offset, pending_conn->slot_num,
						pending_conn->conn_handle);
		conn_list[conn_handle-1] = pending_conn;
		connsm->cur_mode = CONNSM_STATE_NORMAL;
        connsm->level = pending_conn->level;
        connsm->offset = pending_conn->offset;
        connsm->slot_num = pending_conn->slot_num;
        pending_conn = NULL;
		current_mode = MODE_IDLE;
    }

    if (current_mode == MODE_ENTER_CRITICAL && pending_conn && pending_conn->conn_handle == conn_handle) {
        /* The connection has entered into critical mode */
        rc = rt_ble_occupy(pending_conn->level, pending_conn->offset, pending_conn->slot_num,
        					pending_conn->conn_handle);
        if (rc) {
            return rc;
        }
        connsm->cur_mode = CONNSM_STATE_CRITICAL;
        connsm->level = pending_conn->level;
        connsm->offset = pending_conn->offset;
        connsm->slot_num = pending_conn->slot_num;
        critical_conn_list[conn_handle-1] = pending_conn;
        pending_conn = NULL;
        current_mode = MODE_IDLE;
    }
    if (current_mode == MODE_LEAVE_CRITICAL && pending_conn && pending_conn->conn_handle == conn_handle) {
        /* The connection leave the critical mode, release the resources */
        rc = rt_ble_release(pending_conn->level, pending_conn->offset, pending_conn->slot_num);
        if (rc) {
            return rc;
        }
        connsm->cur_mode = CONNSM_STATE_NORMAL;
        connsm->level = conn_list[conn_handle-1]->level;
        connsm->offset = conn_list[conn_handle-1]->offset;
        connsm->slot_num = conn_list[conn_handle-1]->slot_num;
        free(critical_conn_list[conn_handle-1]);
        critical_conn_list[conn_handle-1] = NULL;
        current_mode = MODE_IDLE;
        pending_conn = NULL;
    }
    return 0;
}

rt_ble_conn_t* rt_ble_add_test_conn(int8_t level)
{
    rt_ble_conn_t *conn = (rt_ble_conn_t *)malloc(sizeof(rt_ble_conn_t));
    memset(conn, 0, sizeof(rt_ble_conn_t));
    conn->conn_handle = 1;
    conn->level = level;
    conn->offset = 0;
    conn->slot_num = 1;
    pending_conn = conn;
    current_mode = MODE_ESTB_CONN;
    rt_ble_tree_extend(level);
    return pending_conn;
}


int rt_ble_move_right_odd(void)
{
    int res;
    struct ble_ll_conn_sm *connsm = ble_ll_conn_find_by_handle(1);
    // printf(",connsm cntr = %d, anchor = %lu", connsm->event_cntr, connsm->anchor_point);
	/* First, we switch the subrate factor to 1 */
	connsm->subrate_sema = 0;
    printf(",start connsm->anchor=%lu,cntr=%d", os_cputime_ticks_to_usecs(connsm->anchor_point), connsm->event_cntr);
	res = rt_ble_subrating_tweak(connsm, 1, connsm->subrate_base_event, 1);
	if (res) {
		puts("subrate failed");
		return res;
	}
	/* Then, we move the anchor point 5ms (1 slot) after*/
	res = rt_ble_conn_upd_tweak(connsm, 4);
	if (res) {
		puts("conn upd failed");
		return res;
	}
	/* Wait util the last connection event before the instant */
	rt_ble_conn_upd_wait();
    printf(",upd_cntr=%d,an=%lu,cur=%lu", connsm->event_cntr, os_cputime_ticks_to_usecs(connsm->anchor_point), os_cputime_ticks_to_usecs(os_cputime_get32()));
	connsm->offset++;
	connsm->subrate_sema = 1;
    return rt_ble_subrating_tweak(connsm, pow2[connsm->level] / 2, connsm->subrate_base_event + 1, 1);
}

int rt_ble_move_right_even(void)
{
    struct ble_ll_conn_sm *connsm = ble_ll_conn_find_by_handle(1);
	return rt_ble_subrating_tweak(connsm, pow2[connsm->level] / 2, connsm->subrate_base_event, 1);
}