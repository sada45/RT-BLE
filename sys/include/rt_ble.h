/*
 * Copyright (C) 2022 Zhejiang University
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    sys_rt_ble rt_ble
 * @ingroup     sys
 * @brief       RT-BLE module
 *
 * @{
 *
 * @file
 *
 * @author      Yeming Li < liymemnets@zju.edu.cn>
 */

#ifndef RT_BLE_H
#define RT_BLE_H

/* Add header includes here */

#ifdef __cplusplus
extern "C" {
#endif

/* Declare the API of the module */

#ifdef __cplusplus
}
#endif

#ifndef PKT_LOSS_RATE
#define PKT_LOSS_RATE   0
#endif

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))


// The types of cell node
#define TYPE_EMPTY          0x00
#define TYPE_OCCUPIED       0x20
#define TYPE_OCCUPIED_SUB   0x40
#define TYPE_CHILD_MASK     0x60
#define TYPE_PARENT_MASK    0x80
#define CLEAR               0x1f
#define TYPE_MASK           0xe0
#define MODE_IDLE               0
#define MODE_ESTB_CONN          1
#define MODE_ENTER_CRITICAL     2
#define MODE_LEAVE_CRITICAL     3

#define SEARCH_NONE         0
#define SEARCH_LEFT_FIRTST  1
#define SEARCH_RIGHT_FIRST  2

/* The time for calculation (unit = us) */
#define MAX_L2CAP_US        2088
#define MIN_LL_US           80
#define MAX_EXCH_US         4476
#define ONE_MAX_EXCH_US     2468
#define EMPTY_EXCH_US       460
#define SLOT_US             5004
#define START_UP_US			213
#define L2CAP_EXCH_TIME(a, b)     (300 + 160 + (a + 4) * 8 +  (b + 4)* 8)

typedef struct rt_ble_conn {
    uint16_t conn_handle;
    int8_t level;
    uint8_t slot_num;
    uint16_t offset;
}rt_ble_conn_t;

void rt_ble_tree_init(void);

rt_ble_conn_t* rt_ble_allocate(uint16_t central_pktlen, uint16_t peripheral_pktlen,
								uint32_t wcet_lat, uint32_t wcet_prob, uint16_t conn_handle, uint8_t normal);

int rt_ble_connected(uint16_t conn_handle, uint16_t pri_len);

int rt_ble_disconnect(uint16_t conn_handle);

void rt_ble_clear_pending(void);

int rt_ble_subrating_enter_critical(uint16_t conn_handle);

int rt_ble_conn_upd_enter_critical(uint16_t conn_handle);

int rt_ble_subrating_leave_critical(uint16_t conn_handle);

int rt_ble_conn_upd_leave_critical(uint16_t conn_handle);

rt_ble_conn_t* rt_ble_add_new_conn(uint16_t conn_handle, uint16_t central_pktlen,
							uint16_t peripheral_pktlen, uint32_t lat, uint32_t precent);

int rt_ble_enter_critical(uint16_t conn_handle, uint16_t central_pktlen,
								uint16_t peripheral_pktlen, uint32_t lat, uint32_t wcet_prob);

int rt_ble_leave_critical(uint16_t conn_handle);

int rt_ble_update_callback(uint16_t conn_handle);

rt_ble_conn_t* rt_ble_add_test_conn(int8_t level);

int rt_ble_move_right_odd(void);

void print_tree(void);

int rt_ble_occupy(uint8_t level, uint16_t offset, uint16_t slot_num, uint16_t conn_handle);

int rt_ble_move_right_even(void);
#endif /* RT_BLE_H */
/** @} */
