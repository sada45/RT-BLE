/*
 * Copyright (C) 2022 Zhejiang University
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    sys_blex blex
 * @ingroup     sys
 * @brief       blex
 *
 * @{
 *
 * @file
 *
 * @author      Yeming Li <liymemnets@zju.edu.cn>
 */

#ifndef BLEX_H
#define BLEX_H

/* Add header includes here */

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

typedef struct timetable_item{
	uint16_t handle;
	uint32_t start_ticks;
	uint32_t end_ticks;
	uint32_t duration;
	struct timetable_item *next;
}timetable_item_t;

typedef struct blex_conn{
	uint16_t conn_itvl;
	uint16_t offset;
}blex_conn_t;


int blex_add_new_conn(uint16_t conn_handle, uint16_t central_pktlen,
									uint16_t peripheral_pktlen, uint32_t latency, blex_conn_t *conn);

int blex_try_add(uint16_t conn_handle, uint16_t central_pktlen,
							uint16_t peripheral_pktlen, uint32_t latency);

int blex_update(void);

void blex_connected(uint16_t conn_handle);

void blex_move_right(void);

void blex_move_left(void);

void blex_init(void);

#endif /* BLEX_H */
/** @} */
