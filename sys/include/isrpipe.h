/*
 * Copyright (C) 2016 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup isr_pipe ISR Pipe
 * @ingroup sys
 * @brief ISR -> userspace pipe
 *
 * @{
 * @file
 * @brief       isrpipe Interface
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 *
 */

#ifndef ISRPIPE_H
#define ISRPIPE_H

#include <stdint.h>

#include "mutex.h"
#include "tsrb.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Context structure for isrpipe
 */
typedef struct {
    tsrb_t tsrb;        /**< isrpipe thread safe ringbuffer */
    mutex_t mutex;      /**< isrpipe mutex */
} isrpipe_t;

/**
 * @brief   Static initializer for irspipe
 */
#define ISRPIPE_INIT(tsrb_buf) { .mutex = MUTEX_INIT, \
                                 .tsrb = TSRB_INIT(tsrb_buf) }

/**
 * @brief   Initialisation function for isrpipe
 *
 * @param[in]   isrpipe     isrpipe object to initialize
 * @param[in]   buf         buffer to use as ringbuffer (must be power of two sized!)
 * @param[in]   bufsize     size of @p buf
 */
void isrpipe_init(isrpipe_t *isrpipe, uint8_t *buf, size_t bufsize);

/**
 * @brief   Put one byte into the isrpipe's buffer
 *
 * @param[in]   isrpipe     isrpipe object to operate on
 * @param[in]   c           byte to add to isrpipe buffer
 *
 * @returns     0 if byte could be added
 * @returns     -1 if buffer was full
 */
int isrpipe_write_one(isrpipe_t *isrpipe, uint8_t c);

/**
 * @brief   Put number of bytes into the isrpipe's buffer
 *
 * @param[in]   isrpipe     isrpipe object to operate on
 * @param[in]   buf         bytes to add to isrpipe buffer
 * @param[in]   n           number of bytes to add from buf to isrpipe's buffer
 *
 * @returns     number of bytes that could be added
 * @returns     -1 if buffer was full
 */
int isrpipe_write(isrpipe_t *isrpipe, const uint8_t *buf, size_t n);

/**
 * @brief   Read data from isrpipe (blocking)
 *
 * @param[in]   isrpipe    isrpipe object to operate on
 * @param[in]   buf        buffer to write to
 * @param[in]   count      number of bytes to read
 *
 * @returns     number of bytes read
 */
int isrpipe_read(isrpipe_t *isrpipe, uint8_t *buf, size_t count);

#ifdef __cplusplus
}
#endif
/** @} */
#endif /* ISRPIPE_H */
