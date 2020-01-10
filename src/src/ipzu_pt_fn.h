/*
 *  IBM eServer eHCA Infiniband device driver for Linux on POWER
 *
 *  internal queue handling for user space
 *
 *  Authors: Waleri Fomin <fomin@de.ibm.com>
 *           Reinhard Ernst <rernst@de.ibm.com>
 *           Christoph Raisch <raisch@de.ibm.com>
 *           Hoang-Nam Nguyen <hnguyen@de.ibm.com>
 *
 *  Copyright (c) 2005 IBM Corporation
 *
 *  All rights reserved.
 *
 *  This source code is distributed under a dual license of GPL v2.0 and OpenIB
 *  BSD.
 *
 * OpenIB BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials
 * provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __IPZU_PT_FN_H__
#define __IPZU_PT_FN_H__

#include "ehca_utools.h"
#include "ehca_qes.h"

struct ipzu_queue {
	u8 *current_q_addr;         /* current queue entry */
	u8 *queue;                  /* points to first queue entry */
	u32 qe_size;                /* queue entry size */
	u32 act_nr_of_sg;
	u32 queue_length;           /* queue length allocated in bytes */
	u32 pagesize;
	u32 toggle_state;           /* toggle flag - per page */
	u32 offset;                 /* 64 bit alignment*/
};

/* returns address of current Queue Entry */
static inline void *ipzu_qeit_get(struct ipzu_queue *queue)
{
	return queue->current_q_addr;
}

/*
 * return current Queue Page , increment Queue Page iterator from
 * page to page in struct ipzu_queue, last increment will return 0! and
 * NOT wrap
 * warning: don't use in parallel with ipzu_qeit_get_inc()
 */
void *ipzu_qpageit_get_inc(struct ipzu_queue *queue);

/*
 * return current Queue Entry, increment Queue Entry iterator by one
 * step in struct ipzu_queue, will wrap in ringbuffer
 * (returns address of current Queue Entry BEFORE increment)
 * warning: don't use in parallel with ipzu_qpageit_get_inc()
 * warning: unpredictable results may occur if steps>act_nr_of_queue_entries
 */
static inline void *ipzu_qeit_get_inc(struct ipzu_queue *queue)
{
	void *retvalue;
	u8 *last_entry_in_q = queue->queue + queue->queue_length
		- queue->qe_size;

	retvalue = queue->current_q_addr;
	queue->current_q_addr += queue->qe_size;
	if (queue->current_q_addr > last_entry_in_q) {
		queue->current_q_addr = queue->queue;
		/* toggle the valid flag */
		queue->toggle_state = (~queue->toggle_state) & 1;
	}

	ehca_gen_dbg("queue=%p retvalue=%p new current_q_addr=%p qe_size=%x",
		     queue, retvalue, queue->current_q_addr, queue->qe_size);
	return retvalue;
}

/* return current Queue Entry, increment Queue Entry iterator by one
 * step in struct ipzu_queue, will wrap in ringbuffer
 * (returns address of current Queue Entry BEFORE increment)
 * (returns 0 and does not increment, if wrong valid state)
 * warning: don't use in parallel with ipzu_qpageit_get_inc()
 * warning: unpredictable results may occur if steps>act_nr_of_queue_entries
 */
inline static void *ipzu_qeit_get_inc_valid(struct ipzu_queue *queue)
{
	void *retvalue = ipzu_qeit_get(queue);
	u32 qe = ((struct ehca_cqe *)retvalue)->cqe_flags;
	if ((qe >> 7) != (queue->toggle_state & 1))
		return NULL;
	/* this is a good one */
	ipzu_qeit_get_inc(queue);
	return retvalue;
}

/*
 * returns and resets Queue Entry iterator
 * (returns address (kv) of first Queue Entry)
 */
static inline void *ipzu_qeit_reset(struct ipzu_queue *queue)
{
	queue->current_q_addr = queue->queue;
	return queue->queue;
}

#endif /* __IPZU_PT_FN_H__ */
