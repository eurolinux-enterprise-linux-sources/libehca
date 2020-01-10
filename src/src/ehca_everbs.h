/*
 *  IBM eServer eHCA Infiniband device driver for Linux on POWER
 *
 *  eHCA specific verbs for qp's direct access in user space
 *
 *  Authors: Christoph Raisch <raisch@de.ibm.com>
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

#ifndef __EHCA_EVERBS_H__
#define __EHCA_EVERBS_H__

#include <linux/types.h>

/*
 * direct access qp and send/recv completion flags (can be OR-ed)
 * set this option in ibv_qp_init_attr.qp_type when creating a daqp
 */
enum ehcau_completion_flag {
	DA_SEND_COMPLETION = 0x20,
	DA_RECV_COMPLETION = 0x40,
	DA_QP_ENABLE = 0x80
};

/*
 * da send/recv message size
 * set send/recv message size in ibv_ap_init_attr.cap.max_send/recv_sge
 * respectively when creating a daqp
 */
enum ehcau_msg_size {
	DA_MSG_SIZE_128 = 0,
	DA_MSG_SIZE_256 = 1,
	DA_MSG_SIZE_512 = 2,
	DA_MSG_SIZE_1024 = 3,
	DA_MSG_SIZE_2048 = 4,
	DA_MSG_SIZE_4096 = 5
};

/*
 *
 * ehcau_qp_attr_da - enhanced qp attr containing send/recv queue data
 */
struct ehcau_qp_attr_da {
	/* wqe size in bytes for send */
	__u16 send_wqe_size;
	/* total length of send queue */
	__u64 send_queue_length;
	/* send queue pointer, i.e. first wqe address */
	void *send_queue_ptr;
	/* wqe size in bytes for recv */
	__u16 recv_wqe_size;
	/* total length of recv queue */
	__u64 recv_queue_length;
	/* recv queue pointer, i.e. first wqe address */
	void *recv_queue_ptr;
};

/*
 * returns enhanced qp attr for the given qp
 */
int ehcau_query_qp_da(struct ibv_qp *qp, struct ehcau_qp_attr_da *qp_attr);

/*
 * returns enhanced qp attr for the given srq
 * use only recv queue info of qp_attr
 */
int ehcau_query_srq_da(struct ibv_srq *srq, struct ehcau_qp_attr_da *qp_attr);

/*
 * increments send queue adder and triggers HCA to process written messages
 */
int ehcau_send_wr_trigger(struct ibv_qp *qp, int msg_count);

/*
 * increments recv queue adder
 */
int ehcau_recv_wr_trigger(struct ibv_qp *qp, int msg_count);

/*
 * writes list of send wr to the given send wqe addr
 * @return number of written wqes
 */
int ehcau_write_swqe(void *wqe, struct ibv_qp *qp, struct ibv_send_wr *send_wr,
		     struct ibv_send_wr **bad_wr);

/*
 * writes list of recv wr to the given recv wqe addr
 * @return number of written wqes
 */
int ehcau_write_rwqe(void *wqe, struct ibv_qp *qp, struct ibv_recv_wr *recv_wr,
		     struct ibv_recv_wr **bad_wr);

/*
 * writes list of recv wr to the given recv wqe addr
 * @return number of written wqes
 */
int ehcau_write_srq_rwqe(void *wqe, struct ibv_srq *srq, struct ibv_recv_wr *recv_wr,
			 struct ibv_recv_wr **bad_wr);

/*
 * returns send queue's next entry ptr
 */
#define GET_SQUEUE_NEXT_ENTRY_PTR(current_ptr, qp_attr) \
	(((void *)current_ptr + (qp_attr).send_wqe_size >= \
	  (qp_attr).send_queue_ptr + (qp_attr).send_queue_length) ? \
	 (qp_attr).send_queue_ptr : (void *)current_ptr + \
	 (qp_attr).send_wqe_size)

/*
 * returns recv queue's next entry ptr
 */
#define GET_RQUEUE_NEXT_ENTRY_PTR(current_ptr, qp_attr) \
	(((void *)current_ptr + (qp_attr).recv_wqe_size >= \
	  (void *)(qp_attr).recv_queue_ptr + (qp_attr).recv_queue_length) ? \
	 (qp_attr).recv_queue_ptr : (void *)current_ptr + \
	 (qp_attr).recv_wqe_size)

#endif /* __EHCA_EVERBS_H__ */
