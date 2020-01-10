/*
 *  IBM eServer eHCA Infiniband device driver for Linux on POWER
 *
 *  struct definitions for hcad internal structures and our uverbs decls
 *
 *  Authors: Christoph Raisch <raisch@de.ibm.com>
 *           Hoang-Nam Nguyen <hnguyen@de.ibm.com>
 *           Joachim Fenkes <fenkes@de.ibm.com>
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

#ifndef __EHCA_UCLASSES_H__
#define __EHCA_UCLASSES_H__

#include <infiniband/verbs.h>
#include <infiniband/driver.h>
#include <sys/queue.h>
#include "ipzu_pt_fn.h"
#include "ehca_galpa.h"

#define LIBEHCA_VERSION "1.2.2"
#define ABI_VERSION 8
#define ABI_VERSION_STRING "8"

#define abiver5_rhel45  0
#define abiver5_vanilla 1
#define abiver6_rhel51  2
#define abiver6_vanilla 3

#define QP_PER_DEVICE   64 * 1024

extern int ehca_use_abi;

struct ehcau_device {
	struct ehcau_qp *srqs[QP_PER_DEVICE];
	unsigned int srq_lockvar;

	struct ibv_device ibv_dev;
};

struct ehcau_context {
	/* NOTE: ibv_ctx must be the last of this struct, since
	 * cq_fd must be large enough as requested by num_comp
	 */
	struct ibv_context ibv_ctx;
};

struct ehcau_av {
	struct ibv_ah ib_ah;
	struct ehca_ud_av av;
};

enum ehcau_ext_qp_type {
	EQPT_NORMAL    = 0,
	EQPT_LLQP      = 1,
	EQPT_SRQBASE   = 2,
	EQPT_SRQ       = 3,
};

struct ehcau_qp {
	union {
		struct ibv_qp ib_qp;
		struct ibv_srq ib_srq;
	};
	int qp_type;
	enum ehcau_ext_qp_type ext_type;
	struct ipzu_queue ipz_squeue;
	struct ipzu_queue ipz_rqueue;
	struct h_galpas galpas;
	unsigned int qkey;
	/* qp_num assigned by ehca: sqp0/1 may have got different numbers */
	unsigned int real_qp_num;
        unsigned int lockvar_s;
        unsigned int lockvar_r;
	u32 qp_num;
	u32 token;
	struct ehcau_cq* send_cq;
	unsigned int sqerr_purgeflag;
	LIST_ENTRY(ehcau_qp) list_entries;
	/* unsolicited ack circumvention */
	int unsol_ack_circ;
	int mtu_shift;
	int sq_sig_all;
	u32 nr_swqes;
	u32 message_count;
	u32 packet_count;
	/* srq overflow circumvention */
	unsigned int slots_posted;
	unsigned int slots_polled;
};

#define IS_SRQ(qp) (qp->ext_type == EQPT_SRQ)
#define HAS_SQ(qp) (qp->ext_type != EQPT_SRQ)
#define HAS_RQ(qp) (qp->ext_type != EQPT_SRQBASE)

LIST_HEAD(qp_list, ehcau_qp);

/* must be power of 2 */
#define QP_HASHTAB_LEN 8
struct ehcau_cq {
	struct ibv_cq ib_cq;
	struct ipzu_queue ipz_queue;
	struct h_galpas galpas;
        unsigned int lockvar;
	u32 cq_number;
	u32 token;
	struct qp_list qp_hashtab[QP_HASHTAB_LEN];
};

int ehcau_query_device(struct ibv_context *context,
		       struct ibv_device_attr *device_attr);
int ehcau_query_port(struct ibv_context *context,
		     uint8_t port_num, struct ibv_port_attr *attr);

int ehcau_query_gid(struct ibv_context *context, uint8_t port_num,
		    int index, union ibv_gid *gid);

int ehcau_query_pkey(struct ibv_context *context, uint8_t port_num,
		     int index, uint16_t * pkey);

struct ibv_pd *ehcau_alloc_pd(struct ibv_context *context);

int ehcau_dealloc_pd(struct ibv_pd *pd);

struct ibv_mr *ehcau_reg_mr(struct ibv_pd *pd,
			    void *addr,
			    size_t length, enum ibv_access_flags access);

int ehcau_dereg_mr(struct ibv_mr *mr);

struct ibv_cq *ehcau_create_cq_abiver5(struct ibv_context *context, int cqe,
			       struct ibv_comp_channel *channel,
			       int comp_vector);

struct ibv_cq *ehcau_create_cq_abiver6(struct ibv_context *context, int cqe,
			       struct ibv_comp_channel *channel,
			       int comp_vector);

struct ibv_cq *ehcau_create_cq_abiver7(struct ibv_context *context, int cqe,
			       struct ibv_comp_channel *channel,
			       int comp_vector);

struct ibv_cq *ehcau_create_cq(struct ibv_context *context, int cqe,
			       struct ibv_comp_channel *channel,
			       int comp_vector);

int ehcau_destroy_cq(struct ibv_cq *cq);

int ehcau_destroy_cq_abiver7(struct ibv_cq *cq);

struct ibv_qp *ehcau_create_qp_abiver5(struct ibv_pd *pd,
			       struct ibv_qp_init_attr *attr);

struct ibv_qp *ehcau_create_qp_abiver6(struct ibv_pd *pd,
			       struct ibv_qp_init_attr *attr);

struct ibv_qp *ehcau_create_qp_abiver7(struct ibv_pd *pd,
			       struct ibv_qp_init_attr *attr);

struct ibv_qp *ehcau_create_qp(struct ibv_pd *pd,
			       struct ibv_qp_init_attr *attr);

int ehcau_modify_qp(struct ibv_qp *qp, struct ibv_qp_attr *attr,
		    enum ibv_qp_attr_mask attr_mask);

int ehcau_destroy_qp(struct ibv_qp *qp);

int ehcau_destroy_qp_abiver7(struct ibv_qp *qp);

struct ibv_srq *ehcau_create_srq_abiver5(struct ibv_pd *pd,
					 struct ibv_srq_init_attr *attr);

struct ibv_srq *ehcau_create_srq_abiver7(struct ibv_pd *pd,
					 struct ibv_srq_init_attr *attr);

struct ibv_srq *ehcau_create_srq(struct ibv_pd *pd,
				 struct ibv_srq_init_attr *attr);

int ehcau_modify_srq(struct ibv_srq *srq, struct ibv_srq_attr *attr,
		     enum ibv_srq_attr_mask attr_mask);

int ehcau_query_srq(struct ibv_srq *srq, struct ibv_srq_attr *srq_attr);

int ehcau_destroy_srq(struct ibv_srq *srq);

int ehcau_destroy_srq_abiver7(struct ibv_srq *srq);

struct ibv_ah *ehcau_create_ah(struct ibv_pd *pd, struct ibv_ah_attr *attr);

int ehcau_destroy_ah(struct ibv_ah *ah);

int ehcau_post_send(struct ibv_qp *qp, struct ibv_send_wr *wr,
		    struct ibv_send_wr **bad_wr);

int ehcau_post_recv(struct ibv_qp *qp, struct ibv_recv_wr *wr,
		    struct ibv_recv_wr **bad_wr);

int ehcau_post_srq_recv(struct ibv_srq *srq, struct ibv_recv_wr *wr,
			struct ibv_recv_wr **bad_wr);

int ehcau_req_notify_cq(struct ibv_cq *cq, int solicited);

int ehcau_poll_cq(struct ibv_cq *cq, int num_entries, struct ibv_wc *wc);

int ehcau_attach_mcast(struct ibv_qp *qp, union ibv_gid *gid, uint16_t lid);

int ehcau_detach_mcast(struct ibv_qp *qp, union ibv_gid *gid, uint16_t lid);

int ehcau_query_qp(struct ibv_qp *qp, struct ibv_qp_attr *qp_attr,
		   enum ibv_qp_attr_mask attr_mask,
		   struct ibv_qp_init_attr* init_attr);

/*
 * resp structs from kernel space
 */
struct ipzu_queue_resp {
	u32 qe_size;      /* queue entry size */
	u32 act_nr_of_sg;
	u32 queue_length; /* queue length allocated in bytes */
	u32 pagesize;
	u32 toggle_state;
	u32 offset;
};

struct ipzu_queue_resp_abiver5 {
	u64 queue;
	u32 qe_size;      /* queue entry size */
	u32 act_nr_of_sg;
	u32 queue_length; /* queue length allocated in bytes */
	u32 pagesize;
	u32 toggle_state;
	u32 offset;
};

struct ehcau_create_cq_resp {
	struct ibv_create_cq_resp ibv_resp;
	u32 cq_number;
	u32 token;
	struct ipzu_queue_resp ipz_queue;
	u32 fw_handle_ofs;
	u32 dummy;
};

struct ehcau_create_cq_resp_abiver5 {
	struct ibv_create_cq_resp ibv_resp;
	u32 cq_number;
	u32 token;
	struct ipzu_queue_resp_abiver5 ipz_queue;
	struct h_galpas galpas;
	u32 fw_handle_ofs;
	u32 dummy;
};

struct ehcau_create_cq_resp_abiver6 {
	struct ibv_create_cq_resp ibv_resp;
	u32 cq_number;
	u32 token;
	struct ipzu_queue_resp ipz_queue;
};

struct ehcau_create_cq_resp_abiver7 {
	struct ibv_create_cq_resp ibv_resp;
	u32 cq_number;
	u32 token;
	struct ipzu_queue_resp ipz_queue;
	u32 fw_handle_ofs;
	u32 dummy;
};

struct ehca_create_qp_resp {
	u32 qp_num;
	u32 token;
	u32 qp_type;
	u32 ext_type;
	u32 qkey;
	/* qp_num assigned by ehca: sqp0/1 may have got different numbers */
	u32 real_qp_num;
	u32 fw_handle_ofs;
	u32 dummy; /* padding for 8 byte alignment */
	struct ipzu_queue_resp ipz_squeue;
	struct ipzu_queue_resp ipz_rqueue;
};

struct ehca_create_qp_resp_abiver5 {
	u32 qp_num;
	u32 token;
	u32 qp_type;
	u32 ext_type;
	u32 qkey;
	/* qp_num assigned by ehca: sqp0/1 may have got different numbers */
	u32 real_qp_num;
	u32 fw_handle_ofs;
	u32 dummy; /* padding for 8 byte alignment */
	struct ipzu_queue_resp_abiver5 ipz_squeue;
	struct ipzu_queue_resp_abiver5 ipz_rqueue;
};

struct ehca_create_qp_resp_abiver5_20 {
	u32 qp_num;
	u32 token;
	u32 qp_type;
	u32 qkey;
	/* qp_num assigned by ehca: sqp0/1 may have got different numbers */
	u32 real_qp_num;
	u32 dummy; /* padding for 8 byte alignment */
	struct ipzu_queue_resp_abiver5 ipz_squeue;
	struct ipzu_queue_resp_abiver5 ipz_rqueue;
};

struct ehca_create_qp_resp_abiver6 {
	u32 qp_num;
	u32 token;
	u32 qp_type;
	u32 qkey;
	/* qp_num assigned by ehca: sqp0/1 may have got different numbers */
	u32 real_qp_num;
	u32 dummy; /* padding for 8 byte alignment */
	struct ipzu_queue_resp ipz_squeue;
	struct ipzu_queue_resp ipz_rqueue;
};

struct ehca_create_qp_resp_abiver7 {
	u32 qp_num;
	u32 token;
	u32 qp_type;
	u32 ext_type;
	u32 qkey;
	/* qp_num assigned by ehca: sqp0/1 may have got different numbers */
	u32 real_qp_num;
	u32 dummy; /* padding according to kernel struct */
	struct ipzu_queue_resp ipz_squeue;
	struct ipzu_queue_resp ipz_rqueue;
};

struct ehcau_create_qp_resp {
	struct ibv_create_qp_resp ibv_resp;
	struct ehca_create_qp_resp ehca_resp;
};

struct ehcau_create_qp_resp_abiver5 {
	struct ibv_create_qp_resp ibv_resp;
	struct ehca_create_qp_resp_abiver5 ehca_resp;
	struct h_galpas galpas;
};

struct ehcau_create_qp_resp_abiver5_20 {
	struct ibv_create_qp_resp ibv_resp;
	struct ehca_create_qp_resp_abiver5_20 ehca_resp;
	struct h_galpas galpas;
};

struct ehcau_create_qp_resp_abiver6 {
	struct ibv_create_qp_resp ibv_resp;
	struct ehca_create_qp_resp_abiver6 ehca_resp;
};

struct ehcau_create_qp_resp_abiver7 {
	struct ibv_create_qp_resp ibv_resp;
	struct ehca_create_qp_resp_abiver7 ehca_resp;
};

struct ehcau_create_srq_resp {
	struct ibv_create_srq_resp ibv_resp;
	struct ehca_create_qp_resp ehca_resp;
};

struct ehcau_create_srq_resp_abiver5 {
	struct ibv_create_srq_resp ibv_resp;
	struct ehca_create_qp_resp_abiver5 ehca_resp;
	struct h_galpas galpas;
};

struct ehcau_create_srq_resp_abiver7 {
	struct ibv_create_srq_resp ibv_resp;
	struct ehca_create_qp_resp_abiver7 ehca_resp;
};

struct ehcau_qp* ehca_cq_get_qp(struct ehcau_cq *cq, int qp_num);

void cq_assign_qp(struct ehcau_cq *cq, struct ehcau_qp *qp);

inline static void ehcau_lock(unsigned int * lock_var) {
        register unsigned int temp1 asm ("r5");
        register unsigned int setval asm ("r4")=1;
        __asm__ __volatile__ (
                "1: lwarx %0,0,%2\n"
                "   cmpwi cr0,%0,0\n"
                "   bne- 1b\n"
                "   stwcx. %1,0,%2\n"
                "   bne- 1b\n"
                "   lwsync\n"
                : "=&r" (temp1), "+&r" (setval) : "r" (lock_var) : "cr0",
		"memory");
}

inline static void ehcau_unlock(unsigned int * lock_var) {
	__asm__ __volatile__ ("   lwsync\n");
	*lock_var=0;
}

#endif /* __EHCA_UCLASSES_H__ */
