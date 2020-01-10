/*
 *  IBM eServer eHCA Infiniband device driver for Linux on POWER
 *
 *  post_send/recv, poll_cq, req_notify
 *
 *  Authors: Waleri Fomin <fomin@de.ibm.com>
 *           Joachim Fenkes <fenkes@de.ibm.com>
 *           Reinhard Ernst <rernst@de.ibm.com>
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


#include <errno.h>
#include <unistd.h>
#include <netinet/in.h>
#include <infiniband/verbs.h>
#include <string.h>
#include "ehca_uclasses.h"
#include "ehca_utools.h"
#include "hipz_fns_core.h"
#include "ehca_everbs.h"
#include "ehca_asm.h"
#include "ipzu_pt_fn.h"

/* in RC traffic, insert an empty RDMA READ every this many packets */
#define ACK_CIRC_THRESHOLD 2000000

static inline int write_rwqe(struct ipzu_queue *ipz_rqueue,
			     struct ehca_wqe *wqe_p,
			     struct ibv_recv_wr *recv_wr)
{
	u8 cnt_ds;
	if (unlikely((recv_wr->num_sge < 0) ||
		     (recv_wr->num_sge > ipz_rqueue->act_nr_of_sg))) {
		ehca_gen_err("Invalid number of WQE SGE. "
			     "num_sqe=%x max_nr_of_sg=%x",
			     recv_wr->num_sge, ipz_rqueue->act_nr_of_sg);
		return -EINVAL; /* invalid SG list length */
	}

	clear_cacheline(wqe_p);
	clear_cacheline((u8*)wqe_p + 32);
	clear_cacheline((u8*)wqe_p + 64);

	wqe_p->work_request_id = be64_to_cpu(recv_wr->wr_id);
	wqe_p->nr_of_data_seg = recv_wr->num_sge;

	for (cnt_ds = 0; cnt_ds < recv_wr->num_sge; cnt_ds++) {
		wqe_p->u.all_rcv.sg_list[cnt_ds].vaddr =
			be64_to_cpu(recv_wr->sg_list[cnt_ds].addr);
		wqe_p->u.all_rcv.sg_list[cnt_ds].lkey =
			ntohl(recv_wr->sg_list[cnt_ds].lkey);
		wqe_p->u.all_rcv.sg_list[cnt_ds].length =
			ntohl(recv_wr->sg_list[cnt_ds].length);
	}

	if (unlikely(libehca_trace_on)) {
		ehca_gen_dbg("RECEIVE WQE written into ipz_rqueue=%p",
			     ipz_rqueue);
		ehca_dmp_dbg(wqe_p, 16*(6 + wqe_p->nr_of_data_seg), "recv wqe");
	}

	return 0;
}

static inline int write_swqe(struct ehcau_qp *qp,
			     struct ehca_wqe *wqe_p,
			     const struct ibv_send_wr *send_wr,
			     int hidden)
{
	u32 idx;
	u64 dma_length;
	struct ehcau_av *my_av;
	u32 remote_qkey = send_wr->wr.ud.remote_qkey;
	uint64_t mtu = 0;
	uint32_t seg_len = 0;
	uint64_t wqe_len = 0;

	clear_cacheline(wqe_p);
	clear_cacheline((u8 *)wqe_p + 32);

	if (unlikely((send_wr->num_sge < 0) ||
		     (send_wr->num_sge > qp->ipz_squeue.act_nr_of_sg))) {
		ehca_gen_err("Invalid number of WQE SGE. "
			     "num_sqe=%x max_nr_of_sg=%x",
			     send_wr->num_sge, qp->ipz_rqueue.act_nr_of_sg);
		return -EINVAL; /* invalid SG list length */
	}

	wqe_p->work_request_id = be64_to_cpu(send_wr->wr_id);

	switch (send_wr->opcode & 0xF) {
	case IBV_WR_SEND:
	case IBV_WR_SEND_WITH_IMM:
		wqe_p->optype = WQE_OPTYPE_SEND;
		break;
	case IBV_WR_RDMA_WRITE:
	case IBV_WR_RDMA_WRITE_WITH_IMM:
		wqe_p->optype = WQE_OPTYPE_RDMAWRITE;
		break;
	case IBV_WR_RDMA_READ:
		wqe_p->optype = WQE_OPTYPE_RDMAREAD;
		break;
	default:
		ehca_gen_err("Invalid opcode=%x", send_wr->opcode);
		return -EINVAL; /* invalid opcode */
	}

	wqe_p->wqef = (send_wr->opcode) & 0xF0;

	wqe_p->wr_flag = 0;

	if ((send_wr->send_flags & IBV_SEND_SIGNALED || qp->sq_sig_all)
	    && !hidden)
		wqe_p->wr_flag |= WQE_WRFLAG_REQ_SIGNAL_COM;


	if (send_wr->opcode == IBV_WR_SEND_WITH_IMM ||
	    send_wr->opcode == IBV_WR_RDMA_WRITE_WITH_IMM) {
		/* this might not work as long as HW does not support it */
		wqe_p->immediate_data = send_wr->imm_data;
		wqe_p->wr_flag |= WQE_WRFLAG_IMM_DATA_PRESENT;
	}

	wqe_p->nr_of_data_seg = send_wr->num_sge;

	switch (qp->qp_type) {
	case IBV_QPT_UD:
		/* IB 1.2 spec C10-15 compliance */
		if (send_wr->wr.ud.remote_qkey & 0x80000000)
			remote_qkey = qp->qkey;
		wqe_p->destination_qp_number =
			ntohl(send_wr->wr.ud.remote_qpn << 8);
		wqe_p->local_ee_context_qkey = ntohl(remote_qkey);
		if (unlikely(!send_wr->wr.ud.ah)) {
			ehca_gen_err("wr.ud.ah is NULL. qp=%p", qp);
			return -EINVAL;
		}
		if (unlikely(send_wr->wr.ud.remote_qpn == 0)) {
			ehca_gen_err("dest QP# is 0. qp=%p", qp);
			return -EINVAL;
		}
		my_av = container_of(send_wr->wr.ud.ah, struct ehcau_av, ib_ah);
		wqe_p->u.ud_av.ud_av = my_av->av;

		/* calculate the mtu size out of the av
		 * av - internal ehca structure written by query_port
		 * values allowed for pmtu 1-5 (256 - 4096 byte)
		 */
		mtu = 0x80 << my_av->av.pmtu;

		/* omitted check of IBV_SEND_INLINE
		 * since HW does not support it
		 */
		for (idx = 0; idx < send_wr->num_sge; idx++) {
			wqe_p->u.ud_av.sg_list[idx].vaddr =
				be64_to_cpu(send_wr->sg_list[idx].addr);
			wqe_p->u.ud_av.sg_list[idx].lkey =
				ntohl(send_wr->sg_list[idx].lkey);
			seg_len = ntohl(send_wr->sg_list[idx].length);	
			wqe_p->u.ud_av.sg_list[idx].length = seg_len;
			wqe_len += seg_len;
		} /* eof for idx */
		if (unlikely(wqe_len > mtu)) {
				ehca_gen_err("WQE Size exceed path MTU size "
				"qp=%p wqe_size=%lx mtu=%lx", qp, wqe_len, mtu);
				return -EINVAL;
		}

		break;

	case IBV_QPT_UC:
		if (send_wr->send_flags & IBV_SEND_FENCE)
			wqe_p->wr_flag |= WQE_WRFLAG_FENCE;
		/* no break is intentional here */
	case IBV_QPT_RC:
		/*@@TODO atomic???*/
		wqe_p->u.nud.remote_virtual_adress =
			be64_to_cpu(send_wr->wr.rdma.remote_addr);
		wqe_p->u.nud.rkey = ntohl(send_wr->wr.rdma.rkey);

		/* omitted checking of IBV_SEND_INLINE
		 * since HW does not support it
		 */
		dma_length = 0;
		for (idx = 0; idx < send_wr->num_sge; idx++) {
			wqe_p->u.nud.sg_list[idx].vaddr =
				be64_to_cpu(send_wr->sg_list[idx].addr);
			wqe_p->u.nud.sg_list[idx].lkey =
				ntohl(send_wr->sg_list[idx].lkey);
			wqe_p->u.nud.sg_list[idx].length =
				ntohl(send_wr->sg_list[idx].length);
			dma_length += send_wr->sg_list[idx].length;
		} /* eof idx */
		wqe_p->u.nud.atomic_1st_op_dma_len = be64_to_cpu(dma_length);

		/* unsolicited ack circumvention */
		if (send_wr->opcode == IBV_WR_RDMA_READ) {
			/* on RDMA read, switch on and reset counters */
			qp->message_count = qp->packet_count = 0;
			qp->unsol_ack_circ = 1;
		} else
			/* else estimate #packets */
			qp->packet_count += (dma_length >> qp->mtu_shift) + 1;

		break;
	default:
		ehca_gen_err("Invalid qptype=%x", qp->qp_type);
		return -EINVAL;
	}

	if (unlikely(libehca_trace_on)) {
		ehca_gen_dbg("SEND WQE written into queue qp=%p ", qp);
		ehca_dmp_dbg(wqe_p, 16 * (6 + wqe_p->nr_of_data_seg),
			     "send wqe");
	}
	return 0;
}

/*
 * map_ib_wc_status - convert cqe_status to ib_wc_status
 */
static inline void map_ib_wc_status(u32 cqe_status,
				    enum ibv_wc_status *wc_status)
{
	if (unlikely(cqe_status & 0x80000000)) { /* complete with errors */
		switch (cqe_status & 0x0000003F) {
		case 0x01:
		case 0x21:
			*wc_status = IBV_WC_LOC_LEN_ERR;
			break;
		case 0x02:
		case 0x22:
			*wc_status = IBV_WC_LOC_QP_OP_ERR;
			break;
		case 0x03:
		case 0x23:
			*wc_status = IBV_WC_LOC_EEC_OP_ERR;
			break;
		case 0x04:
		case 0x24:
			*wc_status = IBV_WC_LOC_PROT_ERR;
			break;
		case 0x05:
		case 0x25:
			*wc_status = IBV_WC_WR_FLUSH_ERR;
			break;
		case 0x06:
			*wc_status = IBV_WC_MW_BIND_ERR;
			break;
		case 0x07: /* remote error - look into bits 20:24 */
			switch ((cqe_status & 0x0000F800) >> 11) {
			case 0x0:
				/* PSN Sequence Error!
				 * couldn't find a matching VAPI status!
				 */
				*wc_status = IBV_WC_GENERAL_ERR;
				break;
			case 0x1:
				*wc_status = IBV_WC_REM_INV_REQ_ERR;
				break;
			case 0x2:
				*wc_status = IBV_WC_REM_ACCESS_ERR;
				break;
			case 0x3:
				*wc_status = IBV_WC_REM_OP_ERR;
				break;
			case 0x4:
				*wc_status = IBV_WC_REM_INV_RD_REQ_ERR;
				break;
			}
			break;
		case 0x08:
			*wc_status = IBV_WC_RETRY_EXC_ERR;
			break;
		case 0x09:
			*wc_status = IBV_WC_RNR_RETRY_EXC_ERR;
			break;
		case 0x0A:
		case 0x2D:
			*wc_status = IBV_WC_REM_ABORT_ERR;
			break;
		case 0x0B:
		case 0x2E:
			*wc_status = IBV_WC_INV_EECN_ERR;
			break;
		case 0x0C:
		case 0x2F:
			*wc_status = IBV_WC_INV_EEC_STATE_ERR;
			break;
		case 0x0D:
			*wc_status = IBV_WC_BAD_RESP_ERR;
			break;
		case 0x10:
			/* WQE purged */
			*wc_status = IBV_WC_WR_FLUSH_ERR;
			break;
		default:
			*wc_status = IBV_WC_FATAL_ERR;

		}
	} else
		*wc_status = IBV_WC_SUCCESS;
}

static inline int post_one_send(struct ehcau_qp *my_qp,
			 struct ibv_send_wr *cur_send_wr,
			 struct ibv_send_wr **bad_send_wr,
			 int hidden)
{
	struct ehca_wqe *wqe_p;
	int ret;
	void *start_addr = my_qp->ipz_squeue.current_q_addr;

	/* get pointer next to free WQE */
	wqe_p = ipzu_qeit_get_inc(&my_qp->ipz_squeue);
	if (unlikely(!wqe_p)) {
		/* too many posted work requests: queue overflow */
		if (bad_send_wr)
			*bad_send_wr = cur_send_wr;
		ehca_err(my_qp->ib_qp.context->device, "Too many posted WQEs "
			 "qp_num=%x", my_qp->ib_qp.qp_num);
		return -ENOMEM;
	}
	/* write a SEND WQE into the QUEUE */
	ret = write_swqe(my_qp, wqe_p, cur_send_wr, hidden);
	/* if something failed, reset the
	 * free entry pointer to the start value
	 */
	if (unlikely(ret)) {
		my_qp->ipz_squeue.current_q_addr = start_addr;
		if (bad_send_wr)
			*bad_send_wr = cur_send_wr;
		ehca_err(my_qp->ib_qp.context->device, "Could not write WQE "
			 "qp_num=%x", my_qp->ib_qp.qp_num);
		return -EINVAL;
	}

	return 0;
}

int ehcau_post_send(struct ibv_qp *qp,
		    struct ibv_send_wr *send_wr,
		    struct ibv_send_wr **bad_send_wr)
{
	struct ehcau_qp *my_qp;
	struct ibv_send_wr *cur_send_wr;
	int wqe_cnt = 0;
	int retcode = 0;

        if (!qp) {
		ehca_gen_err("qp=%p check failed line %i", qp, __LINE__);
		return -EFAULT;
	}
	my_qp = container_of(qp, struct ehcau_qp, ib_qp);
        if (!send_wr) {
		ehca_gen_err("send_wr=%p check failed line %i",
			     send_wr, __LINE__);
		return -EFAULT;
	}

	ehca_dbg(qp->context->device, "ehcau_qp=%p qp_num=%x send_wr=%p "
		 "bad_send_wr=%p", my_qp, qp->qp_num, send_wr, bad_send_wr);

	/* LOCK the QUEUE */
	ehcau_lock(&my_qp->lockvar_s);

	/* Send an empty extra RDMA read if:
	 *  1) there has been an RDMA read on this connection before
	 *  2) no RDMA read occurred for ACK_CIRC_THRESHOLD link packets
	 *  3) we can be sure that any previous extra RDMA read has been
	 *     processed so we don't overflow the SQ
	 */
	if (unlikely(my_qp->unsol_ack_circ &&
		     my_qp->packet_count > ACK_CIRC_THRESHOLD &&
		     my_qp->message_count > my_qp->nr_swqes)) {
		/* insert an empty RDMA READ to fix up the remote QP state */
		struct ibv_send_wr circ_wr;
		memset(&circ_wr, 0, sizeof(circ_wr));
		circ_wr.opcode = IBV_WR_RDMA_READ;
		post_one_send(my_qp, &circ_wr, NULL, 1); /* ignore retcode */
		wqe_cnt++;
		ehca_dbg(qp->context->device, "posted circ wr  qp_num=%x", qp->qp_num);
		my_qp->message_count = my_qp->packet_count = 0;
	}

	/* loop processes list of send reqs */
	for (cur_send_wr = send_wr; cur_send_wr;
	     cur_send_wr = cur_send_wr->next) {
		retcode = post_one_send(my_qp, cur_send_wr, bad_send_wr, 0);
		if (unlikely(retcode)) {
			/* if one or more WQEs were successful, don't fail */
			if (wqe_cnt)
				retcode = 0;
			goto post_send_exit0;
		}

		wqe_cnt++;
		ehca_dbg(qp->context->device, "ehca_qp=%p qp_num=%x wqe_cnt=%d",
			 my_qp, qp->qp_num, wqe_cnt);
	} /* eof for cur_send_wr */

post_send_exit0:
	asm_sync_mem(); /* serialize GAL register access */
	hipz_update_SQA(my_qp, wqe_cnt);
	my_qp->message_count += wqe_cnt;
	ehcau_unlock(&my_qp->lockvar_s);
	ehca_dbg(qp->context->device, "ehca_qp=%p qp_num=%x ret=%i wqe_cnt=%d",
		 my_qp, qp->qp_num, retcode, wqe_cnt);
	return retcode;
}

static int is_srq_overflow(struct ehcau_qp *qp)
{
	return qp->slots_polled == qp->slots_posted;
}

static inline int internal_post_recv(struct ehcau_qp *my_qp,
				     struct ibv_recv_wr *recv_wr,
				     struct ibv_recv_wr **bad_recv_wr,
				     struct ibv_context *context)
{
	struct ibv_recv_wr *cur_recv_wr;
	struct ehca_wqe *wqe_p;
	int wqe_cnt = 0;
	int retcode = 0;

        if (!recv_wr) {
		ehca_gen_err("recv_wr=%p check failed line %i",
			     recv_wr, __LINE__);
		return -EFAULT;
	}

	ehca_dbg(context->device,
		 "ehca_qp=%p qp_num=%x recv_wr=%p bad_recv_wr=%p",
		 my_qp, my_qp->qp_num, recv_wr, bad_recv_wr);

	/* LOCK the QUEUE */
	ehcau_lock(&my_qp->lockvar_r);
	/* loop processes list of send reqs */
	for (cur_recv_wr = recv_wr; cur_recv_wr;
	     cur_recv_wr = cur_recv_wr->next) {
		void *start_addr = my_qp->ipz_rqueue.current_q_addr;

		if ((my_qp->ext_type == EQPT_SRQ) && is_srq_overflow(my_qp)) {
			if (bad_recv_wr)
				*bad_recv_wr = cur_recv_wr;
			if (!wqe_cnt)
				retcode = -ENOMEM;
			ehca_err(context->device, "SRQ overflow on qp_num=%x",
				 my_qp->qp_num);
			goto post_recv_exit0;
		}

		/* get pointer next to free WQE */
		wqe_p = ipzu_qeit_get_inc(&my_qp->ipz_rqueue);
		if (unlikely(!wqe_p)) {
			/* too many posted work requests: queue overflow */
			if (bad_recv_wr)
				*bad_recv_wr = cur_recv_wr;
			if (!wqe_cnt) {
				retcode = -ENOMEM;
				ehca_err(context->device,
					 "Too many posted WQEs qp_num=%x",
					 my_qp->qp_num);
			}
			goto post_recv_exit0;
		}
		/* write a RECV WQE into the QUEUE */
		retcode = write_rwqe(&my_qp->ipz_rqueue, wqe_p, cur_recv_wr);
		/* if something failed, reset the
		 * free entry pointer to the start value
		 */
		if (unlikely(retcode)) {
			my_qp->ipz_rqueue.current_q_addr = start_addr;
			*bad_recv_wr = cur_recv_wr;
			if (!wqe_cnt) {
				retcode = -EINVAL;
				ehca_err(context->device,
					 "Could not write WQE qp_num=%x",
					 my_qp->qp_num);
			}
			goto post_recv_exit0;
		}

		if (my_qp->ext_type == EQPT_SRQ)
			my_qp->slots_posted++;

		wqe_cnt++;
		ehca_dbg(context->device, "ehca_qp=%p qp_num=%x wqe_cnt=%d",
			 my_qp, my_qp->qp_num, wqe_cnt);
	} /* eof for cur_recv_wr */

post_recv_exit0:
	asm_sync_mem(); /* serialize GAL register access */
	hipz_update_RQA(my_qp, wqe_cnt);
	ehcau_unlock(&my_qp->lockvar_r);
	ehca_dbg(context->device, "ehca_qp=%p qp_num=%x ret=%i wqe_cnt=%d",
		 my_qp, my_qp->qp_num, retcode, wqe_cnt);
	return retcode;
}

int ehcau_post_recv(struct ibv_qp *qp,
		    struct ibv_recv_wr *recv_wr,
		    struct ibv_recv_wr **bad_recv_wr)
{
        if (!qp) {
		ehca_gen_err("qp=%p check failed line %i", qp, __LINE__);
		return -EFAULT;
	}

	return internal_post_recv(container_of(qp, struct ehcau_qp, ib_qp),
			   recv_wr, bad_recv_wr, qp->context);
}

int ehcau_post_srq_recv(struct ibv_srq *srq,
			struct ibv_recv_wr *recv_wr,
			struct ibv_recv_wr **bad_recv_wr)
{
        if (!srq) {
		ehca_gen_err("srq=%p check failed line %i", srq, __LINE__);
		return -EFAULT;
	}

	return internal_post_recv(container_of(srq, struct ehcau_qp, ib_srq),
				  recv_wr, bad_recv_wr, srq->context);
}

/*
 * Table converts ehca wc opcode to ib
 * Since we use zero to indicate invalid opcode, the actual ib opcode must
 * be decremented!!!
 */
static const u8 ib_wc_opcode[255] = {
	[0x00] = 1, /* for daqp optype is always zero */
	[0x01] = IBV_WC_RECV + 1,
	[0x02] = IBV_WC_RECV_RDMA_WITH_IMM + 1,
	[0x04] = IBV_WC_BIND_MW + 1,
	[0x08] = IBV_WC_FETCH_ADD + 1,
	[0x10] = IBV_WC_COMP_SWAP + 1,
	[0x20] = IBV_WC_RDMA_WRITE + 1,
	[0x40] = IBV_WC_RDMA_READ + 1,
	[0x80] = IBV_WC_SEND + 1
};

/* internal function to poll one entry of cq */
static inline int ehca_poll_cq_one(struct ibv_cq *cq, struct ibv_wc *wc)
{
	int retcode = 0;
	struct ehcau_cq *my_cq = container_of(cq, struct ehcau_cq, ib_cq);
	struct ehca_cqe *cqe;
	int cqe_count = 0;
	struct ehcau_device *my_dev;

	my_dev = container_of(cq->context->device, struct ehcau_device,
			      ibv_dev);

	ehca_dbg(cq->context->device, "ehca_cq=%p cq_num=%x wc=%p",
		 my_cq, my_cq->cq_number, wc);

poll_cq_one_read_cqe:
	cqe = (struct ehca_cqe *)ipzu_qeit_get_inc_valid(&my_cq->ipz_queue);
	if (!cqe) {
		retcode = -EAGAIN;
		ehca_dbg(cq->context->device,
			 "Completion queue is empty ehca_cq=%p cq_num=%x "
			 "retcode=%x", my_cq, my_cq->cq_number, retcode);
		goto  poll_cq_one_exit0;
	}

	/* prevents loads being reordered across this point */
	lwsync();

        cqe_count++;
        if (unlikely(cqe->status & 0x10)) { /* purge bit set */
                struct ehcau_qp *qp = ehca_cq_get_qp(my_cq,
						     cqe->local_qp_number);
		int purgeflag;
		if (!qp) { /* should not happen */
			ehca_err(cq->context->device, "cq_num=%x qp_num=%x "
				 "could not find qp -> ignore cqe",
				 my_cq->cq_number, cqe->local_qp_number);
			ehca_dmp_err(cqe, 64, "cq_num=%x qp_num=%x",
				     my_cq->cq_number, cqe->local_qp_number);
			/* ignore this purged cqe */
                        goto poll_cq_one_read_cqe;
		}
		ehcau_lock(&qp->lockvar_s);
		purgeflag = qp->sqerr_purgeflag;
		ehcau_unlock(&qp->lockvar_s);
                if (purgeflag) {
			ehca_dbg(cq->context->device,
				 "Got CQE with purged bit qp_num=%x src_qp=%x",
				 cqe->local_qp_number, cqe->remote_qp_number);
			ehca_dmp_dbg(cqe, 64, "qp_num=%x src_qp=%x",
				     cqe->local_qp_number,
				     cqe->remote_qp_number);
			/* ignore this to avoid double cqes of bad wqe
			 * that caused sqe and turn off purge flag
			 */
                        qp->sqerr_purgeflag = 0;
                        goto poll_cq_one_read_cqe;
                }
        }

	/* tracing cqe */
	if (unlikely(libehca_trace_on)) {
		ehca_dbg(cq->context->device,
			 "Received COMPLETION ehca_cq=%p cq_num=%x -----",
			 my_cq, my_cq->cq_number);
		ehca_dmp_dbg(cqe, 64, "ehca_cq=%p cq_num=%x",
			     my_cq, my_cq->cq_number);
		ehca_dbg(cq->context->device,
			 "ehca_cq=%p cq_num=%x -------------------------",
			 my_cq, my_cq->cq_number);
	}

	/* we got a completion! */
	wc->wr_id = cqe->work_request_id;

	if (cqe->w_completion_flags & WC_SEND_RECEIVE_BIT) {
		struct ehcau_qp *srq;

		ehcau_lock(&my_dev->srq_lockvar);
		srq = my_dev->srqs[cqe->local_qp_number];
		if (srq)
			srq->slots_polled++;
		ehcau_unlock(&my_dev->srq_lockvar);
	}

	/* eval ib_wc_opcode */
	wc->opcode = ib_wc_opcode[cqe->optype] - 1;
	if (unlikely(wc->opcode == -1)) {
		/* no error code required, but do trace out */
		ehca_err(cq->context->device, "Invalid cqe->OPType=%x "
			 "cqe->status=%x ehca_cq=%p cq_num=%x",
			 cqe->optype, cqe->status, my_cq, my_cq->cq_number);
		/* dump cqe for other infos */
		ehca_dmp_err(cqe, 64, "ehca_cq=%p cq_num=%x",
			     my_cq, my_cq->cq_number);
		/* update also queue adder to throw away this entry!!! */
		goto poll_cq_one_exit0;
	}
	wc->byte_len = ntohl(cqe->nr_bytes_transferred);
	/* eval ib_wc_status */
	if (unlikely(cqe->status & 0x80000000)) { /* complete with errors */
		map_ib_wc_status(cqe->status, &wc->status);
		wc->vendor_err = wc->status;
	} else
		wc->status = IBV_WC_SUCCESS;

	wc->qp_num = cqe->local_qp_number;
	wc->byte_len = cqe->nr_bytes_transferred;
	wc->pkey_index = cqe->pkey_index;
	wc->slid = cqe->rlid;
	wc->dlid_path_bits = cqe->dlid;
	wc->src_qp = cqe->remote_qp_number;
	/*
	 * HW has "Immed data present" and "GRH present" in bits 6 and 5.
	 * SW defines those in bits 1 and 0, so we can just shift and mask.
	 */
	wc->wc_flags = (cqe->w_completion_flags >> 5) & 3;
	wc->imm_data = cqe->immediate_data;
	wc->sl = cqe->service_level;

poll_cq_one_exit0:
	if (cqe_count > 0)
		hipz_update_FECA(my_cq, cqe_count);

	ehca_dbg(cq->context->device, "retcode=%x ehca_cq=%p cq_number=%x "
		 "wc=%p status=%x opcode=%x qp_num=%x byte_len=%x",
		 retcode, my_cq, my_cq->cq_number, wc, wc->status,
		 wc->opcode, wc->qp_num, wc->byte_len);
	return retcode;
}

int ehcau_poll_cq(struct ibv_cq *cq, int num_entries, struct ibv_wc *wc)
{
        if (!cq) {
		ehca_gen_err("cq=%p check failed line %i", cq, __LINE__);
		return -EFAULT;
	}
        if (!wc) {
		ehca_gen_err("wc=%p check failed line %i", wc, __LINE__);
		return -EFAULT;
	}

	int retcode = 0;
	struct ehcau_cq *my_cq = container_of(cq, struct ehcau_cq, ib_cq);
	if (num_entries < 1) {
		ehca_err(cq->context->device, "ehcau_cq=%p, "
			 "invalid num_entries=%d", my_cq, num_entries);
		return -EINVAL;
	}

	ehca_dbg(cq->context->device, "ehcau_cq=%p num_entries=%d wc=%p",
		 my_cq, num_entries, wc);

	int nr = 0;
	struct ibv_wc *current_wc = wc;
	ehcau_lock(&my_cq->lockvar);
	for (nr = 0; nr < num_entries; nr++) {
		retcode = ehca_poll_cq_one(cq, current_wc);
		if (retcode)
			break;
		current_wc++;
	} /* eof for nr */
	ehcau_unlock(&my_cq->lockvar);
	if (retcode == -EAGAIN || !retcode)
		retcode = nr;

	ehca_dbg(cq->context->device, "retcode=%x cq_number=%x wc=%p "
		 "nr_entries=%d", retcode, my_cq->cq_number, wc, nr);
	return retcode;
}

int ehcau_req_notify_cq(struct ibv_cq *cq, int solicited)
{
	struct ehcau_cq *my_cq;

        if (!cq) {
		ehca_gen_err("cq=%p check failed line %i", cq, __LINE__);
		return -EFAULT;
	}
	my_cq = container_of(cq, struct ehcau_cq, ib_cq);
	ehca_dbg(cq->context->device, "ehcau_cq=%p solicited=%x",
		 my_cq, solicited);

	if (solicited) /* IB_CQ_SOLICITED */
		hipz_set_CQx_N0(my_cq, 1);
	else /* IB_CQ_NEXT_COMP */
		hipz_set_CQx_N1(my_cq, 1);
	ehca_dbg(cq->context->device, "ehcau_cq=%p", cq);
	return 0;
}

int ehcau_send_wr_trigger(struct ibv_qp *qp, int wqe_count)
{
	struct ehcau_qp *my_qp;

        if (!qp) {
		ehca_gen_err("qp=%p check failed line %i", qp, __LINE__);
		return -EFAULT;
	}
	my_qp = container_of(qp, struct ehcau_qp, ib_qp);
	ehca_dbg(qp->context->device, "ehca_qp=%p wqe_count=%x",
		 my_qp, wqe_count);

	asm_sync_mem(); /* serialize GAL register access */
	hipz_update_SQA(my_qp, wqe_count);

	ehca_dbg(qp->context->device, "ehca_qp=%p wqe_count=%x",
		 my_qp, wqe_count);
	return 0;
}

int ehcau_recv_wr_trigger(struct ibv_qp *qp, int wqe_count)
{
	struct ehcau_qp *my_qp;

        if (!qp) {
		ehca_gen_err("qp=%p check failed line %i", qp, __LINE__);
		return -EFAULT;
	}
	my_qp = container_of(qp, struct ehcau_qp, ib_qp);
	ehca_dbg(qp->context->device, "ehca_qp=%p wqe_count=%x",
		 my_qp, wqe_count);

	asm_sync_mem(); /* serialize GAL register access */
	hipz_update_RQA(my_qp, wqe_count);

	ehca_dbg(qp->context->device, "ehca_qp=%p wqe_count=%x",
		 my_qp, wqe_count);
	return 0;
}

int ehcau_write_swqe(void *wqe, struct ibv_qp *qp, struct ibv_send_wr *send_wr,
		     struct ibv_send_wr **bad_wr)
{
	struct ehcau_qp *my_qp;
	int retcode;
	struct ibv_send_wr *cur_send_wr;
	int wqe_cnt = 0;
	struct ehca_wqe *wqe_p;

        if (!qp) {
		ehca_gen_err("qp=%p check failed line %i", qp, __LINE__);
		return -EFAULT;
	}
	my_qp = container_of(qp, struct ehcau_qp, ib_qp);
        if (!wqe) {
		ehca_gen_err("wqe=%p check failed line %i", wqe, __LINE__);
		return -EFAULT;
	}
        if (!send_wr) {
		ehca_gen_err("send_wr=%p check failed line %i",
			     send_wr, __LINE__);
		return -EFAULT;
	}

	ehca_dbg(qp->context->device, "ehcau_qp=%p wqe=%p send_wr=%p bad_wr=%p",
		 my_qp, wqe, send_wr, bad_wr);

	/* LOCK the QUEUE */
	ehcau_lock(&my_qp->lockvar_s);

	/* loop processes list of send reqs */
	wqe_p = (struct ehca_wqe*)wqe;
	for (cur_send_wr = send_wr; cur_send_wr;
	     cur_send_wr = cur_send_wr->next) {
		/* write a SEND WQE into the QUEUE */
		retcode = write_swqe(my_qp, wqe_p, cur_send_wr, 0);
		/* if something failed, leave loop */
		if (unlikely(retcode)) {
			*bad_wr = cur_send_wr;
			break;
		}
		wqe_cnt++;
		wqe_p++;
		ehca_dbg(qp->context->device, "ehca_qp %p wqe_cnt %d",
			 my_qp, wqe_cnt);
	} /* eof for cur_send_wr */
	retcode = wqe_cnt;

	/* UNLOCK the QUEUE */
	ehcau_unlock(&my_qp->lockvar_s);
	ehca_dbg(qp->context->device, "ehca_qp=%p ret=%i wqe_cnt=%d",
		 my_qp, retcode, wqe_cnt);
	return retcode;
}

static inline int internal_write_rwqe(void *wqe, struct ehcau_qp *my_qp,
				      struct ibv_recv_wr *recv_wr,
				      struct ibv_recv_wr **bad_wr,
				      struct ibv_context *context)
{
	int retcode;
	struct ibv_recv_wr *cur_recv_wr;
	int wqe_cnt = 0;
	struct ehca_wqe *wqe_p;

        if (!wqe) {
		ehca_gen_err("wqe=%p check failed line %i", wqe, __LINE__);
		return -EFAULT;
	}
        if (!recv_wr) {
		ehca_gen_err("recv_wr=%p check failed line %i",
			     recv_wr, __LINE__);
		return -EFAULT;
	}

	ehca_dbg(context->device, "ehca_qp=%p wqe=%p recv_wr=%p bad_wr=%p",
		 my_qp, wqe, recv_wr, bad_wr);

	/* LOCK the QUEUE */
	ehcau_lock(&my_qp->lockvar_r);

	/* loop processes list of send reqs */
	wqe_p = (struct ehca_wqe*)wqe;
	for (cur_recv_wr = recv_wr; cur_recv_wr;
	     cur_recv_wr = cur_recv_wr->next) {
		/* write a RECV WQE into the QUEUE */
		retcode = write_rwqe(&my_qp->ipz_rqueue, wqe_p, cur_recv_wr);
		/* if something failed, leave loop */
		if (unlikely(retcode)) {
			*bad_wr = cur_recv_wr;
			break;
		}
		wqe_cnt++;
		wqe_p++;
		ehca_dbg(context->device, "ehca_qp %p wqe_cnt %d",
			 my_qp, wqe_cnt);
	} /* eof for cur_recv_wr */
	retcode = wqe_cnt;

	/* UNLOCK the QUEUE */
	ehcau_unlock(&my_qp->lockvar_r);
	ehca_dbg(context->device, "ehca_qp=%p ret=%i wqe_cnt=%d",
		 my_qp, retcode, wqe_cnt);
	return retcode;
}

int ehcau_write_rwqe(void *wqe, struct ibv_qp *qp, struct ibv_recv_wr *recv_wr,
		     struct ibv_recv_wr **bad_wr)
{
        if (!qp) {
		ehca_gen_err("qp=%p check failed line %i", qp, __LINE__);
		return -EFAULT;
	}

	return internal_write_rwqe(wqe, container_of(qp, struct ehcau_qp, ib_qp),
				   recv_wr, bad_wr, qp->context);
}

int ehcau_write_srq_rwqe(void *wqe, struct ibv_srq *srq, struct ibv_recv_wr *recv_wr,
			 struct ibv_recv_wr **bad_wr)
{
        if (!srq) {
		ehca_gen_err("srq=%p check failed line %i", srq, __LINE__);
		return -EFAULT;
	}

	return internal_write_rwqe(wqe, container_of(srq, struct ehcau_qp, ib_srq),
				   recv_wr, bad_wr, srq->context);
}

/* eof ehca_ureqs.c */
