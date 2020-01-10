/*
 *  IBM eServer eHCA Infiniband device driver for Linux on POWER
 *
 *  Userspace functions
 *
 *  Authors: Khadija Souissi <souissik@de.ibm.com>
 *           Christoph Raisch <raisch@de.ibm.com>
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
 *
 */

#include <infiniband/driver.h>
#include "ehca_uclasses.h"
#include "ehca_utools.h"
#include "ehca_everbs.h"
#include "ipzu_pt_fn.h"
#include "hipz_hw.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/mman.h>
#include <netinet/in.h>

void cq_assign_qp(struct ehcau_cq *cq, struct ehcau_qp *qp)
{
	unsigned int qp_num = qp->real_qp_num;
	unsigned int key = qp_num & (QP_HASHTAB_LEN - 1);

	ehcau_lock(&cq->lockvar);
	LIST_INSERT_HEAD(&cq->qp_hashtab[key], qp, list_entries);
	ehcau_unlock(&cq->lockvar);
	ehca_gen_dbg("cq_num=%x real_qp_num=%x", cq->cq_number, qp_num);
}

int cq_unassign_qp(struct ehcau_cq *cq, unsigned int real_qp_num)
{
	int ret = -EINVAL;
	unsigned int key = real_qp_num & (QP_HASHTAB_LEN - 1);
	struct ehcau_qp *qp;
	ehcau_lock(&cq->lockvar);
	for (qp = cq->qp_hashtab[key].lh_first; qp;
	     qp = qp->list_entries.le_next) {
		if (qp->real_qp_num == real_qp_num) {
			LIST_REMOVE(qp, list_entries);
			ehca_gen_dbg("removed qp from cq .cq_num=%x "
				     "real_qp_num=%x",
				     cq->cq_number, real_qp_num);
			ret = 0;
			break;
		}
	}
	ehcau_unlock(&cq->lockvar);

	return ret;
}

struct ehcau_qp* ehca_cq_get_qp(struct ehcau_cq *cq, int real_qp_num)
{
	struct ehcau_qp *ret = NULL;
	unsigned int key = real_qp_num & (QP_HASHTAB_LEN - 1);
	struct ehcau_qp *qp;
	for (qp = cq->qp_hashtab[key].lh_first; qp;
	     qp = qp->list_entries.le_next) {
		if (qp->real_qp_num == real_qp_num) {
			ret = qp;
			break;
		}
	}
	return ret;
}

int ehcau_query_device(struct ibv_context *context,
		       struct ibv_device_attr *device_attr)
{
	int ret;
	struct ibv_query_device cmd;
	uint64_t raw_fw_ver;

	memset(&cmd, 0, sizeof(cmd));
	ret = ibv_cmd_query_device(context, device_attr, &raw_fw_ver, &cmd,
				   sizeof(cmd));
	if (ret)
		ehca_gen_err("ibv_cmd_query_device() failed, ret=%i", ret);

	return ret;
}

int ehcau_query_port(struct ibv_context *context, uint8_t port,
		     struct ibv_port_attr *attr)
{
	int ret;
	struct ibv_query_port cmd;

	memset(&cmd, 0, sizeof(cmd));
	ret = ibv_cmd_query_port(context, port, attr, &cmd, sizeof(cmd));
	if (ret)
		ehca_gen_err("ibv_cmd_query_port failed ret=%i context=%p "
			     "port=%x", ret, context, port);

	return ret;
}

struct ibv_pd *ehcau_alloc_pd(struct ibv_context *context)
{
	struct ibv_pd *pd;
	struct ibv_alloc_pd cmd;
	struct ibv_alloc_pd_resp resp;
	int ret;

	pd = malloc(sizeof(*pd));
	if (!pd) {
		ehca_gen_err("Out of memory to alloc ehcau_pd "
			     "context=%p", context);
		return NULL;
	}

	memset(pd, 0, sizeof(*pd));
	memset(&cmd, 0, sizeof(cmd));
	memset(&resp, 0, sizeof(resp));
	ret = ibv_cmd_alloc_pd(context, pd,
			       &cmd, sizeof(cmd), &resp, sizeof(resp));

	if (ret) {
		ehca_err(context->device, "ibv_cmd_alloc_pd() failed ret=%i "
			 "context=%p", ret, context);
		free(pd);
		return NULL;
	}

	return pd;
}

int ehcau_dealloc_pd(struct ibv_pd *pd)
{
	int ret;

	ret = ibv_cmd_dealloc_pd(pd);
	if (ret) {
		ehca_err(pd->context->device,
			 "ibv_cmd_dealloc_pd failed ret=%i pd=%p", ret, pd);
		return ret;
	}
	free(pd);

	return 0;
}

static int map_queue(struct ipzu_queue *queue,
		     struct ipzu_queue_resp *queue_resp,
		     u64 file_ofs, struct ibv_context *context)
{
	queue->queue = mmap64(NULL, queue_resp->queue_length,
			      PROT_READ | PROT_WRITE, MAP_SHARED,
			      context->cmd_fd, file_ofs);
	if (queue->queue == MAP_FAILED) {
		ehca_err(context->device, "mmap64() failed %s", strerror(errno));
		return -ENOMEM;
	}
	queue->offset = queue_resp->offset;
	queue->queue = (u8 *)((u64)(unsigned long)queue->queue | queue->offset);
	queue->current_q_addr = queue->queue;
	queue->qe_size = queue_resp->qe_size;
	queue->act_nr_of_sg = queue_resp->act_nr_of_sg;
	queue->queue_length = queue_resp->queue_length;
	queue->pagesize = queue_resp->pagesize;
	queue->toggle_state = queue_resp->toggle_state;
	/* access queue mem to fill page cache */
	memset(queue->queue, 0, queue->queue_length);
	return 0;
}

static u64 map_fw_handle(u64 fw_handle_ofs,
			 u64 file_ofs, struct ibv_context *context)
{
	u64 fw_handle = (u64)(unsigned long)
		mmap64(NULL, EHCA_PAGESIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
		       context->cmd_fd, file_ofs);
	if (fw_handle == (u64)(unsigned long)MAP_FAILED)
		ehca_err(context->device, "mmap64() failed %s", strerror(errno));
	else
		fw_handle |= fw_handle_ofs;

	return fw_handle;
}

static int unmap_queue(struct ipzu_queue *queue)
{
	if (!use_mmap)
		return 0;
	return munmap((u8 *)((u64)queue->queue & ~(kpage_size - 1)), queue->queue_length);
}

static int unmap_fw_handle(u64 fw_handle)
{
	if (!use_mmap)
		return 0;
	return munmap((void*)(fw_handle & ~(kpage_size - 1)), EHCA_PAGESIZE);
}

struct ibv_cq *ehcau_create_cq(struct ibv_context *context, int cqe,
			       struct ibv_comp_channel *channel, int comp_vector)
{
	struct ibv_create_cq cmd;
	struct ehcau_create_cq_resp resp;
	struct ehcau_cq *my_cq;
	int ret;
	int i;

	my_cq = malloc(sizeof(*my_cq));
	if (!my_cq) {
		ehca_err(context->device, "Out of memory context=%p cqe=%x",
			 context, cqe);
		return NULL;
	}

	memset(my_cq, 0, sizeof(*my_cq));
	memset(&cmd, 0, sizeof(cmd));
	memset(&resp, 0, sizeof(resp));
	ret = ibv_cmd_create_cq(context, cqe, channel, comp_vector, &my_cq->ib_cq,
				&cmd, sizeof(cmd),
				&resp.ibv_resp, sizeof(resp));
	if (ret) {
		ehca_err(context->device, "ibv_cmd_create_cq() failed "
			 "ret=%x context=%p cqe=%x", ret, context, cqe);
		goto create_cq_exit0;
	}

	for (i = 0; i < QP_HASHTAB_LEN; i++)
		LIST_INIT(&my_cq->qp_hashtab[i]);

	/* copy data returned from kernel */
	my_cq->cq_number = resp.cq_number;
	my_cq->token = resp.token;
	ret = map_queue(&my_cq->ipz_queue,
			&resp.ipz_queue,
			(u64)(my_cq->token | 0x2000000) << kpage_shift,
			context);
	if (ret) {
		ehca_err(context->device, "map_queue() failed cq_num=%x : %s",
			 my_cq->cq_number, strerror(errno));
		goto create_cq_exit1;
	}
	my_cq->galpas.kernel.fw_handle =
		map_fw_handle(resp.fw_handle_ofs,
			      ((u64)my_cq->token) << kpage_shift,
			      context);
	if (my_cq->galpas.kernel.fw_handle == (u64)(unsigned long)MAP_FAILED) {
		ehca_err(context->device, "map_fw_handle() failed cq_num=%x : %s",
			 my_cq->cq_number, strerror(errno));
		goto create_cq_exit2;
	}

	ehca_dbg(context->device, "ehcau_cq=%p cqn=%x token=%x "
		 "ipz_queue.galpa=%p ipz_queue.adr=%p", my_cq,
		 my_cq->cq_number, my_cq->token,
		 (u64*)(unsigned long)my_cq->galpas.kernel.fw_handle,
		 (u64*)my_cq->ipz_queue.queue);
	return &my_cq->ib_cq;

create_cq_exit2:
	ret = unmap_queue(&my_cq->ipz_queue);
	if (ret)
		ehca_err(context->device, "munmap() failed rc=%i cq_num=%x queue=%p",
			 ret, my_cq->cq_number, my_cq->ipz_queue.queue);

create_cq_exit1:
	ret = ibv_cmd_destroy_cq(&my_cq->ib_cq);
	if (ret)
		ehca_err(context->device, "ibv_cmd_destroy_cq() failed "
			 "ret=%i ehcau_cq=%p cq_num=%x",
			 ret, my_cq, my_cq->cq_number);

create_cq_exit0:
	ehca_err(context->device, "An error has occured context=%p cqe=%x",
		 context, cqe);
	free(my_cq);
	return NULL;
}

int ehcau_destroy_cq(struct ibv_cq *cq)
{
	struct ehcau_cq *my_cq;
	int cq_num;
	int tmp_ret, ret = 0;

	my_cq = container_of(cq, struct ehcau_cq, ib_cq);
	cq_num = my_cq->cq_number;

	tmp_ret = unmap_queue(&my_cq->ipz_queue);
	if (tmp_ret) {
		ehca_err(cq->context->device, "munmap() failed rc=%i cq_num=%x "
			 "queue=%p", tmp_ret, cq_num, my_cq->ipz_queue.queue);
		ret = tmp_ret;
	}

	tmp_ret = unmap_fw_handle((u64)(unsigned long)my_cq->galpas.kernel.fw_handle);
	if (tmp_ret) {
		ehca_err(cq->context->device, "munmap() failed rc=%i cq_num=%x "
			 "fw_handle=%p", tmp_ret, cq_num,
			 (void*)(unsigned long)my_cq->galpas.kernel.fw_handle);
		ret = tmp_ret;
	}

	tmp_ret = ibv_cmd_destroy_cq(cq);
	if (tmp_ret) {
		ehca_err(cq->context->device, "ibv_cmd_destroy_cq() failed "
			 "ret=%i ehcau_cq=%p cq_num=%x", tmp_ret, my_cq, cq_num);
		ret = tmp_ret;
	}

	free(my_cq);

out:
	ehca_gen_dbg("ehcau_cq=%p cq_num=%x ret=%i", my_cq, cq_num, ret);
	return ret;
}

static void unmap_qp_queues_and_galpas(struct ehcau_qp *my_qp, struct ibv_context *context)
{
	if (my_qp->ipz_rqueue.queue &&
	    unmap_queue(&my_qp->ipz_rqueue))
		ehca_err(context->device, "unmap_queue(recv) failed qp_num=%x "
			 "rqueue=%p", my_qp->qp_num, my_qp->ipz_rqueue.queue);

	if (my_qp->ipz_squeue.queue &&
	    unmap_queue(&my_qp->ipz_squeue))
		ehca_err(context->device, "unmap_queue(send) failed qp_num=%x "
			 "squeue=%p", my_qp->qp_num, my_qp->ipz_squeue.queue);

	if (my_qp->galpas.kernel.fw_handle &&
	    unmap_fw_handle((u64)(unsigned long)my_qp->galpas.kernel.fw_handle))
		ehca_err(context->device, "unmap_fw_handle() failed qp_num=%x "
			 "fw_handle=%p", my_qp->qp_num,
			 (void*)(unsigned long)my_qp->galpas.kernel.fw_handle);
}

static int map_qp_queues_and_galpas(struct ehcau_qp *my_qp, struct ehca_create_qp_resp *resp,
				    struct ibv_pd *pd, struct ibv_context *context)
{
	/* copy data returned from kernel */
	my_qp->qp_num = resp->qp_num;
	my_qp->token = resp->token;
	my_qp->qp_type = resp->qp_type;
	my_qp->ext_type = resp->ext_type;
	my_qp->qkey = resp->qkey;
	my_qp->real_qp_num = resp->real_qp_num;

	/* rqueue properties */
	if (HAS_RQ(my_qp) &&
	    map_queue(&my_qp->ipz_rqueue, &resp->ipz_rqueue,
		      (u64)(my_qp->token | 0xA000000) << kpage_shift,
		      context)) {
		ehca_err(pd->context->device, "map_queue(recv) failed qp_num=%x : %s",
			 my_qp->qp_num, strerror(errno));
		return -ENOMEM;
	}

	/* squeue properties */
	if (HAS_SQ(my_qp) &&
	    map_queue(&my_qp->ipz_squeue, &resp->ipz_squeue,
		      (u64)(my_qp->token | 0xC000000) << kpage_shift,
		      context)) {
		ehca_err(pd->context->device, "map_queue(send) failed qp_num=%x : %s",
			 my_qp->qp_num, strerror(errno));
		goto error;
	}

	/* right most cast is required to avoid gcc warning in 32 bit mode */
	my_qp->galpas.kernel.fw_handle =
		map_fw_handle(resp->fw_handle_ofs,
			      (u64)(my_qp->token | 0x8000000) << kpage_shift,
			      context);
	if (my_qp->galpas.kernel.fw_handle == (u64)(unsigned long)MAP_FAILED) {
		ehca_err(pd->context->device, "map_fw_handle() failed qp_num=%x : %s",
			 my_qp->qp_num, strerror(errno));
		goto error;
	}

	return 0;

error:
	unmap_qp_queues_and_galpas(my_qp, context);
	return -ENOMEM;
}

struct ibv_qp *ehcau_create_qp(struct ibv_pd *pd, struct ibv_qp_init_attr *attr)
{
	int ret;
	struct ehcau_qp *my_qp;
	struct ibv_create_qp cmd;
	struct ehcau_create_qp_resp resp;
	struct ibv_context *context;
	struct ehcau_device *my_dev;

	context = pd->context;
	my_dev = container_of(context->device, struct ehcau_device, ibv_dev);
	my_qp = malloc(sizeof(*my_qp));
	if (!my_qp) {
		ehca_err(pd->context->device, "Out of memory to alloc qp pd=%p",
			 pd);
		return NULL;
	}

	memset(my_qp, 0, sizeof(*my_qp));
	memset(&cmd, 0, sizeof(cmd));
	memset(&resp, 0, sizeof(resp));

	my_qp->sq_sig_all = attr->sq_sig_all;

	ret = ibv_cmd_create_qp(pd, &my_qp->ib_qp, attr,
				&cmd, sizeof(cmd),
				&resp.ibv_resp, sizeof(resp));

	if (ret) {
		ehca_err(pd->context->device, "ibv_cmd_create_qp() failed "
			 "ret=%x pd=%p", ret, pd);
		goto create_qp_exit0;
	}

	my_qp->nr_swqes = attr->cap.max_send_wr;

	if (map_qp_queues_and_galpas(my_qp, &resp.ehca_resp, pd, context))
		goto create_qp_exit1;

	if (attr->send_cq) {
		struct ehcau_cq *cq = container_of(attr->send_cq,
						   struct ehcau_cq, ib_cq);
		cq_assign_qp(cq, my_qp);
		my_qp->send_cq = cq;
	}

	if (my_qp->ext_type == EQPT_SRQBASE) {
		struct ehcau_qp *srq = container_of(attr->srq, struct ehcau_qp,
						   ib_srq);

		ehcau_lock(&my_dev->srq_lockvar);
		my_dev->srqs[my_qp->real_qp_num] = srq;
		ehcau_unlock(&my_dev->srq_lockvar);
	} else {
		ehcau_lock(&my_dev->srq_lockvar);
		my_dev->srqs[my_qp->real_qp_num] = NULL;
		ehcau_unlock(&my_dev->srq_lockvar);
	}

	ehca_dbg(pd->context->device, "ehcau_qp=%p "
		 "ipz_queue.galpa=%p ipz_rqueue.adr=%p ipz_squeue.adr=%p",
		 my_qp,
		 (u64*)(unsigned long)my_qp->galpas.kernel.fw_handle,
		 (u64*)my_qp->ipz_rqueue.queue,
		 (u64*)my_qp->ipz_squeue.queue);

	return &my_qp->ib_qp;

create_qp_exit1:
	ret = ibv_cmd_destroy_qp(&my_qp->ib_qp);
	if (ret)
		ehca_err(pd->context->device, "ibv_cmd_destroy_qp() failed "
			 "ret=%i qp=%p qp_num=%x", ret, my_qp, my_qp->qp_num);

create_qp_exit0:
	free(my_qp);
	return NULL;
}

int ehcau_modify_qp(struct ibv_qp *qp, struct ibv_qp_attr *attr,
		    enum ibv_qp_attr_mask attr_mask)
{
	int ret;
	struct ibv_modify_qp cmd;
	struct ehcau_qp *my_qp;
	struct ehca_wqe *wqe = NULL;
	int sq_locked = 0;

	if (!attr_mask) /* nothing to modify */
		return 0;
	my_qp = container_of(qp, struct ehcau_qp, ib_qp);

	if ((attr_mask & IBV_QP_STATE) && attr->qp_state == IBV_QPS_RTS) {
		unsigned int qp_type = -1;
		qp_type = my_qp->qp_type;
		if (IBV_QPT_UD == qp_type) {
			/* lock send queue */
			ehcau_lock(&my_qp->lockvar_s);
			sq_locked = 1;
			/* mark next free wqe */
			wqe=(struct ehca_wqe*)
				my_qp->ipz_squeue.current_q_addr;
			wqe->optype = wqe->wqef = 0xff;
			ehca_dbg(qp->context->device,
				 "qp_num=%x next_free_wqe=%p",
				 my_qp->qp_num, wqe);
		}
	}

	ret = ibv_cmd_modify_qp(qp, attr, attr_mask, &cmd, sizeof(cmd));
	if (ret)
		ehca_err(qp->context->device, "ibv_cmd_modify_qp() failed "
			 "ret=%i qp=%p qp_num=%x", ret, qp, my_qp->qp_num);
	else if (attr_mask & IBV_QP_STATE) {
		switch (attr->qp_state) {
		case IBV_QPS_RESET:
			/* reset s/r queue pointers */
			ipzu_qeit_reset(&my_qp->ipz_rqueue);
			ipzu_qeit_reset(&my_qp->ipz_squeue);
			break;
		case IBV_QPS_RTS:
			if (sq_locked && wqe)
				my_qp->sqerr_purgeflag = ~wqe->wqef;
			break;
		default: /* nothing to do */
			break;
		}
	}

	if (sq_locked) /* unlock send queue */
		ehcau_unlock(&my_qp->lockvar_s);

	if (attr_mask & IBV_QP_QKEY)
		my_qp->qkey = attr->qkey;

	if (attr_mask & IBV_QP_PATH_MTU)
		/* store ld(MTU) */
		my_qp->mtu_shift = attr->path_mtu + 7;

	return ret;

}

int ehcau_destroy_qp(struct ibv_qp *qp)
{
	int ret;
	struct ehcau_qp *my_qp;
	u32 qp_num;

	my_qp = container_of(qp, struct ehcau_qp, ib_qp);
	qp_num = my_qp->qp_num;

	if (my_qp->send_cq) {
		ret = cq_unassign_qp(my_qp->send_cq,
				     my_qp->real_qp_num);
		if (ret) {
			ehca_err(qp->context->device,
				 "Couldn't unassign qp from send_cq "
				 "ret=%x real_qp_num=%x cq_num=%x",
				 ret, my_qp->real_qp_num,
				 my_qp->send_cq->cq_number);
			return ret;
		}
	}

	unmap_qp_queues_and_galpas(my_qp, qp->context);

	ret = ibv_cmd_destroy_qp(qp);
	if (ret)
		ehca_err(qp->context->device,
			 "ibv_cmd_destroy_qp() failed ret=%x "
			 "qp=%p qp_num=%x", ret, qp, qp_num);
	else {
		ehca_gen_dbg("ret=%x qp=%p qp_num=%x", ret, qp, qp_num);
		free(my_qp);
	}

	return ret;
}

struct ibv_srq *ehcau_create_srq(struct ibv_pd *pd, struct ibv_srq_init_attr *attr)
{
	int ret;
	struct ehcau_qp *my_qp;
	struct ibv_create_srq cmd;
	struct ehcau_create_srq_resp resp;
	struct ibv_context *context;

	context = pd->context;
	my_qp = malloc(sizeof(*my_qp));
	if (!my_qp) {
		ehca_err(pd->context->device, "Out of memory to alloc qp pd=%p",
			 pd);
		return NULL;
	}

	memset(my_qp, 0, sizeof(*my_qp));
	memset(&cmd, 0, sizeof(cmd));
	memset(&resp, 0, sizeof(resp));

	ret = ibv_cmd_create_srq(pd, &my_qp->ib_srq, attr,
				 &cmd, sizeof(cmd),
				 &resp.ibv_resp, sizeof(resp));

	if (ret) {
		ehca_err(pd->context->device, "ibv_cmd_create_srq() failed "
			 "ret=%i pd=%p", ret, pd);
		goto create_srq_exit0;
	}

	my_qp->slots_polled = attr->attr.max_wr;
	my_qp->slots_posted = 0;

	if (map_qp_queues_and_galpas(my_qp, &resp.ehca_resp, pd, context))
		goto create_srq_exit1;

	ehca_dbg(pd->context->device, "ehcau_qp=%p "
		 "ipz_queue.galpa=%p ipz_rqueue.adr=%p ipz_squeue.adr=%p",
		 my_qp,
		 (u64*)(unsigned long)my_qp->galpas.kernel.fw_handle,
		 (u64*)my_qp->ipz_rqueue.queue,
		 (u64*)my_qp->ipz_squeue.queue);

	return &my_qp->ib_srq;

create_srq_exit1:
	ret = ibv_cmd_destroy_srq(&my_qp->ib_srq);
	if (ret)
		ehca_err(pd->context->device, "ibv_cmd_destroy_srq() failed "
			 "ret=%i qp=%p qp_num=%x", ret, my_qp, my_qp->qp_num);

create_srq_exit0:
	free(my_qp);
	return NULL;
}

int ehcau_modify_srq(struct ibv_srq *srq, struct ibv_srq_attr *attr,
		     enum ibv_srq_attr_mask attr_mask)
{
	int ret;
	struct ibv_modify_srq cmd;
	struct ehcau_qp *my_qp;

	if (!attr_mask) /* nothing to modify */
		return 0;

	my_qp = container_of(srq, struct ehcau_qp, ib_srq);

	ret = ibv_cmd_modify_srq(srq, attr, attr_mask, &cmd, sizeof(cmd));
	if (ret)
		ehca_err(srq->context->device, "ibv_cmd_modify_srq() failed "
			 "ret=%i srq=%p qp_num=%x", ret, srq, my_qp->qp_num);

	return ret;

}

int ehcau_query_srq(struct ibv_srq *srq, struct ibv_srq_attr *srq_attr)
{
	int ret;
	struct ehcau_qp *my_qp;
	struct ibv_query_srq cmd;

	my_qp = container_of(srq, struct ehcau_qp, ib_srq);

	ret =  ibv_cmd_query_srq(srq, srq_attr, &cmd, sizeof(cmd));
	if (ret)
		ehca_err(srq->context->device,
			 "ibv_cmd_query_srq() failed ehcau_qp=%p qp_num=%x "
			 "ret=%i", my_qp, my_qp->qp_num, ret);

	return ret;
}

int ehcau_destroy_srq(struct ibv_srq *srq)
{
	int i, ret;
	struct ehcau_qp *my_qp;
	u32 qp_num;
	struct ehcau_device *my_dev;

	my_dev = container_of(srq->context->device, struct ehcau_device,
			      ibv_dev);

	my_qp = container_of(srq, struct ehcau_qp, ib_srq);
	qp_num = my_qp->qp_num;

	unmap_qp_queues_and_galpas(my_qp, srq->context);

	ret = ibv_cmd_destroy_srq(srq);
	if (ret) {
		ehca_err(srq->context->device,
			 "ibv_cmd_destroy_qp() failed ret=%i "
			 "srq=%p qp_num=%x", ret, srq, qp_num);
		goto out;
	}

	ehcau_lock(&my_dev->srq_lockvar);
	for (i = 0; i < QP_PER_DEVICE; i++) {
		if (my_dev->srqs[i] == my_qp)
			my_dev->srqs[i] = NULL;
	}
	ehcau_unlock(&my_dev->srq_lockvar);

	ehca_gen_dbg("ret=%i srq=%p qp_num=%x", ret, srq, qp_num);
	free(my_qp);

out:
	return ret;
}

static inline int ib_width_enum_to_int(unsigned int width)
{
	/* enum ib_port_width exists only in kernel space
	 * since this is the only place using ib_port_width
	 * we use the hardcoded values
	 */
	switch (width) {
	case 1:  return  1;
	case 2:  return  4;
	case 4:  return  8;
	case 8:  return 12;
	default: return -1;
	}
}

int ehcau_calc_ipd(struct ibv_context *context, struct ibv_port_attr *pa,
		   enum ibv_rate path_rate, u32 *ipd)
{
	int path = ibv_rate_to_mult(path_rate);
	int link;

	if (path_rate == IBV_RATE_MAX) {
		*ipd = 0;
		return 0;
	}

	if (unlikely(path < 0)) {
		ehca_err(context->device, "Invalid static rate! path_rate=%x",
			 path_rate);
		return -EINVAL;
	}

	link = ib_width_enum_to_int(pa->active_width) * pa->active_speed;
	if (unlikely(link <= 0)) {
		ehca_err(context->device, "Invalid port width or speed  "
			 "width=%i speed=%i", pa->active_width, pa->active_speed);
		link = 12;
	}

	if (path >= link)
		/* no need to throttle if path faster than link */
		*ipd = 0;
	else
		/* IPD = round((link / path) - 1) */
		*ipd = ((link + (path >> 1)) / path) - 1;

	return 0;
}

struct ibv_ah *ehcau_create_ah(struct ibv_pd *pd, struct ibv_ah_attr *attr)
{
	struct ehcau_av *my_av;
	struct ibv_port_attr pa;
	int rc;
	u32 ipd;

	rc = ehcau_query_port(pd->context, attr->port_num, &pa);
	if (rc) {
		ehca_err(pd->context->device, "query_port() failed "
			 "rc=%i context=%p port_num=%x",
			 rc, pd->context, attr->port_num);
		return NULL;
	}

	my_av = malloc(sizeof(*my_av));
	if (!my_av) {
		ehca_err(pd->context->device, "no address handle");
		return NULL;
	}

	memset(my_av, 0, sizeof(*my_av));
	my_av->av.sl = attr->sl;
	my_av->av.dlid = ntohs(attr->dlid);
	my_av->av.slid_path_bits = attr->src_path_bits;

	if (ehcau_calc_ipd(pd->context, &pa, attr->static_rate, &ipd)) {
		free(my_av);
		return NULL;
	}
	my_av->av.ipd = ipd;

	/* ah_attr don't provide this, so use port maximum */
	my_av->av.pmtu = pa.max_mtu;

	my_av->av.lnh = attr->is_global;
	my_av->av.grh.word_0 |= EHCA_BMASK_SET(GRH_IPVERSION_MASK, 6);
	my_av->av.grh.word_0 |= EHCA_BMASK_SET(GRH_TCLASS_MASK,
					       attr->grh.traffic_class);
	my_av->av.grh.word_0 |= EHCA_BMASK_SET(GRH_FLOWLABEL_MASK,
					       attr->grh.flow_label);
	my_av->av.grh.word_0 |= EHCA_BMASK_SET(GRH_HOPLIMIT_MASK,
					       attr->grh.hop_limit);
	my_av->av.grh.word_0 |= EHCA_BMASK_SET(GRH_NEXTHEADER_MASK, 0x1B);
	/* IB transport */
	my_av->av.grh.word_0 = be64_to_cpu(my_av->av.grh.word_0);

	/* set sgid in grh.word_1 */
	if (attr->is_global) {
		union ibv_gid gid;
		memset(&gid, 0, sizeof(gid));
		rc = ibv_query_gid(pd->context, attr->port_num,
				   attr->grh.sgid_index, &gid);
		if (rc) {
			ehca_err(pd->context->device, "ibv_query_gid() failed "
				 "rc=%i context=%p port_num=%x "
				 "sgid_index=%x",
				 rc, pd->context, attr->port_num,
				 attr->grh.sgid_index);
			free(my_av);
			return NULL;
		}
		memcpy(&my_av->av.grh.word_1, &gid, sizeof(gid));
	}

	/* dgid comes in grh.word_3 */
	memcpy(&my_av->av.grh.word_3, &attr->grh.dgid, sizeof(attr->grh.dgid));

	return &my_av->ib_ah;
}

int ehcau_destroy_ah(struct ibv_ah *ah)
{
	ehca_dbg(ah->context->device, "ah=%p", ah);
	free(ah);

	return 0;
}

int ehcau_attach_mcast(struct ibv_qp *qp, union ibv_gid *gid, uint16_t lid)
{
	int ret;
	struct ehcau_qp *my_qp;
	u32 qp_num;

	my_qp = container_of(qp, struct ehcau_qp, ib_qp);
	qp_num = my_qp->qp_num;

	ret = ibv_cmd_attach_mcast(qp, gid, lid);
	if (ret)
		ehca_err(qp->context->device, "ehcau_qp=%p qp_num=%x "
			 "ibv_cmd_attach_mcast() failed "
			 "ret=%i", my_qp, my_qp->qp_num, ret);

	return ret;
}

int ehcau_detach_mcast(struct ibv_qp *qp, union ibv_gid *gid, uint16_t lid)
{
	int ret;
	struct ehcau_qp *my_qp;
	u32 qp_num;

	my_qp = container_of(qp, struct ehcau_qp, ib_qp);
	qp_num = my_qp->qp_num;

	ret = ibv_cmd_detach_mcast(qp, gid, lid);
	if (ret)
		ehca_err(qp->context->device, "ehcau_qp=%p qp_num=%x "
			 "ibv_cmd_detach_mcast() failed "
			 "ret=%i", my_qp, my_qp->qp_num, ret);

	return ret;
}


int ehcau_query_qp(struct ibv_qp *qp, struct ibv_qp_attr *qp_attr,
		   enum ibv_qp_attr_mask attr_mask,
		   struct ibv_qp_init_attr* init_attr)
{
	int ret;
	struct ehcau_qp *my_qp;
	struct ibv_query_qp cmd;

	my_qp = container_of(qp, struct ehcau_qp, ib_qp);

        ret =  ibv_cmd_query_qp(qp, qp_attr, attr_mask, init_attr,
				&cmd, sizeof(cmd));
	if (ret)
		ehca_err(qp->context->device, "ehcau_qp=%p qp_num=%x "
			 "ibv_cmd_query_() failed "
			 "ret=%i", my_qp, my_qp->qp_num, ret);

	return ret;
}


int ehcau_query_qp_da(struct ibv_qp *qp, struct ehcau_qp_attr_da *qp_attr)
{
	struct ehcau_qp *my_qp;

	my_qp = container_of(qp, struct ehcau_qp, ib_qp);

	qp_attr->send_wqe_size = my_qp->ipz_squeue.qe_size;
	qp_attr->send_queue_length = my_qp->ipz_squeue.queue_length;
	qp_attr->send_queue_ptr = my_qp->ipz_squeue.queue;
	qp_attr->recv_wqe_size = my_qp->ipz_rqueue.qe_size;
	qp_attr->recv_queue_length = my_qp->ipz_rqueue.queue_length;
	qp_attr->recv_queue_ptr = my_qp->ipz_rqueue.queue;

	ehca_dbg(qp->context->device, "qp=%p qp_num=%x "
		 "send_wqe_size=%x send_queue_size=%lx send_queue_ptr=%p "
		 "recv_wqe_size=%x recv_queue_size=%lx recv_queue_ptr=%p",
		 qp, my_qp->qp_num,
		 qp_attr->send_wqe_size,
		 (unsigned long)qp_attr->send_queue_length,
		 qp_attr->send_queue_ptr,
		 qp_attr->recv_wqe_size,
		 (unsigned long)qp_attr->recv_queue_length,
		 qp_attr->recv_queue_ptr);
	return 0;
}

int ehcau_query_srq_da(struct ibv_srq *srq, struct ehcau_qp_attr_da *qp_attr)
{
	struct ehcau_qp *my_qp;

	my_qp = container_of(srq, struct ehcau_qp, ib_srq);

	qp_attr->send_wqe_size = qp_attr->send_queue_length = 0;
	qp_attr->send_queue_ptr = NULL;
	qp_attr->recv_wqe_size = my_qp->ipz_rqueue.qe_size;
	qp_attr->recv_queue_length = my_qp->ipz_rqueue.queue_length;
	qp_attr->recv_queue_ptr = my_qp->ipz_rqueue.queue;

	ehca_dbg(srq->context->device, "srq=%p qp_num=%x "
		 "recv_wqe_size=%x recv_queue_size=%lx recv_queue_ptr=%p",
		 srq, my_qp->qp_num,
		 qp_attr->recv_wqe_size,
		 (unsigned long)qp_attr->recv_queue_length,
		 qp_attr->recv_queue_ptr);
	return 0;
}

/* eof ehca_umain.c */
