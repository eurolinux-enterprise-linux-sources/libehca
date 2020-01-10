/*
 *  IBM eServer eHCA Infiniband device driver for Linux on POWER
 *
 *  Userspace functions
 *
 *  Authors: Christoph Raisch <raisch@de.ibm.com>
 *           Hoang-Nam Nguyen <hnguyen@de.ibm.com>
 *           Stefan Roscher   <stefan.roscher@de.ibm.com>
 *
 *  Copyright (c) 2007 IBM Corporation
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

static int map_queue(struct ipzu_queue *queue,
		     struct ipzu_queue_resp_abiver5 *queue_resp)
{
	queue->offset = queue_resp->offset;
	queue->queue = (u8 *)(queue_resp->queue | queue->offset);
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

struct ibv_cq *ehcau_create_cq_abiver5(struct ibv_context *context, int cqe,
				       struct ibv_comp_channel *channel, int comp_vector)
{
	struct ibv_create_cq cmd;
	struct ehcau_cq *my_cq;
	struct ehcau_create_cq_resp_abiver5 resp;
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
				&resp.ibv_resp,
				sizeof(resp));
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
			&resp.ipz_queue);
	if (ret) {
		ehca_err(context->device, "map_queue() failed cq_num=%x : %s",
			 my_cq->cq_number, strerror(errno));
		goto create_cq_exit1;
	}
	my_cq->galpas = resp.galpas;

	ehca_dbg(context->device, "ehcau_cq=%p cqn=%x token=%x "
		 "ipz_queue.galpa=%p ipz_queue.adr=%p", my_cq,
		 my_cq->cq_number, my_cq->token,
		 (u64*)(unsigned long)my_cq->galpas.kernel.fw_handle,
		 (u64*)my_cq->ipz_queue.queue);
	return &my_cq->ib_cq;

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

static int map_qp_queues_and_galpas5(struct ehcau_qp *my_qp,
				    struct ehca_create_qp_resp_abiver5 *resp,
				    struct h_galpas *galpas,
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
	    map_queue(&my_qp->ipz_rqueue, &resp->ipz_rqueue)) {
		ehca_err(pd->context->device, "map_queue(recv) failed qp_num=%x : %s",
			 my_qp->qp_num, strerror(errno));
		return -ENOMEM;
 	}

 	/* squeue properties */
 	if (HAS_SQ(my_qp) &&
	    map_queue(&my_qp->ipz_squeue, &resp->ipz_squeue)) {
		ehca_err(pd->context->device, "map_queue(send) failed qp_num=%x : %s",
			 my_qp->qp_num, strerror(errno));
		return -ENOMEM;
 	}
 	/* right most cast is required to avoid gcc warning in 32 bit mode */
 	my_qp->galpas = *galpas;
 	return 0;
}

static int map_qp_queues_and_galpas5_20(struct ehcau_qp *my_qp,
				    struct ehca_create_qp_resp_abiver5_20 *resp,
				    struct h_galpas *galpas,
				    struct ibv_pd *pd, struct ibv_context *context)
{
 	/* copy data returned from kernel */
	my_qp->qp_num = resp->qp_num;
	my_qp->token = resp->token;
	my_qp->qp_type = resp->qp_type;
 	my_qp->qkey = resp->qkey;
 	my_qp->real_qp_num = resp->real_qp_num;

 	/* rqueue properties */
 	if (HAS_RQ(my_qp) &&
	    map_queue(&my_qp->ipz_rqueue, &resp->ipz_rqueue)) {
		ehca_err(pd->context->device, "map_queue(recv) failed qp_num=%x : %s",
			 my_qp->qp_num, strerror(errno));
		return -ENOMEM;
 	}

 	/* squeue properties */
 	if (HAS_SQ(my_qp) &&
	    map_queue(&my_qp->ipz_squeue, &resp->ipz_squeue)) {
		ehca_err(pd->context->device, "map_queue(send) failed qp_num=%x : %s",
			 my_qp->qp_num, strerror(errno));
		return -ENOMEM;
 	}

 	/* right most cast is required to avoid gcc warning in 32 bit mode */
 	my_qp->galpas = *galpas;
 	return 0;
}

struct ibv_qp *ehcau_create_qp_abiver5(struct ibv_pd *pd, struct ibv_qp_init_attr *attr)
{
	int ret;
	struct ehcau_qp *my_qp;
	struct ibv_create_qp cmd;
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

	if (ehca_use_abi == abiver5_rhel45) {
		struct ehcau_create_qp_resp_abiver5 resp;
		memset(&resp, 0, sizeof(resp));

		ret = ibv_cmd_create_qp(pd, &my_qp->ib_qp, attr,
					&cmd, sizeof(cmd),
					&resp.ibv_resp, sizeof resp);

		if (ret) {
			ehca_err(pd->context->device, "ibv_cmd_create_qp() failed "
				 "ret=%x pd=%p", ret, pd);
			goto create_qp_exit0;
		}
		if (map_qp_queues_and_galpas5(my_qp, &resp.ehca_resp,
					      &resp.galpas, pd, context))
			goto create_qp_exit1;
	} else if (ehca_use_abi == abiver5_vanilla) {
		struct ehcau_create_qp_resp_abiver5_20 resp;
		memset(&resp, 0, sizeof(resp));

		ret = ibv_cmd_create_qp(pd, &my_qp->ib_qp, attr,
					&cmd, sizeof(cmd),
					&resp.ibv_resp, sizeof resp);

		if (ret) {
			ehca_err(pd->context->device, "ibv_cmd_create_qp() failed "
				 "ret=%x pd=%p", ret, pd);
			goto create_qp_exit0;
		}
		if (map_qp_queues_and_galpas5_20(my_qp, &resp.ehca_resp,
						 &resp.galpas, pd, context))
			goto create_qp_exit1;
	}
	if (attr->send_cq) {
		struct ehcau_cq *cq = container_of(attr->send_cq,
						   struct ehcau_cq, ib_cq);
		cq_assign_qp(cq, my_qp);
		my_qp->send_cq = cq;
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

struct ibv_srq *ehcau_create_srq_abiver5(struct ibv_pd *pd, struct ibv_srq_init_attr *attr)
{
	int ret;
	struct ehcau_qp *my_qp;
	struct ibv_create_srq cmd;
	struct ehcau_create_srq_resp_abiver5 resp;
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
				 &resp.ibv_resp, sizeof resp);

	if (ret) {
		ehca_err(pd->context->device, "ibv_cmd_create_srq() failed "
			 "ret=%i pd=%p", ret, pd);
		goto create_srq_exit0;
	}

	if (map_qp_queues_and_galpas5(my_qp, &resp.ehca_resp, &resp.galpas, pd, context))
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
