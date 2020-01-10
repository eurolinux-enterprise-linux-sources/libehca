/*
 *  IBM eServer eHCA Infiniband device driver for Linux on POWER
 *
 *  MR/MW functions
 *
 *  Authors: Dietmar Decker <ddecker@de.ibm.com>
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

#include <stdlib.h>
#include <infiniband/driver.h>
#include "ehca_utools.h"


struct ibv_mr *ehcau_reg_mr(struct ibv_pd *pd,
			    void *addr,
			    size_t length,
			    enum ibv_access_flags access)
{
	int ret;
	struct ibv_mr *mr;
	struct ibv_reg_mr cmd;

	ehca_dbg(pd->context->device, "pd=%p addr=%p length=%lx access=%x",
		 pd, addr, (unsigned long)length, access);

	mr = malloc(sizeof(struct ibv_mr));
	if (!mr) {
		ehca_err(pd->context->device, "no memory");
		return NULL;
	}

#ifdef IBV_CMD_REG_MR_HAS_RESP_PARAMS
	{
		struct ibv_reg_mr_resp resp;

		ret = ibv_cmd_reg_mr(pd, addr, length, (u64)(unsigned long)addr,
				     access, mr, &cmd, sizeof(struct ibv_reg_mr),
				     &resp, sizeof(struct ibv_reg_mr_resp));
	}
#else
	ret = ibv_cmd_reg_mr(pd, addr, length, (u64)(unsigned long)addr,
			     access, mr, &cmd, sizeof(struct ibv_reg_mr));
#endif
	if (ret) {
		ehca_err(pd->context->device, "ibv_cmd_reg_mr ret=%i", ret);
		free(mr);
		return NULL;
	}

	ehca_dbg(pd->context->device, "mr=%p", mr);
	return mr;
}


int ehcau_dereg_mr(struct ibv_mr *mr)
{
	int ret;

	ehca_dbg(mr->context->device, "mr=%p", mr);

	ret = ibv_cmd_dereg_mr(mr);
	if (ret) {
		ehca_err(mr->context->device, "ibv_cmd_dereg_mr failed, ret=%i",
			 ret);
		return ret;
	}

	free(mr);
	ehca_gen_dbg("mr=%p", mr);
	return 0;
}
