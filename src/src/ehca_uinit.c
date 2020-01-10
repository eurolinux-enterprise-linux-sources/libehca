/*
 *  IBM eServer eHCA Infiniband device driver for Linux on POWER
 *
 *  Initialization for user verbs
 *
 *  Authors: Khadija Souissi <souissik@de.ibm.com>
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

#if HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <sys/mman.h>
#include <pthread.h>
#include <sys/utsname.h>
#ifndef HAVE_IBV_READ_SYSFS_FILE
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#endif

#ifndef HAVE_IBV_REGISTER_DRIVER
#include <sysfs/libsysfs.h>
#endif

#include "ehca_uclasses.h"

/* 1/default: call mmap() to map queue pages and firmware
 * 0: for older kernels just copy adrs given by driver
 */
int use_mmap = 1;
int ehca_use_abi = 0;
unsigned long kpage_size = 0x1000; /* set properly by module init */
unsigned long kpage_shift = 0; 	/* set properly by module init */

static struct ibv_context_ops ehcau_ctx_ops = {
	.query_device  = ehcau_query_device,
	.query_port    = ehcau_query_port,
	.alloc_pd      = ehcau_alloc_pd,
	.dealloc_pd    = ehcau_dealloc_pd,
	.reg_mr        = ehcau_reg_mr,
	.dereg_mr      = ehcau_dereg_mr,
	.create_cq     = ehcau_create_cq,
	.poll_cq       = ehcau_poll_cq,
	.req_notify_cq = ehcau_req_notify_cq,
	.cq_event      = NULL,	/* TODO: Not implemented yet */
	.destroy_cq    = ehcau_destroy_cq,
	.create_qp     = ehcau_create_qp,
	.modify_qp     = ehcau_modify_qp,
	.query_qp      = ehcau_query_qp,
	.destroy_qp    = ehcau_destroy_qp,
	.create_srq    = ehcau_create_srq,
	.modify_srq    = ehcau_modify_srq,
	.query_srq     = ehcau_query_srq,
	.destroy_srq   = ehcau_destroy_srq,
	.post_srq_recv = ehcau_post_srq_recv,
	.post_send     = ehcau_post_send,
	.post_recv     = ehcau_post_recv,
	.create_ah     = ehcau_create_ah,
	.destroy_ah    = ehcau_destroy_ah,
	.attach_mcast  = ehcau_attach_mcast,
	.detach_mcast  = ehcau_detach_mcast
};

static struct ibv_context *ehcau_alloc_context(struct ibv_device *ibdev,
					       int cmd_fd)
{
	struct ehcau_context *my_context;
	struct ibv_get_context cmd;
	struct ibv_get_context_resp resp;

	ehca_dbg(ibdev, "device=%p cmd_fd=%x", ibdev, cmd_fd);

	my_context = (struct ehcau_context *)malloc(sizeof(*my_context));
	if (!my_context)
		return NULL;
	memset(my_context, 0, sizeof(*my_context));

	my_context->ibv_ctx.cmd_fd = cmd_fd;

	memset(&cmd, 0, sizeof(cmd));
	memset(&resp, 0, sizeof(resp));
	if (ibv_cmd_get_context(&my_context->ibv_ctx, &cmd,
				sizeof(cmd), &resp, sizeof(resp))) {
		ehca_err(ibdev, "ibv_cmd_get_context() failed device=%p",
			 ibdev);
		goto alloc_context_exit0;
	}

	my_context->ibv_ctx.device = ibdev;
	my_context->ibv_ctx.ops = ehcau_ctx_ops;

	ehca_dbg(ibdev, "retcode=context=%p device=%p",
		 &my_context->ibv_ctx, ibdev);
	return &my_context->ibv_ctx;

alloc_context_exit0:
	free(my_context);
	ehca_dbg(ibdev, "retcode=context=NULL device=%p", ibdev);
	return NULL;
}

static void ehcau_free_context(struct ibv_context *ibctx)
{
	struct ehcau_context *my_context;
	ehca_dbg(ibctx->device, "context=%p", ibctx);
	if (!ibctx)
		ehca_err(ibctx->device, "context pointer is NULL");
	else {
		my_context = container_of(ibctx, struct ehcau_context, ibv_ctx);
		free(my_context);
	}
	ehca_dbg(ibctx->device, "context=%p", ibctx);
}

static char *str_strip(char *str)
{
	char *last;
	/* strip left white spaces */
	while (*str == ' ' || *str == '\t' || *str == '\n' || *str == '\r')
		str++;
	/* strip right white spaces */
	last = str + (strlen(str) - 1);
	while (last>=str &&
	       (*last == ' ' || *last == '\t' ||
		*last == '\n' || *last == '\r'))
		last--;
	*(last + 1) = 0;
	return str;
}

static struct ibv_device_ops ehcau_dev_ops = {
	.alloc_context = ehcau_alloc_context,
	.free_context  = ehcau_free_context
};

/*
 * Keep a private implementation of HAVE_IBV_READ_SYSFS_FILE to handle
 * old versions of libibverbs that didn't implement it.  This can be
 * removed when libibverbs 1.0.3 or newer is available "everywhere."
 */
#ifndef HAVE_IBV_READ_SYSFS_FILE
static int ibv_read_sysfs_file(const char *dir, const char *file,
			       char *buf, size_t size)
{
	char path[256];
	int fd;
	int len;

	snprintf(path, sizeof path, "%s/%s", dir, file);

	fd = open(path, O_RDONLY);
	if (fd < 0)
		return -1;

	len = read(fd, buf, size);

	close(fd);

	if (len > 0 && buf[len - 1] == '\n')
		buf[--len] = '\0';

	return len;
}
#endif /* HAVE_IBV_READ_SYSFS_FILE */

static struct ibv_device *ehca_driver_init(const char *uverbs_sys_path,
					   int abi_version)
{
	struct ehcau_device *my_dev = NULL;
	char value[64];
	int num_ports  = 0;
	struct utsname systeminfo;
	int kernel_release;
	int kernel_ext_release = 0;

	ehca_gen_dbg("libehca v" LIBEHCA_VERSION " (ABI version " ABI_VERSION_STRING ")");
	ehca_gen_dbg("uverbs_sys_path=%s", uverbs_sys_path);

	if (uname (&systeminfo) == -1)
		printf("libehca was not able to get systeminfo\n");
	sscanf(systeminfo.release, "2.6.%i.%i", &kernel_release, &kernel_ext_release);

	if (ibv_read_sysfs_file(uverbs_sys_path, "device/name",
				value, sizeof value) < 0)
		return NULL;

	if (strcmp("lhca", str_strip(value)))
		return NULL;

	if (ibv_read_sysfs_file(uverbs_sys_path, "device/num_ports",
				value, sizeof value) < 0)
		return NULL;

	sscanf(value, "%i", &num_ports);

	if (num_ports < 1)
		return NULL;

	/* here we've got our dev */
	my_dev = malloc(sizeof *my_dev);
	if (!my_dev) {
		fprintf(stderr, "Fatal: couldn't allocate device for %s\n",
			uverbs_sys_path);
		abort();
	}

	if (abi_version <= 5) {
		if (kernel_release == 9){
			ehca_use_abi = abiver5_rhel45;
			ehca_gen_dbg("Using abiver5_20 for queue and galpas mapping");
		} else {
			ehca_use_abi = abiver5_vanilla;
			ehca_gen_dbg("Using abiver5 for queue and galpas mapping");
		}
		use_mmap = 0;
		ehcau_ctx_ops.create_cq  = ehcau_create_cq_abiver5;
		ehcau_ctx_ops.create_qp  = ehcau_create_qp_abiver5;
		ehcau_ctx_ops.create_srq  = ehcau_create_srq_abiver5;

	} else if (abi_version == 6) {
		if (kernel_release == 18) {
			ehca_use_abi = abiver6_rhel51;
			ehcau_ctx_ops.create_cq  = ehcau_create_cq_abiver7;
		} else {
			ehca_use_abi = abiver6_vanilla;
			ehcau_ctx_ops.create_cq  = ehcau_create_cq_abiver6;
		}
		ehcau_ctx_ops.create_qp  = ehcau_create_qp_abiver6;
		ehcau_ctx_ops.create_srq  = ehcau_create_srq_abiver7;
		ehcau_ctx_ops.destroy_srq  = ehcau_destroy_srq_abiver7;
		ehca_gen_dbg("Using abiver6 for queue and galpas mapping");
	} else if (abi_version == 7) {
		ehcau_ctx_ops.create_cq  = ehcau_create_cq_abiver7;
		ehcau_ctx_ops.create_qp  = ehcau_create_qp_abiver7;
		ehcau_ctx_ops.create_srq  = ehcau_create_srq_abiver7;
		ehcau_ctx_ops.destroy_srq  = ehcau_destroy_srq_abiver7;
		ehca_gen_dbg("Using abiver7 for queue and galpas mapping");
	} else if (abi_version != ABI_VERSION) {
		ehca_gen_dbg("ABI version mismatch: kernel=%d libehca=%d",
			     abi_version, ABI_VERSION);
		return NULL;
	}

	my_dev->ibv_dev.ops = ehcau_dev_ops;

	ehca_gen_dbg("ehcau_device=%p", my_dev);

	return &my_dev->ibv_dev;
}

/*
 * Export the old libsysfs sysfs_class_device-based driver entry point
 * if libsysfs headers are installed.  It doesn't hurt to export it,
 * even if libibverbs is new enough not to use it; but if libsysfs
 * headers are not installed, we can assume that the version of
 * libibverbs we are building against is new enough not to use
 * openib_driver_init().
 */
#ifndef HAVE_IBV_REGISTER_DRIVER
struct ibv_device *openib_driver_init(struct sysfs_class_device *sysdev)
{
	int abi_ver = 0;
	char value[8];

	if (ibv_read_sysfs_file(sysdev->path, "abi_version",
				value, sizeof value) > 0)
		abi_ver = strtol(value, NULL, 10);

	return ehca_driver_init(sysdev->path, abi_ver);
}
#endif

int libehca_trace_on = 0; /* default is no debug */
#define CFG_VAR_TRACE_ON    "LIBEHCA_TRACE_ON"

void __attribute__ ((constructor)) ehcau_init(void)
{
	unsigned long tmp;
	char *value = getenv(CFG_VAR_TRACE_ON);
	if (value) {
		value = str_strip(value);
		libehca_trace_on = (*value) - '0';
	}
	kpage_size = sysconf(_SC_PAGESIZE);

	tmp = kpage_size;
	while ((tmp & 0x1) == 0){
		kpage_shift++;
		tmp = tmp >> 1;
	}

#ifdef HAVE_IBV_REGISTER_DRIVER
	ibv_register_driver("ehca", ehca_driver_init);
#endif
}

/* eof ehca_uinit.c */
