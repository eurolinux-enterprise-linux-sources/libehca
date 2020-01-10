/*
 *  IBM eServer eHCA Infiniband device driver for Linux on POWER
 *
 *  pSeries interface definitions
 *
 *  Authors: Waleri Fomin <fomin@de.ibm.com>
 *           Christoph Raisch <raisch@de.ibm.com>
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

#ifndef __EHCA_GALPA_H__
#define __EHCA_GALPA_H__

/*
 * eHCA page (mapped into p-memory)
 * resource to access eHCA register pages in CPU address space
 */
struct h_galpa {
	u64 fw_handle;
	/* for pSeries this is a 64bit memory address where
	 * I/O memory is mapped into CPU address space (kv)
	 */
};

/* resource to access eHCA address space registers, all types */
struct h_galpas {
	u32 pid;		/*PID of userspace galpa checking */
	struct h_galpa user;	/* user space accessible resource,
				 * set to 0 if unused
				 */
	struct h_galpa kernel;	/* kernel space accessible resource,
				 * set to 0 if unused
				 */
};
/* store value at offset into galpa, will be inline function */
void hipz_galpa_store(struct h_galpa galpa, u32 offset, u64 value);

/* return value from offset in galpa, will be inline function */
u64 hipz_galpa_load(struct h_galpa galpa, u32 offset);

/* signals clear upper 32 bits of stored value in 32bit uspace,
 * so we need special "safe" functions for CQx_Ny */
#ifdef __PPC64__
#define hipz_galpa_enable_Nx_safe(galpa, offset) \
	hipz_galpa_store(galpa, offset, 1ULL << 63)
#else
void hipz_galpa_enable_Nx_safe(struct h_galpa galpa, u32 offset);
#endif

#endif /* __EHCA_GALPA_H__ */
