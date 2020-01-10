/*
 *  IBM eServer eHCA Infiniband device driver for Linux on POWER
 *
 *  Some helper macros with assembler instructions
 *
 *  Authors: Khadija Souissi <souissik@de.ibm.com>
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


#ifndef __EHCA_ASM_H__
#define __EHCA_ASM_H__

#if defined(CONFIG_PPC_PSERIES) || defined (__PPC64__) || defined (__PPC__)

#define clear_cacheline(adr) __asm__ __volatile("dcbz 0,%0"::"r"(adr))
/* rmb() in 32-bit mode does a full sync, while we need a lwsync */
#define lwsync()  __asm__ __volatile__ ("lwsync" : : : "memory")

#define mftb() ({ unsigned long rval; \
                  asm volatile("mftb %0" : "=r" (rval)); rval; })

#define asm_sync_mem() __asm__ __volatile__ ("sync" : : : "memory")

#else
#error "invalid platform"
#endif

#endif /* __EHCA_ASM_H__ */
