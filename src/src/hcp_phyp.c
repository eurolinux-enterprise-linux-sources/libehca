/*
 *  IBM eServer eHCA Infiniband device driver for Linux on POWER
 *
 *   load store abstraction for ehca register access
 *
 *  Authors:  Christoph Raisch <raisch@de.ibm.com>
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

#include "ehca_utools.h"
#include "ehca_galpa.h"

#ifdef __PPC64__
u64 hipz_galpa_load(struct h_galpa galpa, u32 offset)
{
	u64 addr = galpa.fw_handle + offset;
	ehca_gen_dbg("addr=%lx offset=%x ", addr, offset);
	u64 out = *(u64 *) addr;
	ehca_gen_dbg("addr=%lx value=%lx", addr, out);
	return out;
};

void hipz_galpa_store(struct h_galpa galpa, u32 offset, u64 value)
{
	u64 addr = galpa.fw_handle + offset;
	ehca_gen_dbg("addr=%lx offset=%x value=%lx", addr, offset, value);
	*(u64 *) addr = value;
#ifdef EHCA_USE_HCALL
	/* hipz_galpa_load(galpa, offset);*/
#endif
};

#else /* ___PPC64__ the only way to do 8 byte store in 32 bit mode ?*/

inline static void load64(u64 *from, u64 *to)
{
	unsigned long temp = 0;
	asm("ld %2,0(%0)\n\t"
	    "std %2,0(%1)":/*no output*/:"r"(from),"r"(to),"r" (temp):"r0");
}

u64 hipz_galpa_load(struct h_galpa galpa, u32 offset)
{
	void * addr = ((void*)(unsigned long)galpa.fw_handle) + offset;
	ehca_gen_dbg("addr=%p offset=%x ", addr, offset);
	u32 temp[4];
	/* 8 byte align the stack adress*/
	u64 *outadr=(u64*)((((u32)&temp) + 7) & ~7);
	load64(addr, outadr);
	ehca_gen_dbg("addr=%p value=%Lx", addr, *outadr);
	return *outadr;
};

inline static void store64(void *adr, unsigned int datal, unsigned int datah)
{
	unsigned long temp=0;
	asm("sldi %3,%1,32\n\t"
	    "or 0,%3,%0\n\t"
	    "std 0,0(%2)\n" :/* no output */: "r" (datal), "r" (datah),
	    "r" (adr), "r" (temp):"r0");
}

void hipz_galpa_store(struct h_galpa galpa, u32 offset, u64 value)
{
	void * addr = ((void*)(unsigned long)galpa.fw_handle) + offset;
	ehca_gen_dbg("addr=%p offset=%x value=%Lx", addr, offset, value);
	store64(addr, value&0xffffffffULL, value >>32ULL);
#ifdef EHCA_USE_HCALL
	/* hipz_galpa_load(galpa, offset);
	 * synchronize explicitly
	 */
#endif
};

/*
 * signals in 32bit userspace clear high 32 bits of registers,
 * so check high 32 bits are still != 0 after storing; repeat if == 0
 * needs datah != 0 or will loop forever!
 */
inline static void store64_safe(void *adr, unsigned int datal, unsigned int datah)
{
	unsigned long temp=0;
	asm("\n"
	    ".store64_safe_loop:         \n\t"
	    "sldi   %3,%1,32             \n\t"
	    "or     0,%3,%0              \n\t"
	    "std    0,0(%2)              \n\t"
	    "rotrdi 0,0,32               \n\t"
	    "cmpwi  0,0,0                \n\t"
	    "beq    0,.store64_safe_loop \n"
	    :/* no output */: "r" (datal), "r" (datah),
	    "r" (adr), "r" (temp):"r0");
}

void hipz_galpa_enable_Nx_safe(struct h_galpa galpa, u32 offset)
{
	const u64 value = 1ULL << 63; /* constant value with high 32 bit != 0 */
	void * addr = ((void*)(unsigned long)galpa.fw_handle) + offset;
	ehca_gen_dbg("addr=%p offset=%x value=%Lx", addr, offset, value);
	store64_safe(addr, value&0xffffffffULL, value >>32ULL);
#ifdef EHCA_USE_HCALL
	/* hipz_galpa_load(galpa, offset);
	 * synchronize explicitly
	 */
#endif
};

#endif /* ___PPC64__*/
